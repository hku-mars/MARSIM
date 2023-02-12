
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <quadrotor_dynamics.hpp>
#include <std_msgs/Float32MultiArray.h>

using namespace std;
using namespace Eigen;

std_msgs::Float32MultiArray RPM_msg;
Vector4d RPM_input;
void RPMCallbck(const std_msgs::Float32MultiArray& msg)
{
    RPM_msg = msg;
    RPM_input<< RPM_msg.data[0], RPM_msg.data[1],RPM_msg.data[2],RPM_msg.data[3];
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "quadrotor_dynamics");

    ros::NodeHandle n("~");


    double init_x, init_y, init_z,mass;
    double simulation_rate;
    std::string quad_name;
    n.param("mass", mass, 0.9);
    n.param("init_state_x", init_x, 0.0);
    n.param("init_state_y", init_y, 0.0);
    n.param("init_state_z", init_z, 1.0);
    n.param("simulation_rate", simulation_rate, 200.0);
    n.param("quadrotor_name", quad_name, std::string("quadrotor"));

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 10);
    // ros::Subscriber cmd_sub = n.subscribe("cmd", 100, &cmd_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber rpm_sub = n.subscribe("cmd_RPM", 100, RPMCallbck);


    Matrix3d Internal_mat;
    Internal_mat << 2.64e-3,0,0,
                    0,2.64e-3,0,
                    0,0,4.96e-3;
    quadrotor_dynamics quadrotor(mass, Internal_mat);

    Vector3d init_pos;
    Vector4d init_q, actuator;
    init_pos << init_x, init_y, init_z;
    init_q << 1,0,0.0,0.0;
    quadrotor.init(init_pos , init_q);
    actuator << 0.0,0.0,0.0,0.0;
    // quadrotor.setActuatoroutput(actuator);

    ros::Rate rate(simulation_rate);
    rate.sleep();

    ros::Time last_time = ros::Time::now();
    while(n.ok())
    {
        ros::spinOnce();
        ROS_INFO("RPM_input = %.2f,%.2f,%.2f,%.2f",RPM_input(0),RPM_input(1),RPM_input(2),RPM_input(3));
        quadrotor.setRPM(RPM_input);

        ros::Time now_time = ros::Time::now();
        // ROS_INFO("dt = %lf", (now_time-last_time).toSec());
        quadrotor.step_forward((now_time-last_time).toSec());
        last_time = now_time;

        //publish odometry
        nav_msgs::Odometry odom;
        odom.header.frame_id = "world";
        odom.header.stamp = now_time;
        Vector3d pos,vel,acc,angular_vel,angular_vel_world;
        pos = quadrotor.getPos();
        vel = quadrotor.getVel();
        acc = quadrotor.getAcc();
        angular_vel = quadrotor.getAngularVel();
        Vector4d quat;
        quat = quadrotor.getQuat();
        Matrix3d R_body2world;
        angular_vel_world = R_body2world*angular_vel;
        R_body2world = quadrotor.getR();
        odom.pose.pose.position.x = pos(0);
        odom.pose.pose.position.y = pos(1);
        odom.pose.pose.position.z = pos(2);
        odom.pose.pose.orientation.w = quat(0);
        odom.pose.pose.orientation.x = quat(1);
        odom.pose.pose.orientation.y = quat(2);
        odom.pose.pose.orientation.z = quat(3);
        odom.twist.twist.linear.x = vel(0);
        odom.twist.twist.linear.y = vel(1);
        odom.twist.twist.linear.z = vel(2);
        odom.twist.twist.angular.x = angular_vel_world(0);
        odom.twist.twist.angular.y = angular_vel_world(1);
        odom.twist.twist.angular.z = angular_vel_world(2);
        odom_pub.publish(odom);

        ROS_INFO("Odom = %f,%f,%f, %f,%f,%f,%f",pos(0),pos(1),pos(2),quat(0),quat(1),quat(2),quat(3));

        //imu generate
        sensor_msgs::Imu imu_msg;
        imu_msg.header.frame_id = "/" + quad_name;
        imu_msg.header.stamp = now_time;
        imu_msg.orientation.w = quat(0);
        imu_msg.orientation.x = quat(1);
        imu_msg.orientation.y = quat(2);
        imu_msg.orientation.z = quat(3);
        imu_msg.angular_velocity.x = angular_vel(0);
        imu_msg.angular_velocity.y = angular_vel(1);
        imu_msg.angular_velocity.z = angular_vel(2);
        acc = R_body2world.inverse() * (acc + Eigen::Vector3d(0,0,-9.8));
        imu_msg.linear_acceleration.x = acc(0);
        imu_msg.linear_acceleration.y = acc(1);
        imu_msg.linear_acceleration.z = acc(2);
        imu_pub.publish(imu_msg);

        rate.sleep();
    }

    return 0;
  
}