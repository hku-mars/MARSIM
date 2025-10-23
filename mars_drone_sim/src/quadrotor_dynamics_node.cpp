
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <quadrotor_dynamics.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std;
using namespace Eigen;

class QuadrotorDynamicsNode : public rclcpp::Node
{
public:
    QuadrotorDynamicsNode() : Node("quadrotor_dynamics_node"), quadrotor(0.9, Matrix3d::Identity())
    {
        rpm_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "cmd_RPM", 10, std::bind(&QuadrotorDynamicsNode::rpm_callback, this, std::placeholders::_1));
        
        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        
        timer = this->create_wall_timer(
            std::chrono::milliseconds(5), std::bind(&QuadrotorDynamicsNode::dynamics_loop, this));
        
        RPM_input.setZero();
        last_time = this->now();
    }

    void initialize_quadrotor(double mass, const Matrix3d& Internal_mat, const Vector3d& init_pos, const Vector4d& init_q, const std::string& name)
    {
        quadrotor = quadrotor_dynamics(mass, Internal_mat);
        quadrotor.init(init_pos, init_q);
        quad_name = name;
        last_time = this->now();
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr rpm_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::TimerBase::SharedPtr timer;
    
    Vector4d RPM_input;

    void rpm_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        RPM_input << msg->data[0], msg->data[1], msg->data[2], msg->data[3];
    }

    void dynamics_loop()
    {
        RCLCPP_INFO(this->get_logger(), "RPM_input = %.2f,%.2f,%.2f,%.2f", RPM_input(0), RPM_input(1), RPM_input(2), RPM_input(3));
        quadrotor.setRPM(RPM_input);

        auto now_time = this->now();
        double dt = (now_time - last_time).seconds();
        quadrotor.step_forward(dt);
        last_time = now_time;

        // Publish odometry
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = "world";
        odom.header.stamp = now_time;
        
        Vector3d pos = quadrotor.getPos();
        Vector3d vel = quadrotor.getVel();
        Vector3d acc = quadrotor.getAcc();
        Vector3d angular_vel = quadrotor.getAngularVel();
        Vector4d quat = quadrotor.getQuat();
        Matrix3d R_body2world = quadrotor.getR();
        Vector3d angular_vel_world = R_body2world * angular_vel;
        
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
        odom_pub->publish(odom);

        RCLCPP_INFO(this->get_logger(), "Odom = %f,%f,%f, %f,%f,%f,%f", pos(0), pos(1), pos(2), quat(0), quat(1), quat(2), quat(3));

        // Publish IMU
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.frame_id = "/" + quad_name;
        imu_msg.header.stamp = now_time;
        imu_msg.orientation.w = quat(0);
        imu_msg.orientation.x = quat(1);
        imu_msg.orientation.y = quat(2);
        imu_msg.orientation.z = quat(3);
        imu_msg.angular_velocity.x = angular_vel(0);
        imu_msg.angular_velocity.y = angular_vel(1);
        imu_msg.angular_velocity.z = angular_vel(2);
        acc = R_body2world.inverse() * (acc + Eigen::Vector3d(0, 0, -9.8));
        imu_msg.linear_acceleration.x = acc(0);
        imu_msg.linear_acceleration.y = acc(1);
        imu_msg.linear_acceleration.z = acc(2);
        imu_pub->publish(imu_msg);
    }

    quadrotor_dynamics quadrotor;
    rclcpp::Time last_time;
    std::string quad_name;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QuadrotorDynamicsNode>();
    
    // Get parameters
    double init_x, init_y, init_z, mass;
    double simulation_rate;
    
    node->declare_parameter("mass", 0.9);
    node->declare_parameter("init_state_x", 0.0);
    node->declare_parameter("init_state_y", 0.0);
    node->declare_parameter("init_state_z", 1.0);
    node->declare_parameter("simulation_rate", 200.0);
    node->declare_parameter("quadrotor_name", std::string("quadrotor"));
    
    mass = node->get_parameter("mass").as_double();
    init_x = node->get_parameter("init_state_x").as_double();
    init_y = node->get_parameter("init_state_y").as_double();
    init_z = node->get_parameter("init_state_z").as_double();
    simulation_rate = node->get_parameter("simulation_rate").as_double();
    std::string quad_name = node->get_parameter("quadrotor_name").as_string();

    // Initialize quadrotor dynamics
    Matrix3d Internal_mat;
    Internal_mat << 2.64e-3, 0, 0,
                    0, 2.64e-3, 0,
                    0, 0, 4.96e-3;

    Vector3d init_pos;
    Vector4d init_q;
    init_pos << init_x, init_y, init_z;
    init_q << 1, 0, 0.0, 0.0;
    
    node->initialize_quadrotor(mass, Internal_mat, init_pos, init_q, quad_name);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}