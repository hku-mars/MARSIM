
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cascadePID.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

using namespace std;
using namespace Eigen;

class CascadePIDNode : public rclcpp::Node
{
public:
    CascadePIDNode() : Node("cascade_pid_node"), quad_PID(20)
    {
        control_RPM_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("cmd_RPM", 10);
        
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&CascadePIDNode::odom_callback, this, std::placeholders::_1));
        
        cmd_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "cmd", 10, std::bind(&CascadePIDNode::cmd_callback, this, std::placeholders::_1));
        
        fuel_cmd_sub = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
            "/planning/pos_cmd_1", 10, std::bind(&CascadePIDNode::fuel_position_cmd_callback, this, std::placeholders::_1));
        
        control_timer = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&CascadePIDNode::run_control, this));
        
        t_init = this->now();
        yaw_des = 0.0;
        first_odom_flag = 0;
        control_flag = 0;
        
        // Set default hover position to prevent immediate falling
        pos_des << 0.0, 0.0, 1.0;  // Hover at 1 meter height
        vel_des << 0.0, 0.0, 0.0;
        acc_des << 0.0, 0.0, 0.0;
        control_flag = 1;  // Enable control by default to maintain hover
    }

    void configure_controller(int drone_id, double controller_rate, double mass, 
                            const Matrix3d& Internal_mat, double arm_length, double k_F,
                            double angle_stable_time, double damping_ratio,
                            double init_x, double init_y, double init_z, double init_yaw)
    {
        quad_PID.setdroneid(drone_id);
        quad_PID.setrate(controller_rate);
        quad_PID.setInternal(mass, Internal_mat, arm_length, k_F);
        quad_PID.setParam(angle_stable_time, damping_ratio);
        
        pos_des << init_x, init_y, init_z;
        vel_des << 0, 0, 0;
        acc_des << 0, 0, 0;
        yaw_des = init_yaw;
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr control_RPM_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cmd_sub;
    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr fuel_cmd_sub;
    rclcpp::TimerBase::SharedPtr control_timer;
    
    Vector3d pos_des, vel_des, acc_des;
    double yaw_des;
    cascadePID quad_PID;
    rclcpp::Time t_init, t2;

    nav_msgs::msg::Odometry current_odom, last_odom;
    int first_odom_flag, control_flag;
    geometry_msgs::msg::PoseStamped pose_cmd;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        if(first_odom_flag == 1)
        {
            last_odom = current_odom;
        }else{
            first_odom_flag = 1;
        }
        current_odom = *odom;
    }

    void cmd_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        pose_cmd = *msg;
        control_flag = 1;
    }

    void fuel_position_cmd_callback(const quadrotor_msgs::msg::PositionCommand::SharedPtr cmd) {
        pos_des = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
        vel_des = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
        acc_des = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);

        yaw_des = cmd->yaw;
        control_flag = 1;  // Enable control when receiving position commands
    }

    void run_control()
    {
        if(first_odom_flag == 1) 
        {

        //run controller
        Vector3d current_euler;
        Eigen::Quaterniond quaternion_temp(current_odom.pose.pose.orientation.w, current_odom.pose.pose.orientation.x\
                                        , current_odom.pose.pose.orientation.y, current_odom.pose.pose.orientation.z);
        Vector3d current_pos, current_vel;
        current_pos << current_odom.pose.pose.position.x, current_odom.pose.pose.position.y, current_odom.pose.pose.position.z;
        current_vel << current_odom.twist.twist.linear.x, current_odom.twist.twist.linear.y, current_odom.twist.twist.linear.z; 
        quad_PID.FeedbackInput(current_pos, current_vel, quaternion_temp);

        if(control_flag == 1)
        {
            // Only use pose_cmd if it was set via cmd_callback (not fuel_position_cmd_callback)
            // The fuel_position_cmd_callback already sets pos_des, vel_des, acc_des, yaw_des
            if(pose_cmd.header.stamp.sec != 0 || pose_cmd.header.stamp.nanosec != 0)
            {
                pos_des << pose_cmd.pose.position.x, pose_cmd.pose.position.y, pose_cmd.pose.position.z;
                
                Eigen::Quaterniond q_des(pose_cmd.pose.orientation.w, pose_cmd.pose.orientation.x\
                                            , pose_cmd.pose.orientation.y, pose_cmd.pose.orientation.z);
                Eigen::Matrix3d R_des;
                R_des = q_des.matrix();
                Eigen::Vector3d x_body;
                x_body << 1,0,0;
                x_body = R_des * x_body;

                yaw_des = atan2(x_body(1),x_body(0));//pose_cmd.pose.orientation.z
            }
            t2 = this->now();
        }
        
        quad_PID.setOdomdes(pos_des,vel_des,acc_des, yaw_des);

        quad_PID.RunController();
        Eigen::Vector3d Torque_des;
        Torque_des = quad_PID.getTorquedes();
        Vector4d RPM_output = quad_PID.getRPMoutputs();

        std_msgs::msg::Float32MultiArray rpm_array;
        rpm_array.data.clear();
        rpm_array.data.push_back(RPM_output(0));
        rpm_array.data.push_back(RPM_output(1));
        rpm_array.data.push_back(RPM_output(2));
        rpm_array.data.push_back(RPM_output(3));
        control_RPM_pub->publish(rpm_array);

        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CascadePIDNode>();

    // Get parameters
    double init_x, init_y, init_z;
    double init_yaw;
    double controller_rate, angle_stable_time, damping_ratio;
    int drone_id;
    std::string quad_name;
    
    node->declare_parameter("init_state_x", 0.0);
    node->declare_parameter("init_state_y", 0.0);
    node->declare_parameter("init_state_z", 1.0);
    node->declare_parameter("init_state_yaw", 1.0);
    node->declare_parameter("controller_rate", 200.0);
    node->declare_parameter("quadrotor_name", std::string("quadrotor"));
    node->declare_parameter("angle_stable_time", 0.1);
    node->declare_parameter("damping_ratio", 0.7);
    node->declare_parameter("drone_id", 0);
    
    init_x = node->get_parameter("init_state_x").as_double();
    init_y = node->get_parameter("init_state_y").as_double();
    init_z = node->get_parameter("init_state_z").as_double();
    init_yaw = node->get_parameter("init_state_yaw").as_double();
    controller_rate = node->get_parameter("controller_rate").as_double();
    quad_name = node->get_parameter("quadrotor_name").as_string();
    angle_stable_time = node->get_parameter("angle_stable_time").as_double();
    damping_ratio = node->get_parameter("damping_ratio").as_double();
    drone_id = node->get_parameter("drone_id").as_int();
    
    init_yaw = init_yaw / 180.0 * M_PI;

    Matrix3d Internal_mat;
    Internal_mat << 2.64e-3,0,0,
                    0,2.64e-3,0,
                    0,0,4.96e-3;
    double arm_length = 0.22;
    double k_F = 8.98132e-9;
    k_F = 3.0*k_F;
    double mass = 1.9;

    // Configure the PID controller
    node->configure_controller(drone_id, controller_rate, mass, Internal_mat, 
                              arm_length, k_F, angle_stable_time, damping_ratio,
                              init_x, init_y, init_z, init_yaw);
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}