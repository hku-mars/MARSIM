#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class Interface : public rclcpp::Node
{
public:
    Interface();

private:
    rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pub, pub2;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub, sub2;

    quadrotor_msgs::msg::PositionCommand cmd, cmd2;
    int _n_seq;

    void messageCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void messageCallback2(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

Interface::Interface() : Node("test_interface_node") {
    pub = this->create_publisher<quadrotor_msgs::msg::PositionCommand>
            ("/planning/pos_cmd_1", 10);
    pub2 = this->create_publisher<quadrotor_msgs::msg::PositionCommand>
            ("/planning/pos_cmd_2", 10);
    sub = this->create_subscription<geometry_msgs::msg::PoseStamped>
            ("/quadrotor_1_pos_cmd", 10, std::bind(&Interface::messageCallback, this, std::placeholders::_1));
    sub2 = this->create_subscription<geometry_msgs::msg::PoseStamped>
            ("/quadrotor_2_pos_cmd", 10, std::bind(&Interface::messageCallback2, this, std::placeholders::_1));
    
    // Subscribe to RViz goal pose
    auto goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>
            ("/goal_pose", 10, std::bind(&Interface::messageCallback, this, std::placeholders::_1));

    _n_seq = 0;

    /* kP */
    // double pos_gain[3] = { 5.7, 5.7, 6.2 };
    // double vel_gain[3] = { 3.4, 3.4, 4.0 };

    double pos_gain[3] = { 7, 7, 6.2 };
    double vel_gain[3] = { 4, 4, 4.0 };

    /* control parameter */
    cmd.kx[0] = pos_gain[0];
    cmd.kx[1] = pos_gain[1];
    cmd.kx[2] = pos_gain[2];

    cmd.kv[0] = vel_gain[0];
    cmd.kv[1] = vel_gain[1];
    cmd.kv[2] = vel_gain[2];

        /* control parameter */
    cmd2.kx[0] = pos_gain[0];
    cmd2.kx[1] = pos_gain[1];
    cmd2.kx[2] = pos_gain[2];

    cmd2.kv[0] = vel_gain[0];
    cmd2.kv[1] = vel_gain[1];
    cmd2.kv[2] = vel_gain[2];

}

void Interface::messageCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // header
    cmd.header.stamp = msg->header.stamp;
    cmd.header.frame_id = "world";

    cmd.trajectory_id = 0;
    cmd.trajectory_flag = 1;
    cmd.position.x = msg->pose.position.x;
    cmd.position.y = msg->pose.position.y;
    cmd.position.z = msg->pose.position.z;
    cmd.velocity.x = 0;
    cmd.velocity.y = 0;
    cmd.velocity.z = 0;
    cmd.acceleration.x = 0;
    cmd.acceleration.y = 0;
    cmd.acceleration.z = 0;

    tf2::Quaternion quat;
    double roll, pitch, yaw;
    tf2::fromMsg(msg->pose.orientation, quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);   

    cmd.yaw = yaw;

    pub->publish(cmd);

    // cmd.position.y = msg->pose.position.y + 2;

    // cmd.yaw = -1.7;

    // pub2.publish(cmd);
}

void Interface::messageCallback2(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // header
    cmd2.header.stamp = msg->header.stamp;
    cmd2.header.frame_id = "world";

    cmd2.trajectory_id = 0;
    cmd2.trajectory_flag = 1;
    cmd2.position.x = msg->pose.position.x;
    cmd2.position.y = msg->pose.position.y;
    cmd2.position.z = msg->pose.position.z;
    cmd2.velocity.x = 0;
    cmd2.velocity.y = 0;
    cmd2.velocity.z = 0;
    cmd2.acceleration.x = 0;
    cmd2.acceleration.y = 0;
    cmd2.acceleration.z = 0;

    tf2::Quaternion quat;
    double roll, pitch, yaw;
    tf2::fromMsg(msg->pose.orientation, quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);   

    cmd2.yaw = yaw;

    pub2->publish(cmd2);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_WARN(rclcpp::get_logger("test_interface"), "*****START*****");
    
    auto interface_node = std::make_shared<Interface>();
    rclcpp::spin(interface_node);
    rclcpp::shutdown();

    return 0;
}