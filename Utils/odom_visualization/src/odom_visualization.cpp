#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

class OdomVisualizationNode : public rclcpp::Node
{
public:
    OdomVisualizationNode() : Node("odom_visualization")
    {
        // Declare parameters
        this->declare_parameter("mesh_resource", "package://odom_visualization/meshes/yunque.dae");
        this->declare_parameter("color_r", 1.0);
        this->declare_parameter("color_g", 0.0);
        this->declare_parameter("color_b", 0.0);
        this->declare_parameter("color_a", 1.0);
        this->declare_parameter("robot_scale", 2.0);
        this->declare_parameter("frame_id", "world");
        this->declare_parameter("drone_id", 0);
        this->declare_parameter("quadrotor_name", "quadrotor");

        // Get parameters
        mesh_resource_ = this->get_parameter("mesh_resource").as_string();
        color_r_ = this->get_parameter("color_r").as_double();
        color_g_ = this->get_parameter("color_g").as_double();
        color_b_ = this->get_parameter("color_b").as_double();
        color_a_ = this->get_parameter("color_a").as_double();
        scale_ = this->get_parameter("robot_scale").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        drone_id_ = this->get_parameter("drone_id").as_int();
        quad_name_ = this->get_parameter("quadrotor_name").as_string();

        // Publishers
        mesh_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("robot", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
        traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("trajectory", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
        marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&OdomVisualizationNode::odom_callback, this, std::placeholders::_1));
        
        cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
            "cmd", 10, std::bind(&OdomVisualizationNode::cmd_callback, this, std::placeholders::_1));

        // TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Initialize path
        path_.header.frame_id = frame_id_;
        
        RCLCPP_INFO(this->get_logger(), "Odom visualization node started for drone %d", drone_id_);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Update path
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = msg->header;
        pose_stamped.pose = msg->pose.pose;
        path_.poses.push_back(pose_stamped);
        
        // Limit path length
        if (path_.poses.size() > 1000) {
            path_.poses.erase(path_.poses.begin());
        }
        
        path_.header.stamp = msg->header.stamp;
        path_pub_->publish(path_);

        // Publish pose
        pose_pub_->publish(pose_stamped);

        // Publish 3D mesh model
        publish_mesh(msg);

        // Publish trajectory line
        publish_trajectory(msg);

        // Publish TF
        publish_tf(msg);
    }

    void cmd_callback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
    {
        // Store command for visualization
        last_cmd_ = *msg;
        has_cmd_ = true;
    }

    void publish_mesh(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        visualization_msgs::msg::Marker mesh;
        mesh.header.frame_id = frame_id_;
        mesh.header.stamp = msg->header.stamp;
        mesh.ns = "mesh";
        mesh.id = drone_id_;
        mesh.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        mesh.action = visualization_msgs::msg::Marker::ADD;
        
        mesh.pose = msg->pose.pose;
        
        mesh.scale.x = scale_;
        mesh.scale.y = scale_;
        mesh.scale.z = scale_;
        
        mesh.color.r = color_r_;
        mesh.color.g = color_g_;
        mesh.color.b = color_b_;
        mesh.color.a = color_a_;
        
        mesh.mesh_resource = mesh_resource_;
        
        mesh_pub_->publish(mesh);
    }

    void publish_trajectory(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        static geometry_msgs::msg::Point last_point;
        static bool first_point = true;
        
        if (first_point) {
            last_point.x = msg->pose.pose.position.x;
            last_point.y = msg->pose.pose.position.y;
            last_point.z = msg->pose.pose.position.z;
            first_point = false;
            return;
        }

        visualization_msgs::msg::Marker traj;
        traj.header.frame_id = frame_id_;
        traj.header.stamp = msg->header.stamp;
        traj.ns = "trajectory";
        traj.id = drone_id_;
        traj.type = visualization_msgs::msg::Marker::LINE_LIST;
        traj.action = visualization_msgs::msg::Marker::ADD;
        
        traj.pose.orientation.w = 1.0;
        traj.scale.x = 0.05;  // Line width
        
        traj.color.r = 0.0;
        traj.color.g = 1.0;
        traj.color.b = 0.0;
        traj.color.a = 0.8;
        
        // Add line from last point to current point
        traj.points.push_back(last_point);
        
        geometry_msgs::msg::Point current_point;
        current_point.x = msg->pose.pose.position.x;
        current_point.y = msg->pose.pose.position.y;
        current_point.z = msg->pose.pose.position.z;
        traj.points.push_back(current_point);
        
        // Add colors for each point
        std_msgs::msg::ColorRGBA color;
        color.r = 0.0; color.g = 1.0; color.b = 0.0; color.a = 0.8;
        traj.colors.push_back(color);
        traj.colors.push_back(color);
        
        traj_pub_->publish(traj);
        
        last_point = current_point;
    }

    void publish_tf(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = msg->header.stamp;
        transform.header.frame_id = frame_id_;
        transform.child_frame_id = quad_name_ + "/base_link";
        
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;
        
        transform.transform.rotation = msg->pose.pose.orientation;
        
        tf_broadcaster_->sendTransform(transform);
    }

    // Member variables
    std::string mesh_resource_;
    double color_r_, color_g_, color_b_, color_a_;
    double scale_;
    std::string frame_id_;
    int drone_id_;
    std::string quad_name_;
    
    nav_msgs::msg::Path path_;
    quadrotor_msgs::msg::PositionCommand last_cmd_;
    bool has_cmd_ = false;

    // ROS 2 objects
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomVisualizationNode>());
    rclcpp::shutdown();
    return 0;
}
