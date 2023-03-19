#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <fstream>

// Add a global frame counter
size_t frame_counter = 0;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Convert the sensor_msgs::PointCloud2 message to a pcl::PointCloud<pcl::PointXYZ> object
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *cloud);
  
  // Open the CSV file in append mode, preserving previous data
  std::ofstream csv_file("/home/ruize/PC_dealing_results.csv", std::ios::app);

  if (!csv_file.is_open())
  {
    ROS_ERROR("Failed to open the CSV file for writing");
    return;
  }

  // Write the header row to the CSV file only for the first frame
  if (frame_counter == 0)
  {
    csv_file << "frame,index,r,theta,phi\n";
  }

  // Iterate through the points in the point cloud and convert them to polar coordinates
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    float x = cloud->points[i].x;
    float y = cloud->points[i].y;
    float z = cloud->points[i].z;
    
    float r = std::sqrt(x * x + y * y + z * z); // radius
    float theta = std::atan2(y, x); // azimuth angle in radians [-π, π]
    float phi = std::acos(z / r); // polar angle in radians [0, π]

    // Include the frame_counter in the CSV file
    csv_file << frame_counter << "," << i << "," << r << "," << theta << "," << phi << "\n";
  }
  csv_file.close();
  ROS_INFO("Polar coordinates of frame %zu saved to PC_dealing_results.csv", frame_counter);

  // Increment the frame counter
  frame_counter++;
}

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "point_cloud_listener");

  // Create a node handle
  ros::NodeHandle node;

  // Create a subscriber that listens to the "/map_generator/global_cloud" topic
  // and calls the pointCloudCallback function when a new message is received
  ros::Subscriber cloud_sub = node.subscribe<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 10, pointCloudCallback);

  // Enter the main event loop and process the incoming messages
  ros::spin();

  return 0;
}