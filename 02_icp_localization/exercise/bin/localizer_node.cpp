#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "constants.h"
#include "icp/eigen_kdtree.h"
#include "localizer2d.h"
#include "map.h"
#include "ros_bridge.h"
bool function_called = false;
// Map callback definition
void callback_map(const nav_msgs::OccupancyGridConstPtr&);
// Initial pose callback definition
void callback_initialpose( const geometry_msgs::PoseWithCovarianceStampedConstPtr&);
// Scan callback definition
void callback_scan(const sensor_msgs::LaserScanConstPtr&);
void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    ROS_INFO("Received message from /initialpose: Subscribed!");
}

std::shared_ptr<Map> map_ptr = nullptr;
ros::Publisher pub_scan, pub_odom;

Localizer2D localizer;

int main(int argc, char** argv) {
  // Initialize ROS system
  // TODO
  ros::init(argc, argv, "localizer_node");
  // Create a NodeHandle to manage the node.
  // The namespace of the node is set to global
  ros::NodeHandle nh("/");
  // Create a Map object and assign it to map_ptr
map_ptr = std::make_shared<Map>();


  // Create shared pointer for the Map object
  // TODO
  ros::Subscriber sub_map = nh.subscribe("/map", 1, callback_map);
  ros::Subscriber sub_initialpose = nh.subscribe("/initialpose", 1, callback_initialpose);
  ros::Subscriber sub_scan = nh.subscribe("/base_scan", 1, callback_scan);

  //
  /**
   * Subscribe to the topics:
   * /map [nav_msgs::OccupancyGrid]
   * /initialpose [geometry_msgs::PoseWithCovarianceStamped]
   * /base_scan [sensor_msgs::LaserScan]
   * and assign the correct callbacks
   
   * Advertise the following topic:
   * /odom_out [nav_msgs::Odometry]
   */
  pub_odom = nh.advertise<nav_msgs::Odometry>("/odom_out", 10);
  // TODO

  // Scan advertiser for visualization purposes
  pub_scan = nh.advertise<sensor_msgs::LaserScan>("/scan_out", 10);

  ROS_INFO("Node started. Waiting for input data");

  // Spin the node
  ros::spin();

  return 0;
}

void callback_map(const nav_msgs::OccupancyGridConstPtr& msg_) {
  // If the internal map is not initialized, load the incoming occupancyGrid and
  // set the localizer map accordingly
  // Remember to load the map only once during the execution of the map.
  ROS_INFO("map initialization");

  if (!function_called) {
    ROS_INFO("Map initialization");


    // Load the map from the incoming message
    map_ptr->loadOccupancyGrid(msg_);

    // Set the map for the localizer
    localizer.setMap(map_ptr);
  } else {
    ROS_INFO("Map already initialized, skipping.");
  }



}


void callback_initialpose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg_) {
  /**
   * Convert the PoseWithCovarianceStamped message to an Eigen Isometry and
   * inform the localizer.
   * You can check ros_bridge.h for helps :)
   */
      ROS_INFO("Received initial pose with timestamp: %f", msg_->header.stamp.toSec());

    // Add your custom logic here to handle the initial pose data
    // ...

    // Remember to check for null messages before accessing fields if necessary
    if (msg_ == nullptr) {
        ROS_ERROR("Received a null message on /initialpose topic.");
        return;
    }
 ROS_INFO("Received an initial pose message.");
  // Extract the pose message from PoseWithCovarianceStamped
  const geometry_msgs::Pose& pose_msg = msg_->pose.pose;

  // Convert the Pose message to an Eigen::Isometry2f
  Eigen::Isometry2f isometry_initial_pose;
  pose2isometry(pose_msg, isometry_initial_pose);

  // Now, 'isometry' contains the pose in Eigen::Isometry2f format.
  // Calculate the rotation angle using atan2
  float rotation_angle = atan2(isometry_initial_pose.linear()(1, 0), isometry_initial_pose.linear()(0, 0));

 
  ROS_INFO("Transformed Isometry: (%f, %f, %f)", isometry_initial_pose.translation().x(), isometry_initial_pose.translation().y(), rotation_angle);
  localizer.setInitialPose(isometry_initial_pose);
}
 

void callback_scan(const sensor_msgs::LaserScanConstPtr& msg_) {
  ROS_INFO("Received a scan message.");
  /**
   * Convert the LaserScan message into a Localizer2D::ContainerType
   * [std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>]
   */
  // TODO
  std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> scan_data;

  // Use the scan2eigen function to convert the LaserScan message
  scan2eigen(msg_, scan_data);
  /**
   * Set the laser parameters and process the incoming scan through the
   * localizer
   */
  // TODO
    // Set the laser parameters based on the LaserScan message
  localizer.setLaserParams(msg_->range_min, msg_->range_max, msg_->angle_min, msg_->angle_max, msg_->angle_increment);

  /**
   * Send a transform message between FRAME_WORLD and FRAME_LASER.
   * The transform should contain the pose of the laser with respect to the map
   * frame.
   * You can use the isometry2transformStamped function to convert the isometry
   * to the right message type.
   * Look at include/constants.h for frame names
   *
   * The timestamp of the message should be equal to the timestamp of the
   * received message (msg_->header.stamp)
   */

    geometry_msgs::TransformStamped transform_msg;

  transform_msg.header.frame_id = FRAME_WORLD;
  transform_msg.child_frame_id = FRAME_LASER;
  transform_msg.header.stamp = msg_->header.stamp;
  isometry2transformStamped(localizer.X(), transform_msg, transform_msg.header.frame_id,transform_msg.child_frame_id,transform_msg.header.stamp); // Use the current laser pose
  static tf2_ros::TransformBroadcaster br;
  br.sendTransform(transform_msg);
  // TODO

  /**
   * Send a nav_msgs::Odometry message containing the current laser_in_world
   * transform.
   * You can use transformStamped2odometry to convert the previously computed
   * TransformStamped message to a nav_msgs::Odometry message.
   */
  // TODO
  nav_msgs::Odometry odom_msg;
  transformStamped2odometry(transform_msg,odom_msg);
  pub_odom.publish(odom_msg);
  // Sends a copy of msg_ with FRAME_LASER set as frame_id
  // Used to visualize the scan attached to the current laser estimate.
  sensor_msgs::LaserScan out_scan = *msg_;
  out_scan.header.frame_id = FRAME_LASER;
  pub_scan.publish(out_scan);
}
