#include "ros_bridge.h"

void scan2eigen(const sensor_msgs::LaserScan::ConstPtr& msg_,
                std::vector<Eigen::Vector2f,
                            Eigen::aligned_allocator<Eigen::Vector2f>>& dest_) {
  /**
   * @brief Unproject points stored as ranges in a sensor_msgs::LaserScan
   * message into a more suitable std::vector<Eigen::Vector2f> format.
   *
   * a LaserScan message contains many fields.
   * Right now we require:
   *
   * - angle_min : start angle of the scan [rad]
   * - angle_increment : angular distance between two measurements [rad]
   * - range_min : minimum range value [m]
   * - range_max : maximum range value [m]
   * - ranges : range data [m] <- this is a std::vector<float>
   *
   * For additional informations on the LaserScan message, refer to:
   * http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
   *
   * The unprojection transforms the polar representation of a point p_i
   * (range_i, angle_i) into cartesian representation (x_i, y_i)
   *
   * Points outside the range (range_min, range_max) should be discarded.
   */
  dest_.clear();
  float a = msg_->angle_min;
  float ainc = msg_->angle_increment;
  float rmin = msg_->range_min;
  float rmax = msg_->range_max;
  const auto ranges = msg_->ranges;
  // TODO: Iterate over the available ranges and fill dest_ accordingly.
  for (const auto r : ranges) {
    if (r >= rmin && r <= rmax) {
      dest_.push_back(Eigen::Vector2f(r * cos(a), r * sin(a)));
    }
    a += ainc;
  }
}

void eigen2scan(
    const std::vector<Eigen::Vector2f,
                      Eigen::aligned_allocator<Eigen::Vector2f>>& src_,
    sensor_msgs::LaserScan& dest_, float range_min_, float range_max_,
    float angle_min_, float angle_max_, float angle_increment_) {
  size_t num_points = (angle_max_ - angle_min_) / angle_increment_;
  dest_.ranges.resize(num_points, range_max_);

  for (const auto& p : src_) {
    long int p_idx =
        floor((atan2(p.y(), p.x()) - angle_min_) / angle_increment_);
    if (p_idx < 0 || p_idx >= num_points) continue;
    dest_.ranges[p_idx] = p.norm();
  }

  dest_.range_min = range_min_;
  dest_.range_max = range_max_;
  dest_.angle_min = angle_min_;
  dest_.angle_max = angle_max_;
  dest_.angle_increment = angle_increment_;
}

void isometry2transformStamped(const Eigen::Isometry2f& pose_,
                               geometry_msgs::TransformStamped& msg_,
                               const std::string& frame_id_,
                               const std::string& child_frame_id_,
                               const ros::Time& stamp_) {
  /**
   * @brief To return results into the ROS environment, we need to represent our
   * sensor's pose into a suitable format (geometry_msgs::TransformStamped).
   *
   * The header of the message contains two important parameters we need to
   setup:
   *
   * - stamp : timestamp of the message (should be synchronized with the
   received scan)
   * - frame_id : Frame this data is associated with (in our case, the global
   frame id)
   *
   * Furthermore, the payload of the message contains a 3D transform
   representation expressed as
   *
   * - translation : [x, y, z]
   * - rotation : [qx, qy, qz, qw]
   *
   * Both of these components are accessible from:
   *
   *  msg_.transform.<translation|rotation>
   *
   * To see the TransformStamped message format, refer to:
   * http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TransformStamped.html
   *
   * First, setup the header of the message by copying the frame_id_ and stamp_
   * parameters in the right place ( hint msg_.header.<place> )
   *
   * Second, setup the child_frame_id component
   *
   * Secondly, load the translation and rotation expressed by a quaternion we
   provide.
   */

  // TODO Setup the message header
  msg_.header.frame_id = frame_id_;
  msg_.header.stamp = stamp_;

  // TODO Setup the child_frame_id
  msg_.child_frame_id = child_frame_id_;

  // TODO Load the translation values in msg_.transform.translation
  msg_.transform.translation.x = pose_.translation().x();
  msg_.transform.translation.y = pose_.translation().y();
  msg_.transform.translation.z = 0.0;

  Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
  R.block<2, 2>(0, 0) = pose_.linear();
  Eigen::Quaternionf q(R);
  q.normalize();

  // TODO Load the rotation values in msg_.transform.rotation
  msg_.transform.rotation.w = q.w();
  msg_.transform.rotation.x = q.x();
  msg_.transform.rotation.y = q.y();
  msg_.transform.rotation.z = q.z();
}

void transformStamped2odometry(const geometry_msgs::TransformStamped& msg_,
                               nav_msgs::Odometry& odom_) {
  /**
   * @brief Given the similarity between geometry_msgs::TransformStamped and
   * nav_msgs::Odometry messages, we can copy the data from one to the other.
   *
   * Bear in mind that ROS format uses different representations in these
   * messages (even though the data is exactly the same).
   *
   * Header-wise, you can copy msg_ header into odom_
   *
   * Pose-wise, the Odometry message contains:
   *
   * - position : [x, y, z]
   * - orientation : [qx, qy, qz, qw]
   *
   * Both these components are accessible from:
   *
   *  odom_.pose.pose.<position|orientation>
   *
   * To see the TransformStamped message format, refer to:
   * http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
   *
   */

  // TODO Setup the message header
  odom_.header.frame_id = msg_.header.frame_id;
  odom_.header.stamp = msg_.header.stamp;
  // TODO Setup the child_frame_id
  odom_.child_frame_id = msg_.child_frame_id;
  // TODO Load the translation values in odom_.pose.pose.position
  odom_.pose.pose.position.x = msg_.transform.translation.x;
  odom_.pose.pose.position.y = msg_.transform.translation.y;
  odom_.pose.pose.position.z = msg_.transform.translation.z;
  // TODO Load the orientation values in odom_.pose.pose.orientation
  odom_.pose.pose.orientation.x = msg_.transform.rotation.x;
  odom_.pose.pose.orientation.y = msg_.transform.rotation.y;
  odom_.pose.pose.orientation.z = msg_.transform.rotation.z;
  odom_.pose.pose.orientation.w = msg_.transform.rotation.w;
}

void pose2isometry(const geometry_msgs::Pose& pose_, Eigen::Isometry2f& iso_) {
  /**
   * @brief Given a geometry_msgs::Pose object, generate the corresponding
   * Isometry in 2D
   *
   */

  Eigen::Quaternionf q;
  q.x() = pose_.orientation.x;
  q.y() = pose_.orientation.y;
  q.z() = pose_.orientation.z;
  q.w() = pose_.orientation.w;
  Eigen::Matrix3f R = q.toRotationMatrix();

  iso_.linear() = R.topLeftCorner<2, 2>();
  iso_.translation() << pose_.position.x, pose_.position.y;
}
