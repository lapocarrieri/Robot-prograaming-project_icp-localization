#include "localizer2d.h"
#include "eigen_kdtree.h"
#include "icp/eigen_icp_2d.h"
#include "eigen_kdtree.h"
#include <iostream>
#include <fstream>
Localizer2D::Localizer2D()
    : _map(nullptr),
      _laser_in_world(Eigen::Isometry2f::Identity()),
      _obst_tree_ptr(nullptr) {}

/**
 * @brief Set the internal map reference and constructs the KD-Tree containing
 * obstacles coordinates for fast access.
 *
 * @param map_
 */
#include <Eigen/Core> // Include the Eigen library

// Define the point type you want to use (Eigen::Vector3f)
using PointType = Eigen::Vector3f;

// Define the container type using the point type
using ContainerType = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

// Define the tree node type using the container's iterator
using TreeNodeType = TreeNode_<ContainerType::iterator>;


void Localizer2D::setMap(std::shared_ptr<Map> map_) {
  // Set the internal map pointer
  _map = map_;
   // Check if the map is initialized
    if (_map) {
        // Get map size (rows and columns)
        size_t mapRows = _map->rows();
        size_t mapCols = _map->cols();

        // Clear any existing obstacle data in _obst_vect
        
	ContainerType kd_points;

        // Get the map grid data
        const std::vector<int8_t>& mapGrid = _map->grid();

        // Iterate through the occupancy grid to find obstacles
        for (size_t row = 0; row < mapRows; ++row) {
            for (size_t col = 0; col < mapCols; ++col) {
                // Get the cell type from the grid data
                int8_t cellType = mapGrid[row * mapCols + col];

                // Check if the cell represents an obstacle
                if (cellType == static_cast<int8_t>(CellType::Occupied)) {
                    // Calculate the world coordinates of the obstacle cell
                    float worldX = col * _map->resolution(); // Convert map column to world X
                    float worldY = row * _map->resolution(); // Convert map row to world Y

                    // Create a PointType (Eigen::Vector2f) for the obstacle and add it to _obst_vect
                    PointType obstaclePoint(worldX, worldY);
                   kd_points.push_back(obstaclePoint);
                
		    //std::cout << "Added obstacle at X: " << worldX << ", Y: " << worldY << std::endl;
                }
	    }
	}
	    // If the map is initialized, create KD-Tree using TreeNodeType with kd_points
        TreeNode_ kd_tree(kd_points.begin(), kd_points.end(), 10); // Replace 10 with your desired maximum points in leaf
 // Store the KD-Tree in the obst_tree_ptr member variable or as needed within your class.
       // Adjust this as needed based on how you manage your KD-Tree instance.
     
	   
         

    }
}

/**
 * @brief Set the current estimate for laser_in_world
 *
 * @param initial_pose_
 */
void Localizer2D::setInitialPose(const Eigen::Isometry2f& initial_pose_) {
  // TODO
}

/**
 * @brief Process the input scan.
 * First creates a prediction using the current laser_in_world estimate
 *
 * @param scan_
 */
void Localizer2D::process(const ContainerType& scan_) {
  // Use initial pose to get a synthetic scan to compare with scan_
  // TODO

  /**
   * Align prediction and scan_ using ICP.
   * Set the current estimate of laser in world as initial guess (replace the
   * solver X before running ICP)
   */
  // TODO

  /**
   * Store the solver result (X) as the new laser_in_world estimate
   *
   */
  // TODO
}

/**
 * @brief Set the parameters of the laser scanner. Used to predict
 * measurements.
 * These parameters should be taken from the incoming sensor_msgs::LaserScan
 * message
 *
 * For further documentation, refer to:
 * http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html
 *
 *
 * @param range_min_
 * @param range_max_
 * @param angle_min_
 * @param angle_max_
 * @param angle_increment_
 */
void Localizer2D::setLaserParams(float range_min_, float range_max_,
                                 float angle_min_, float angle_max_,
                                 float angle_increment_) {
  _range_min = range_min_;
  _range_max = range_max_;
  _angle_min = angle_min_;
  _angle_max = angle_max_;
  _angle_increment = angle_increment_;
}

/**
 * @brief Computes the predicted scan at the current laser_in_world pose
 * estimate.
 *
 * @param dest_ Output predicted scan
 */
void Localizer2D::getPrediction(ContainerType& prediction_) {
  prediction_.clear();
  /**
   * To compute the prediction, query the KD-Tree and search for all points
   * around the current laser_in_world estimate.
   * You may use additional sensor's informations to refine the prediction.
   */
  // TODO
}
