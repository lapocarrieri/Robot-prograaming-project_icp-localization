#pragma once

#include <nav_msgs/OccupancyGrid.h>

#include <Eigen/Core>
#include <opencv2/core.hpp>

enum CellType : int8_t { Unknown = -1, Free = 0, Occupied = 100 };

class Map {
 public:
  /**
   * @brief Construct a new Map object
   *
   */
  Map();
  /**
   * @brief Extracts the map data and metadata from a  nav_msgs::OccupancyGrid
   * message
   *
   * @param message
   */
  void loadOccupancyGrid(const nav_msgs::OccupancyGridConstPtr& message);
  /**
   * @brief Returns the map data in form of std::vector<int8_t>
   * Every element of the vector is:
   * CellType::Unknown : -1
   * CellType::Free : 0
   * CellType::Occupied: 100   *
   * @return const std::vector<int8_t>&
   */
  inline const std::vector<int8_t>& grid() const { return _grid; }
  /**
   * @brief Returns the map in OpenCV format
   *
   * @return const cv::Mat&
   */
  inline const cv::Mat& map() const { return _frame_grid; }
  /**
   * @brief Return the number of rows of the map
   *
   * @return size_t
   */
  inline size_t rows() const { return _rows; }
  /**
   * @brief Return the number of columns of the map
   *
   * @return size_t
   */
  inline size_t cols() const { return _cols; }
  /**
   * @brief Return the resolution of the map [meters per pixel]
   *
   * @return float
   */
  inline float resolution() const { return _resolution; }
  /**
   * @brief Returns the inverse resolution of hte map [pixels per meters]
   *
   * @return float
   */
  inline float inverse_resolution() const { return _inverse_resolution; }
  /**
   * @brief Returns the size of the map in OpenCV format
   *
   * @return cv::Size2i
   */
  inline cv::Size2i size() const { return cv::Size2i(_cols, _rows); }
  /**
   * @brief Returns the origin of the map
   *
   * @return Eigen::Vector3f
   */
  inline Eigen::Vector3f origin() const { return _origin; }
  /**
   * @brief Returns the state of the map
   *
   * @return true if a valid map has been received
   * @return false otherwise
   */
  inline bool initialized() const { return _rows != 0 || _cols != 0; }

  // Accessors on map
  inline int8_t operator()(size_t r, size_t c) const {
    return _grid[r * _cols + c];
  }
  inline int8_t operator()(Eigen::Vector2i p) const {
    return _grid[p.x() * _cols + p.y()];
  }
  inline int8_t operator()(cv::Point2i p) const {
    return _grid[p.x * _cols + p.y];
  }

  /**
   * @brief Converts a point in the grid map into world coordinates
   *
   * @param p point in gridmap
   * @return Eigen::Vector2f point in world
   */
  inline Eigen::Vector2f grid2world(cv::Point2i p) const {
    return Eigen::Vector2f(p.y * _resolution + _origin.y(),
                           p.x * _resolution + _origin.x());
  }

  /**
   * @brief Converts a point in world into grid coordinates
   *
   * @param p point in world
   * @return cv::Point2i point in gridmap
   */
  inline cv::Point2i world2grid(Eigen::Vector2f p) {
    return cv::Point2i(p.x(), p.y()) * _inverse_resolution;
  }

 protected:
  cv::Mat _frame_grid;
  std::vector<int8_t> _grid;
  size_t _rows, _cols;
  float _resolution, _inverse_resolution;
  Eigen::Vector3f _origin;
};