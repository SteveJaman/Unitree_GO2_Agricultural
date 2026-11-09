#ifndef SEMANTIC_TRAVERSABILITY_LAYER__HEIGHT_BAND_LAYER_HPP_
#define SEMANTIC_TRAVERSABILITY_LAYER__HEIGHT_BAND_LAYER_HPP_

#include <deque>
#include <mutex>
#include <string>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace semantic_traversability_layer
{

class HeightBandLayer : public nav2_costmap_2d::Layer
{
public:
  HeightBandLayer();
  ~HeightBandLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override { return false; }

private:
  void pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  // Floor height for this frame from a z histogram near ground_z (floor_estimation: histogram).
  double estimateFloor(const sensor_msgs::msg::PointCloud2 & cloud);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  struct ClassifiedPoint {
    double wx, wy;
    unsigned char cost;
  };
  std::vector<ClassifiedPoint> classified_points_;
  std::mutex points_mutex_;

  // Parameters
  bool enabled_;
  std::string topic_;
  std::string global_frame_;
  double ground_z_;
  std::string floor_estimation_;   // "fixed" (ground_z) | "histogram"
  double floor_window_, floor_bin_, floor_search_radius_;
  int floor_smoothing_frames_, floor_min_points_;
  std::deque<double> floor_history_;
  double ground_tolerance_;
  double step_over_height_max_;
  double lethal_height_min_;
  double min_range_;
  double max_range_;
  unsigned char step_over_cost_;
  unsigned char caution_cost_;

  double robot_x_, robot_y_;
  bool has_robot_pose_;
};

}  // namespace semantic_traversability_layer

#endif  // SEMANTIC_TRAVERSABILITY_LAYER__HEIGHT_BAND_LAYER_HPP_
