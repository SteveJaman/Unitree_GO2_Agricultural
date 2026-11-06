#ifndef SEMANTIC_TRAVERSABILITY_LAYER__SEMANTIC_COSTMAP_LAYER_HPP_
#define SEMANTIC_TRAVERSABILITY_LAYER__SEMANTIC_COSTMAP_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "go2_semantic_msgs/msg/semantic_obstacle_array.hpp"

namespace semantic_traversability_layer
{

class SemanticCostmapLayer : public nav2_costmap_2d::Layer
{
public:
  SemanticCostmapLayer();
  ~SemanticCostmapLayer() override = default;

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
  void obstacleCallback(
    go2_semantic_msgs::msg::SemanticObstacleArray::ConstSharedPtr msg);

  rclcpp::Subscription<go2_semantic_msgs::msg::SemanticObstacleArray>::SharedPtr sub_;

  struct CostMark {
    double wx, wy;
    double radius;
    unsigned char cost;
  };
  std::vector<CostMark> marks_;
  std::mutex marks_mutex_;

  bool enabled_;
  std::string topic_;
  double max_range_;
  double robot_x_, robot_y_;
  bool has_robot_pose_;
};

}  // namespace semantic_traversability_layer

#endif  // SEMANTIC_TRAVERSABILITY_LAYER__SEMANTIC_COSTMAP_LAYER_HPP_
