#include "semantic_traversability_layer/semantic_costmap_layer.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  semantic_traversability_layer::SemanticCostmapLayer,
  nav2_costmap_2d::Layer)

namespace semantic_traversability_layer
{

SemanticCostmapLayer::SemanticCostmapLayer()
: enabled_(false),
  max_range_(8.0),
  confidence_min_(0.7),
  avoid_cost_(254), caution_cost_(200), step_over_cost_(0),
  robot_x_(0.0), robot_y_(0.0),
  has_robot_pose_(false)
{
}

void SemanticCostmapLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("SemanticCostmapLayer: failed to lock node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(false));
  declareParameter("topic", rclcpp::ParameterValue(
    std::string("/semantic/obstacles")));
  declareParameter("max_range", rclcpp::ParameterValue(8.0));
  // Obstacles below confidence_min are ignored. A message with a label gets its
  // cost from these; one without (older publishers) keeps the cost it carries.
  declareParameter("confidence_min", rclcpp::ParameterValue(0.7));
  declareParameter("avoid_cost", rclcpp::ParameterValue(254));
  declareParameter("caution_cost", rclcpp::ParameterValue(200));
  declareParameter("step_over_cost", rclcpp::ParameterValue(0));
  // Added on each side of an oriented footprint (size_x/size_y > 0). Obstacles
  // without one are marked as a disc of their radius.
  declareParameter("footprint_margin", rclcpp::ParameterValue(0.05));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".max_range", max_range_);
  node->get_parameter(name_ + ".confidence_min", confidence_min_);
  node->get_parameter(name_ + ".avoid_cost", avoid_cost_);
  node->get_parameter(name_ + ".caution_cost", caution_cost_);
  node->get_parameter(name_ + ".step_over_cost", step_over_cost_);
  node->get_parameter(name_ + ".footprint_margin", footprint_margin_);

  if (enabled_) {
    sub_ = node->create_subscription<
      go2_semantic_msgs::msg::SemanticObstacleArray>(
      topic_, rclcpp::SensorDataQoS(),
      std::bind(&SemanticCostmapLayer::obstacleCallback, this,
                std::placeholders::_1));
    RCLCPP_INFO(node->get_logger(),
      "SemanticCostmapLayer: topic=%s max_range=%.1f confidence_min=%.2f "
      "costs avoid=%d caution=%d step_over=%d footprint_margin=%.2f",
      topic_.c_str(), max_range_, confidence_min_, avoid_cost_, caution_cost_, step_over_cost_,
      footprint_margin_);
  } else {
    RCLCPP_INFO(node->get_logger(), "SemanticCostmapLayer: disabled");
  }
}

void SemanticCostmapLayer::obstacleCallback(
  go2_semantic_msgs::msg::SemanticObstacleArray::ConstSharedPtr msg)
{
  if (!enabled_) {return;}

  std::vector<CostMark> new_marks;
  new_marks.reserve(msg->obstacles.size());

  for (const auto & obs : msg->obstacles) {
    if (obs.confidence < confidence_min_) {continue;}
    const int cost = labelCost(obs.label, obs.cost);
    if (cost <= 0) {continue;}

    if (has_robot_pose_) {
      const double dx = obs.position.x - robot_x_;
      const double dy = obs.position.y - robot_y_;
      if (dx * dx + dy * dy > max_range_ * max_range_) {
        continue;
      }
    }

    CostMark mk;
    mk.wx = obs.position.x;
    mk.wy = obs.position.y;
    mk.radius = obs.radius;
    mk.cost = static_cast<unsigned char>(std::min(cost, 254));
    // A long object (a bookshelf) as a disc would block far more than the object.
    mk.oriented = obs.size_x > 0.0f && obs.size_y > 0.0f;
    mk.hx = mk.oriented ? obs.size_x / 2.0 + footprint_margin_ : 0.0;
    mk.hy = mk.oriented ? obs.size_y / 2.0 + footprint_margin_ : 0.0;
    mk.yaw = obs.yaw;
    new_marks.push_back(mk);
  }

  std::lock_guard<std::mutex> lock(marks_mutex_);
  marks_ = std::move(new_marks);
}

int SemanticCostmapLayer::labelCost(const std::string & label, int msg_cost) const
{
  if (label.empty()) {return msg_cost;}
  if (label == "AVOID") {return avoid_cost_;}
  if (label == "CAUTION") {return caution_cost_;}
  if (label == "STEP_OVER") {return step_over_cost_;}
  return 0;  // UNKNOWN or unrecognised: no cost
}

void SemanticCostmapLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!enabled_) {return;}

  robot_x_ = robot_x;
  robot_y_ = robot_y;
  has_robot_pose_ = true;

  *min_x = std::min(*min_x, robot_x - max_range_);
  *min_y = std::min(*min_y, robot_y - max_range_);
  *max_x = std::max(*max_x, robot_x + max_range_);
  *max_y = std::max(*max_y, robot_y + max_range_);
}

void SemanticCostmapLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {return;}

  std::lock_guard<std::mutex> lock(marks_mutex_);

  for (const auto & mk : marks_) {
    const double res = master_grid.getResolution();
    const double reach = mk.oriented ? std::hypot(mk.hx, mk.hy) : mk.radius;
    const int r_cells = static_cast<int>(std::ceil(reach / res));
    const double c = std::cos(mk.yaw), s = std::sin(mk.yaw);

    unsigned int cx, cy;
    if (!master_grid.worldToMap(mk.wx, mk.wy, cx, cy)) {
      continue;
    }

    for (int dx = -r_cells; dx <= r_cells; ++dx) {
      for (int dy = -r_cells; dy <= r_cells; ++dy) {
        if (dx * dx + dy * dy > r_cells * r_cells) {continue;}

        const int ix = static_cast<int>(cx) + dx;
        const int iy = static_cast<int>(cy) + dy;
        if (ix < min_i || ix >= max_i || iy < min_j || iy >= max_j) {
          continue;
        }

        const unsigned int mx = static_cast<unsigned int>(ix);
        const unsigned int my = static_cast<unsigned int>(iy);
        if (mk.oriented) {
          double px, py;
          master_grid.mapToWorld(mx, my, px, py);
          const double ox = px - mk.wx, oy = py - mk.wy;
          if (std::abs(c * ox + s * oy) > mk.hx || std::abs(-s * ox + c * oy) > mk.hy) {
            continue;
          }
        }
        const unsigned char existing = master_grid.getCost(mx, my);
        if (mk.cost > existing || existing == nav2_costmap_2d::NO_INFORMATION) {
          master_grid.setCost(mx, my, mk.cost);
        }
      }
    }
  }
}

void SemanticCostmapLayer::reset()
{
  std::lock_guard<std::mutex> lock(marks_mutex_);
  marks_.clear();
}

}  // namespace semantic_traversability_layer
