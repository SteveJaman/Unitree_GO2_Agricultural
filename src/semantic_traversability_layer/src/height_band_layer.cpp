#include "semantic_traversability_layer/height_band_layer.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

PLUGINLIB_EXPORT_CLASS(
  semantic_traversability_layer::HeightBandLayer,
  nav2_costmap_2d::Layer)

namespace semantic_traversability_layer
{

HeightBandLayer::HeightBandLayer()
: enabled_(false),
  ground_z_(0.0),
  ground_tolerance_(0.02),
  step_over_height_max_(0.15),
  lethal_height_min_(0.25),
  max_range_(5.0),
  step_over_cost_(40),
  caution_cost_(200),
  robot_x_(0.0), robot_y_(0.0),
  has_robot_pose_(false)
{
}

void HeightBandLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("HeightBandLayer: failed to lock node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(false));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/lidar3d/obstacle_points")));
  declareParameter("ground_z", rclcpp::ParameterValue(0.0));
  declareParameter("ground_tolerance", rclcpp::ParameterValue(0.02));
  declareParameter("step_over_height_max", rclcpp::ParameterValue(0.15));
  declareParameter("lethal_height_min", rclcpp::ParameterValue(0.25));
  declareParameter("max_range", rclcpp::ParameterValue(5.0));
  declareParameter("step_over_cost", rclcpp::ParameterValue(40));
  declareParameter("caution_cost", rclcpp::ParameterValue(200));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".ground_z", ground_z_);
  node->get_parameter(name_ + ".ground_tolerance", ground_tolerance_);
  node->get_parameter(name_ + ".step_over_height_max", step_over_height_max_);
  node->get_parameter(name_ + ".lethal_height_min", lethal_height_min_);
  node->get_parameter(name_ + ".max_range", max_range_);

  int soc, cc;
  node->get_parameter(name_ + ".step_over_cost", soc);
  node->get_parameter(name_ + ".caution_cost", cc);
  step_over_cost_ = static_cast<unsigned char>(std::clamp(soc, 0, 253));
  caution_cost_ = static_cast<unsigned char>(std::clamp(cc, 0, 253));

  global_frame_ = layered_costmap_->getGlobalFrameID();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (enabled_) {
    cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_, rclcpp::SensorDataQoS(),
      std::bind(&HeightBandLayer::pointCloudCallback, this, std::placeholders::_1));
    RCLCPP_INFO(node->get_logger(),
      "HeightBandLayer: topic=%s step_over<%.2f lethal>=%.2f",
      topic_.c_str(), step_over_height_max_, lethal_height_min_);
  } else {
    RCLCPP_INFO(node->get_logger(), "HeightBandLayer: disabled");
  }
}

void HeightBandLayer::pointCloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!enabled_) {return;}

  sensor_msgs::msg::PointCloud2 cloud;
  if (msg->header.frame_id == global_frame_) {
    cloud = *msg;
  } else {
    try {
      auto tf = tf_buffer_->lookupTransform(
        global_frame_, msg->header.frame_id, tf2::TimePointZero);
      tf2::doTransform(*msg, cloud, tf);
    } catch (const tf2::TransformException & ex) {
      auto node = node_.lock();
      if (node) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
          "HeightBandLayer: tf %s -> %s failed: %s",
          msg->header.frame_id.c_str(), global_frame_.c_str(), ex.what());
      }
      return;
    }
  }

  std::vector<ClassifiedPoint> new_points;
  new_points.reserve(cloud.width * cloud.height / 4);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const float px = *iter_x;
    const float py = *iter_y;
    const float pz = *iter_z;

    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
      continue;
    }

    if (has_robot_pose_) {
      const double dx = px - robot_x_;
      const double dy = py - robot_y_;
      if (dx * dx + dy * dy > max_range_ * max_range_) {
        continue;
      }
    }

    const double h = pz - ground_z_;

    unsigned char cost = 0;
    if (h < ground_tolerance_) {
      continue;
    } else if (h < step_over_height_max_) {
      cost = step_over_cost_;
    } else if (h < lethal_height_min_) {
      cost = caution_cost_;
    } else {
      cost = nav2_costmap_2d::LETHAL_OBSTACLE;
    }

    new_points.push_back({static_cast<double>(px), static_cast<double>(py), cost});
  }

  std::lock_guard<std::mutex> lock(points_mutex_);
  classified_points_ = std::move(new_points);
}

void HeightBandLayer::updateBounds(
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

void HeightBandLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {return;}

  std::lock_guard<std::mutex> lock(points_mutex_);

  for (const auto & pt : classified_points_) {
    unsigned int mx, my;
    if (!master_grid.worldToMap(pt.wx, pt.wy, mx, my)) {
      continue;
    }
    const int ix = static_cast<int>(mx);
    const int iy = static_cast<int>(my);
    if (ix < min_i || ix >= max_i || iy < min_j || iy >= max_j) {
      continue;
    }

    const unsigned char existing = master_grid.getCost(mx, my);
    if (pt.cost > existing || existing == nav2_costmap_2d::NO_INFORMATION) {
      master_grid.setCost(mx, my, pt.cost);
    }
  }
}

void HeightBandLayer::reset()
{
  std::lock_guard<std::mutex> lock(points_mutex_);
  classified_points_.clear();
}

}  // namespace semantic_traversability_layer
