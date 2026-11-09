#include "semantic_traversability_layer/height_band_layer.hpp"

#include <algorithm>
#include <cmath>
#include <string>
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
  floor_estimation_("fixed"),
  floor_window_(0.15), floor_bin_(0.005), floor_search_radius_(3.0),
  floor_smoothing_frames_(5), floor_min_points_(20),
  ground_tolerance_(0.02),
  step_over_height_max_(0.15),
  lethal_height_min_(0.25),
  min_range_(0.0),
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
  // fixed: the floor is ground_z. histogram: per frame, the most populated
  // floor_bin z-bin within +-floor_window of ground_z among points within
  // floor_search_radius of the robot, median-smoothed over floor_smoothing_frames
  // (Yassine Salimi's estimate_floor). For a robot whose body pitches and rolls.
  declareParameter("floor_estimation", rclcpp::ParameterValue(std::string("fixed")));
  declareParameter("floor_window", rclcpp::ParameterValue(0.15));
  declareParameter("floor_bin", rclcpp::ParameterValue(0.005));
  declareParameter("floor_search_radius", rclcpp::ParameterValue(3.0));
  declareParameter("floor_smoothing_frames", rclcpp::ParameterValue(5));
  declareParameter("floor_min_points", rclcpp::ParameterValue(20));
  declareParameter("step_over_height_max", rclcpp::ParameterValue(0.15));
  declareParameter("lethal_height_min", rclcpp::ParameterValue(0.25));
  declareParameter("min_range", rclcpp::ParameterValue(0.0));
  declareParameter("max_range", rclcpp::ParameterValue(5.0));
  declareParameter("step_over_cost", rclcpp::ParameterValue(40));
  declareParameter("caution_cost", rclcpp::ParameterValue(200));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".ground_z", ground_z_);
  node->get_parameter(name_ + ".ground_tolerance", ground_tolerance_);
  node->get_parameter(name_ + ".floor_estimation", floor_estimation_);
  node->get_parameter(name_ + ".floor_window", floor_window_);
  node->get_parameter(name_ + ".floor_bin", floor_bin_);
  node->get_parameter(name_ + ".floor_search_radius", floor_search_radius_);
  node->get_parameter(name_ + ".floor_smoothing_frames", floor_smoothing_frames_);
  node->get_parameter(name_ + ".floor_min_points", floor_min_points_);
  if (floor_estimation_ != "fixed" && floor_estimation_ != "histogram") {
    RCLCPP_WARN(node->get_logger(), "HeightBandLayer: floor_estimation '%s' unknown, using fixed",
      floor_estimation_.c_str());
    floor_estimation_ = "fixed";
  }
  node->get_parameter(name_ + ".step_over_height_max", step_over_height_max_);
  node->get_parameter(name_ + ".lethal_height_min", lethal_height_min_);
  node->get_parameter(name_ + ".min_range", min_range_);
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
      "HeightBandLayer: topic=%s min_range=%.2f step_over<%.2f lethal>=%.2f floor=%s ground_z=%.3f",
      topic_.c_str(), min_range_, step_over_height_max_, lethal_height_min_,
      floor_estimation_.c_str(), ground_z_);
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

  const double ground = floor_estimation_ == "histogram" ? estimateFloor(cloud) : ground_z_;

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
      const double dist_sq = dx * dx + dy * dy;
      if (dist_sq > max_range_ * max_range_ ||
          dist_sq < min_range_ * min_range_) {
        continue;
      }
    }

    const double h = pz - ground;

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

double HeightBandLayer::estimateFloor(const sensor_msgs::msg::PointCloud2 & cloud)
{
  // Most populated floor_bin_ bin within +-floor_window_ of ground_z_, refined by the
  // median of the points within 1 cm of it, then median over the last frames.
  const int nbins = std::max(1, static_cast<int>(std::ceil(2.0 * floor_window_ / floor_bin_)));
  std::vector<int> counts(nbins, 0);
  std::vector<float> zs;
  zs.reserve(cloud.width * cloud.height / 2);
  const double r2 = floor_search_radius_ * floor_search_radius_;
  sensor_msgs::PointCloud2ConstIterator<float> ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z");
  for (; ix != ix.end(); ++ix, ++iy, ++iz) {
    const float z = *iz;
    if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(z)) {continue;}
    if (std::abs(z - ground_z_) >= floor_window_) {continue;}
    if (has_robot_pose_) {
      const double dx = *ix - robot_x_, dy = *iy - robot_y_;
      if (dx * dx + dy * dy > r2) {continue;}
    }
    const int b = static_cast<int>((z - (ground_z_ - floor_window_)) / floor_bin_);
    if (b < 0 || b >= nbins) {continue;}
    ++counts[b];
    zs.push_back(z);
  }

  double frame = floor_history_.empty() ? ground_z_ : floor_history_.back();
  if (static_cast<int>(zs.size()) >= floor_min_points_) {
    const int k = static_cast<int>(std::max_element(counts.begin(), counts.end()) - counts.begin());
    const double centre = ground_z_ - floor_window_ + (k + 0.5) * floor_bin_;
    std::vector<float> near;
    for (float z : zs) {
      if (std::abs(z - centre) < 0.01) {near.push_back(z);}
    }
    if (!near.empty()) {
      std::nth_element(near.begin(), near.begin() + near.size() / 2, near.end());
      frame = near[near.size() / 2];
    } else {
      frame = centre;
    }
    floor_history_.push_back(frame);
    while (static_cast<int>(floor_history_.size()) > std::max(1, floor_smoothing_frames_)) {
      floor_history_.pop_front();
    }
  }
  std::vector<double> h(floor_history_.begin(), floor_history_.end());
  double smoothed = ground_z_;
  if (!h.empty()) {
    std::nth_element(h.begin(), h.begin() + h.size() / 2, h.end());
    smoothed = h[h.size() / 2];
  }
  auto node = node_.lock();
  if (node) {
    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
      "HeightBandLayer: ground_z frame=%.4f smoothed=%.4f (n=%zu)", frame, smoothed, zs.size());
  }
  return smoothed;
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
