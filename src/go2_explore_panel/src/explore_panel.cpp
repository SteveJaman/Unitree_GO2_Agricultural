#include "go2_explore_panel/explore_panel.hpp"

#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>

// These two headers define the full DisplayContext type and its
// getRosNodeAbstraction() method. Without them, the compiler only sees
// the forward declaration from panel.hpp.
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>

namespace go2_explore_panel
{

ExplorePanel::ExplorePanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout;

  status_label_ = new QLabel("Exploration: IDLE");
  status_label_->setStyleSheet("QLabel { color: #aaa; font-size: 12px; }");
  layout->addWidget(status_label_);

  toggle_button_ = new QPushButton("Auto Explore: START");
  toggle_button_->setMinimumHeight(36);
  toggle_button_->setStyleSheet(
    "QPushButton { background-color: #2a7; color: white; "
    "font-weight: bold; border-radius: 4px; }"
    "QPushButton:hover { background-color: #3b8; }");
  layout->addWidget(toggle_button_);

  setLayout(layout);

  connect(toggle_button_, &QPushButton::clicked,
          this, &ExplorePanel::onToggleClicked);
}

ExplorePanel::~ExplorePanel() = default;

void ExplorePanel::onInitialize()
{
  // getDisplayContext() is an instance method on rviz_common::Panel —
  // call it via `this`, not as a static.
  auto ctx = getDisplayContext();
  if (!ctx) {
    status_label_->setText("Exploration: NO RViz CONTEXT");
    return;
  }

  auto abstraction = ctx->getRosNodeAbstraction().lock();
  if (!abstraction) {
    status_label_->setText("Exploration: NO ROS NODE");
    return;
  }

  node_ = abstraction->get_raw_node();

  resume_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/explore/resume", 10);
}

void ExplorePanel::onToggleClicked()
{
  if (!resume_pub_) return;

  exploring_ = !exploring_;

  std_msgs::msg::Bool msg;
  msg.data = exploring_;
  resume_pub_->publish(msg);

  if (exploring_) {
    status_label_->setText("Exploration: RUNNING");
    status_label_->setStyleSheet("QLabel { color: #2a7; font-weight: bold; }");
    toggle_button_->setText("Auto Explore: STOP");
    toggle_button_->setStyleSheet(
      "QPushButton { background-color: #c33; color: white; "
      "font-weight: bold; border-radius: 4px; }"
      "QPushButton:hover { background-color: #d44; }");
  } else {
    status_label_->setText("Exploration: STOPPED");
    status_label_->setStyleSheet("QLabel { color: #c33; font-weight: bold; }");
    toggle_button_->setText("Auto Explore: START");
    toggle_button_->setStyleSheet(
      "QPushButton { background-color: #2a7; color: white; "
      "font-weight: bold; border-radius: 4px; }"
      "QPushButton:hover { background-color: #3b8; }");
  }
}

}  // namespace go2_explore_panel

PLUGINLIB_EXPORT_CLASS(go2_explore_panel::ExplorePanel, rviz_common::Panel)
