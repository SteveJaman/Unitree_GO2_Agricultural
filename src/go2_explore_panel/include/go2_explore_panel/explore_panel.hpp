#ifndef GO2_EXPLORE_PANEL__EXPLORE_PANEL_HPP_
#define GO2_EXPLORE_PANEL__EXPLORE_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <QLabel>
#include <QPushButton>

namespace go2_explore_panel
{

class ExplorePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ExplorePanel(QWidget * parent = nullptr);
  ~ExplorePanel() override;

  void onInitialize() override;

private Q_SLOTS:
  void onToggleClicked();

private:
  QLabel * status_label_;
  QPushButton * toggle_button_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr resume_pub_;
  rclcpp::Node::SharedPtr node_;
  bool exploring_{false};
};

}  // namespace go2_explore_panel

#endif  // GO2_EXPLORE_PANEL__EXPLORE_PANEL_HPP_
