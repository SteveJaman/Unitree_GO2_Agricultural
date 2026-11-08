#include "legged_mppi_critic/step_over_critic.hpp"

#include <cstddef>

namespace mppi::critics
{

void StepOverCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(cost_weight_, "cost_weight", 0.2f);
  getParam(step_over_cost_min_, "step_over_cost_min", 30);
  getParam(step_over_cost_max_, "step_over_cost_max", 60);
  RCLCPP_INFO(
    logger_, "StepOverCritic: reward %.3f per step-over cell, cost band [%d, %d]",
    cost_weight_, step_over_cost_min_, step_over_cost_max_);
}

void StepOverCritic::score(CriticData & data)
{
  if (!enabled_ || costmap_ == nullptr) {
    return;
  }
  const auto & xs = data.trajectories.x;
  const auto & ys = data.trajectories.y;
  const std::size_t batch = xs.shape(0);
  const std::size_t steps = xs.shape(1);

  for (std::size_t i = 0; i < batch; ++i) {
    unsigned int last_mx = 0, last_my = 0;
    bool have_last = false;
    int count = 0;
    for (std::size_t t = 0; t < steps; ++t) {
      unsigned int mx, my;
      if (!costmap_->worldToMap(xs(i, t), ys(i, t), mx, my)) {
        have_last = false;
        continue;
      }
      // A cell counts once per visit: consecutive poses in one cell are one crossing.
      if (have_last && mx == last_mx && my == last_my) {
        continue;
      }
      last_mx = mx;
      last_my = my;
      have_last = true;
      const int c = costmap_->getCost(mx, my);
      if (c >= step_over_cost_min_ && c <= step_over_cost_max_) {
        ++count;
      }
    }
    data.costs(i) -= cost_weight_ * static_cast<float>(count);
  }
}

}  // namespace mppi::critics

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mppi::critics::StepOverCritic, mppi::critics::CriticFunction)
