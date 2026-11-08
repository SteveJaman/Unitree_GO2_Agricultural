#ifndef LEGGED_MPPI_CRITIC__STEP_OVER_CRITIC_HPP_
#define LEGGED_MPPI_CRITIC__STEP_OVER_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"

namespace mppi::critics
{

/**
 * Rewards trajectories by the number of distinct step-over cells they cross.
 *
 * For every sampled trajectory the critic walks the local-costmap cells under
 * the trajectory's poses (robot centre), counts the cells whose cost lies in
 * [step_over_cost_min, step_over_cost_max], and subtracts
 * cost_weight * count from that trajectory's cost. Caution and lethal cells
 * are not touched here: CostCritic / ObstaclesCritic already penalise them.
 *
 * Parameters (under the critic's name):
 *   enabled              (true)
 *   cost_weight          (0.2)
 *   step_over_cost_min   (30)
 *   step_over_cost_max   (60)
 */
class StepOverCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  float cost_weight_{0.2f};
  int step_over_cost_min_{30};
  int step_over_cost_max_{60};
};

}  // namespace mppi::critics

#endif  // LEGGED_MPPI_CRITIC__STEP_OVER_CRITIC_HPP_
