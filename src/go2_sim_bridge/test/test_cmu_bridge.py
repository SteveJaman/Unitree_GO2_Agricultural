import math

from geometry_msgs.msg import Twist

from go2_sim_bridge.cmu_bridge import clamp, twist_to_sim


def make_twist(vx=0.0, vy=0.0, wz=0.0):
    t = Twist()
    t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
    return t


def test_clamp_limits_both_signs():
    assert clamp(2.0, 0.5) == 0.5
    assert clamp(-2.0, 0.5) == -0.5
    assert clamp(0.2, 0.5) == 0.2


def test_clamp_disabled_when_limit_non_positive():
    assert clamp(5.0, 0.0) == 5.0


def test_twist_to_sim_applies_limits():
    assert twist_to_sim(make_twist(1.0, 0.3, -2.0), 0.5, 0.0, 1.0) == (0.5, 0.3, -1.0)


def test_twist_to_sim_zero_lateral_limit_means_unlimited():
    # max_vy=0 disables clamping, matching the clamp() contract.
    assert twist_to_sim(make_twist(vy=0.2), 0.5, 0.0, 1.0)[1] == 0.2


def test_twist_to_sim_rejects_nan():
    assert twist_to_sim(make_twist(math.nan, 0.0, 0.1), 0.5, 0.0, 1.0) == (0.0, 0.0, 0.0)
