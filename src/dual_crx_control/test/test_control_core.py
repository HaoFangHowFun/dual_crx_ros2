import math
import pytest

from dual_crx_control.core import BOTH, LEFT, READY, RIGHT, ControlCore, JOINTS


def complete(core, now=10.0):
    for arm in (LEFT, RIGHT):
        names = list(reversed(JOINTS[arm]))
        assert core.update_state(arm, now, names, list(range(6)), [0.0] * 6)[0]
        core.controllers[arm] = True
    core.refresh(now)


def test_readiness_requires_both_complete_fresh_and_controllers():
    core = ControlCore()
    assert not core.readiness(10.0)[0]
    assert not core.update_state(LEFT, 10.0, JOINTS[LEFT][:-1], [0] * 5, [0] * 5)[0]
    assert core.update_state(LEFT, 10.0, JOINTS[LEFT], [0] * 6, [0] * 6)[0]
    assert not core.readiness(10.0)[0]
    complete(core)
    assert core.readiness(10.0)[0]
    assert not core.readiness(11.0)[0]


def test_canonical_order_and_bad_state_rejected():
    core = ControlCore()
    names = list(reversed(JOINTS[LEFT]))
    assert core.update_state(LEFT, 1.0, names, range(6), [0] * 6)[0]
    assert core.canonical(LEFT)[0] == list(JOINTS[LEFT])
    assert core.canonical(LEFT)[1] == list(reversed(range(6)))
    assert not core.update_state(LEFT, 1.0, [JOINTS[LEFT][0]] * 6, [0] * 6, [0] * 6)[0]
    assert not core.update_state(LEFT, 1.0, JOINTS[LEFT], [math.nan] * 6, [0] * 6)[0]


def test_atomic_ownership_conflicts_release_and_timeout():
    core = ControlCore()
    complete(core)
    assert core.acquire("a", "GUI", LEFT, "JOG", 1.0, 10.0)[0]
    assert not core.acquire("b", "TEST", LEFT, "JOG", 1.0, 10.0)[0]
    assert not core.acquire("b", "TEST", BOTH, "JOINT", 1.0, 10.0)[0]
    assert RIGHT not in core.leases
    assert core.release("a", LEFT)[0]
    assert core.acquire("b", "TEST", BOTH, "JOINT", 0.5, 10.0)[0]
    core.refresh(10.6)
    assert not core.leases


def test_jog_validation_and_lease_deadman():
    core = ControlCore(max_jog_velocity=0.2)
    complete(core)
    core.acquire("gui", "GUI", LEFT, "JOG", 0.5, 10.0)
    assert core.validate_jog("gui", LEFT, ["left_J1"], [0.1], 10.0, 10.0)[0]
    assert not core.validate_jog("other", LEFT, ["left_J1"], [0.1], 10.0, 10.0)[0]
    assert not core.validate_jog("gui", LEFT, ["right_J1"], [0.1], 10.0, 10.0)[0]
    assert not core.validate_jog("gui", LEFT, ["left_J1"], [0.3], 10.0, 10.0)[0]
    assert not core.validate_jog("gui", LEFT, ["left_J1"], [math.inf], 10.0, 10.0)[0]
    assert not core.validate_jog("gui", LEFT, ["left_J1"], [0.1], 9.0, 10.0)[0]
    core.active_scope = LEFT
    core.refresh(10.6)
    assert core.active_scope is None


def test_goal_validation_and_physical_write_protection():
    core = ControlCore()
    complete(core)
    core.acquire("test", "TEST", BOTH, "JOINT", 1.0, 10.0)
    names = list(JOINTS[LEFT] + JOINTS[RIGHT])
    assert core.validate_goal("test", "both_arms", names, [0.0] * 12, 10.0)[0]
    assert not core.validate_goal("test", "both_arms", names[:-1], [0.0] * 11, 10.0)[0]
    assert not core.validate_goal("test", "both_arms", names, [math.nan] * 12, 10.0)[0]
    assert not core.validate_goal("test", "both_arms", names, [7.0] * 12, 10.0)[0]
    physical = ControlCore(physical=True)
    complete(physical)
    assert not physical.acquire("x", "GUI", LEFT, "JOG", 1.0, 10.0)[0]


def test_physical_control_can_be_explicitly_enabled():
    physical = ControlCore(physical=True, allow_physical_control=True)
    complete(physical)
    acquired, _, _ = physical.acquire("gui", "GUI", LEFT, "JOG", 1.0, 10.0)
    assert acquired
    assert physical.control_state == READY
    assert physical.validate_jog("gui", LEFT, ["left_J1"], [0.1], 10.0, 10.0)[0]


def test_cartesian_pose_validation():
    core = ControlCore(); complete(core)
    core.acquire("gui", "GUI", LEFT, "CARTESIAN", 1.0, 10.0)
    assert core.validate_cartesian_pose("gui", LEFT, "world", "left_flange",
                                        [0.5, 0.1, 0.4], [0.0, 0.0, 0.0, 1.0],
                                        10.0, 10.0)[0]
    assert not core.validate_cartesian_pose("gui", BOTH, "world", "left_flange",
                                            [0, 0, 0], [0, 0, 0, 1], 10.0, 10.0)[0]
    assert not core.validate_cartesian_pose("gui", LEFT, "", "left_flange",
                                            [0, 0, 0], [0, 0, 0, 1], 10.0, 10.0)[0]
    assert not core.validate_cartesian_pose("gui", LEFT, "world", "right_flange",
                                            [0, 0, 0], [0, 0, 0, 1], 10.0, 10.0)[0]
    assert not core.validate_cartesian_pose("gui", LEFT, "world", "left_flange",
                                            [0, 0, 0], [0, 0, 0, 2], 10.0, 10.0)[0]


def test_cartesian_jog_validation():
    core = ControlCore(); complete(core)
    core.acquire("gui", "GUI", LEFT, "CARTESIAN_JOG", 1.0, 10.0)
    assert core.validate_cartesian_jog("gui", LEFT, "world", "left_flange",
                                       [0.01, 0, 0], [0, 0, 0], 10.0, 10.0)[0]
    assert not core.validate_cartesian_jog("gui", LEFT, "world", "left_flange",
                                           [0.04, 0, 0], [0, 0, 0], 10.0, 10.0)[0]
    assert not core.validate_cartesian_jog("gui", LEFT, "world", "left_flange",
                                           [0, 0, 0], [0, 0, 0.3], 10.0, 10.0)[0]
    assert not core.validate_cartesian_jog("gui", RIGHT, "world", "right_flange",
                                           [0.01, 0, 0], [0, 0, 0], 10.0, 10.0)[0]
