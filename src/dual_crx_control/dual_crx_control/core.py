"""Policy-independent safety, state and lease logic (no ROS dependencies)."""

from dataclasses import dataclass
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

LEFT, RIGHT, BOTH = 1, 2, 3
OFFLINE, READ_ONLY, READY, JOGGING, PLANNING, EXECUTING, STOPPING, FAULT, ESTOP = range(9)
JOINTS = {
    LEFT: tuple(f"left_J{i}" for i in range(1, 7)),
    RIGHT: tuple(f"right_J{i}" for i in range(1, 7)),
}
GROUP_JOINTS = {
    "left_arm": JOINTS[LEFT],
    "right_arm": JOINTS[RIGHT],
    "both_arms": JOINTS[LEFT] + JOINTS[RIGHT],
}


@dataclass
class Lease:
    client_id: str
    source_type: str
    mode: str
    expires_at: float


class ControlCore:
    """Deterministic state/ownership validation used by ROS and unit tests."""

    def __init__(self, *, physical=False, state_timeout=0.5, max_jog_velocity=0.2,
                 min_lease=0.1, max_lease=5.0):
        self.physical = physical
        self.state_timeout = state_timeout
        self.max_jog_velocity = max_jog_velocity
        self.min_lease = min_lease
        self.max_lease = max_lease
        self.states: Dict[int, Tuple[float, Dict[str, Tuple[float, float]]]] = {}
        self.leases: Dict[int, Lease] = {}
        self.controllers = {LEFT: False, RIGHT: False}
        self.active_scope: Optional[int] = None
        self.control_state = READ_ONLY if physical else OFFLINE
        self.reason = "physical mode is read-only" if physical else "waiting for complete state"

    @staticmethod
    def arms(scope: int) -> Tuple[int, ...]:
        if scope == LEFT:
            return (LEFT,)
        if scope == RIGHT:
            return (RIGHT,)
        if scope == BOTH:
            return (LEFT, RIGHT)
        return ()

    def update_state(self, arm: int, stamp: float, names: Sequence[str],
                     positions: Sequence[float], velocities: Sequence[float]) -> Tuple[bool, str]:
        expected = JOINTS.get(arm)
        if expected is None:
            return False, "invalid arm"
        if len(names) != 6 or len(set(names)) != 6 or set(names) != set(expected):
            return False, "joint state must contain each expected joint exactly once"
        if len(positions) != 6 or (velocities and len(velocities) != 6):
            return False, "joint state array length mismatch"
        velocity_values = velocities if velocities else [0.0] * 6
        if not all(math.isfinite(x) for x in list(positions) + list(velocity_values)):
            return False, "joint state contains non-finite values"
        incoming = dict(zip(names, zip(positions, velocity_values)))
        self.states[arm] = (stamp, {name: incoming[name] for name in expected})
        return True, "accepted"

    def readiness(self, now: float) -> Tuple[bool, str]:
        for arm, label in ((LEFT, "left"), (RIGHT, "right")):
            if arm not in self.states:
                return False, f"missing {label} joint state"
            if now - self.states[arm][0] > self.state_timeout or self.states[arm][0] > now + 0.1:
                return False, f"{label} joint state is stale or future-dated"
            if not self.controllers[arm]:
                return False, f"{label} controller unavailable"
        return True, "ready"

    def refresh(self, now: float) -> None:
        expired = [arm for arm, lease in self.leases.items() if lease.expires_at <= now]
        for arm in expired:
            self.leases.pop(arm, None)
        if expired and self.active_scope and any(a in expired for a in self.arms(self.active_scope)):
            self.active_scope = None
            self.reason = "lease heartbeat timed out; motion stopped"
        ready, reason = self.readiness(now)
        if self.physical:
            self.control_state = READ_ONLY
            self.reason = "physical mode is read-only"
        elif not ready:
            self.control_state = OFFLINE
            self.reason = reason
        elif self.control_state in (OFFLINE, READ_ONLY, JOGGING, STOPPING):
            self.control_state = READY
            if not expired:
                self.reason = "ready"

    def acquire(self, client_id: str, source_type: str, scope: int, mode: str,
                duration: float, now: float) -> Tuple[bool, str, float]:
        self.refresh(now)
        arms = self.arms(scope)
        if not client_id.strip() or not arms:
            return False, "client_id and valid arm scope are required", 0.0
        if self.physical:
            return False, "physical mode is read-only", 0.0
        ready, reason = self.readiness(now)
        if not ready:
            return False, reason, 0.0
        duration = min(max(duration, self.min_lease), self.max_lease)
        conflicts = [a for a in arms if a in self.leases and self.leases[a].client_id != client_id]
        if conflicts:
            return False, "arm is owned by another client", 0.0
        expiry = now + duration
        lease = Lease(client_id, source_type or "UNKNOWN", mode or "JOINT", expiry)
        for arm in arms:  # BOTH is committed only after the conflict check.
            self.leases[arm] = lease
        return True, "acquired", expiry

    def heartbeat(self, client_id: str, scope: int, duration: float, now: float):
        self.refresh(now)
        arms = self.arms(scope)
        if not arms or any(a not in self.leases or self.leases[a].client_id != client_id for a in arms):
            return False, "client does not own requested scope", 0.0
        expiry = now + min(max(duration, self.min_lease), self.max_lease)
        for arm in arms:
            self.leases[arm].expires_at = expiry
        return True, "renewed", expiry

    def release(self, client_id: str, scope: int) -> Tuple[bool, str]:
        arms = self.arms(scope)
        owned = [a for a in arms if a in self.leases and self.leases[a].client_id == client_id]
        if len(owned) != len(arms) or not arms:
            return False, "client does not own requested scope"
        for arm in arms:
            self.leases.pop(arm, None)
        if self.active_scope and any(a in arms for a in self.arms(self.active_scope)):
            self.active_scope = None
        return True, "released"

    def require_owner(self, client_id: str, scope: int, now: float) -> Tuple[bool, str]:
        self.refresh(now)
        if self.physical:
            return False, "physical mode is read-only"
        arms = self.arms(scope)
        if not arms or any(a not in self.leases or self.leases[a].client_id != client_id for a in arms):
            return False, "client does not own requested scope"
        return True, "accepted"

    def validate_jog(self, client_id: str, scope: int, names: Sequence[str],
                     velocities: Sequence[float], stamp: float, now: float) -> Tuple[bool, str]:
        owned, reason = self.require_owner(client_id, scope, now)
        if not owned:
            return False, reason
        if scope == BOTH:
            return False, "Phase 1 jog supports one arm at a time"
        if abs(now - stamp) > 0.25:
            return False, "jog timestamp is stale or future-dated"
        if len(names) != 1 or len(velocities) != 1 or names[0] not in JOINTS[scope]:
            return False, "jog must select exactly one canonical joint for its arm"
        velocity = velocities[0]
        if not math.isfinite(velocity) or abs(velocity) > self.max_jog_velocity:
            return False, "jog velocity is non-finite or exceeds configured limit"
        if self.active_scope not in (None, scope):
            return False, "another arm scope has an active command"
        return True, "accepted"

    def validate_goal(self, client_id: str, group: str, names: Sequence[str],
                      positions: Sequence[float], now: float) -> Tuple[bool, str, int]:
        expected = GROUP_JOINTS.get(group)
        scope = {"left_arm": LEFT, "right_arm": RIGHT, "both_arms": BOTH}.get(group, 0)
        if expected is None:
            return False, "unknown planning group", scope
        owned, reason = self.require_owner(client_id, scope, now)
        if not owned:
            return False, reason, scope
        if len(names) != len(expected) or len(set(names)) != len(expected) or set(names) != set(expected):
            return False, "target must contain each group joint exactly once", scope
        if len(positions) != len(expected) or not all(math.isfinite(x) for x in positions):
            return False, "target positions are incomplete or non-finite", scope
        # A conservative absolute guard catches unit mistakes; MoveIt enforces model limits/collision.
        if any(abs(x) > 2.0 * math.pi for x in positions):
            return False, "target exceeds absolute safety bound", scope
        if self.active_scope is not None:
            return False, "jog or planned command is already active", scope
        return True, "accepted", scope

    def validate_cartesian_pose(self, client_id: str, scope: int, frame_id: str,
                                tcp_link: str, position: Sequence[float],
                                orientation: Sequence[float], stamp: float,
                                now: float) -> Tuple[bool, str]:
        owned, reason = self.require_owner(client_id, scope, now)
        if not owned:
            return False, reason
        if scope not in (LEFT, RIGHT):
            return False, "Phase 2 Cartesian pose supports one arm at a time"
        expected_tcp = "left_flange" if scope == LEFT else "right_flange"
        if tcp_link != expected_tcp:
            return False, f"tcp_link must be {expected_tcp}"
        if not frame_id.strip():
            return False, "target frame_id is required"
        if stamp and abs(now - stamp) > 0.5:
            return False, "Cartesian target timestamp is stale or future-dated"
        values = list(position) + list(orientation)
        if len(position) != 3 or len(orientation) != 4 or not all(math.isfinite(x) for x in values):
            return False, "Cartesian pose contains incomplete or non-finite values"
        norm = math.sqrt(sum(x * x for x in orientation))
        if abs(norm - 1.0) > 1e-3:
            return False, "Cartesian orientation quaternion must be normalized"
        if any(abs(x) > 5.0 for x in position):
            return False, "Cartesian position exceeds absolute workspace guard"
        if self.active_scope is not None:
            return False, "jog or planned command is already active"
        return True, "accepted"

    def validate_cartesian_jog(self, client_id: str, scope: int, frame_id: str,
                               tcp_link: str, linear: Sequence[float],
                               angular: Sequence[float], stamp: float,
                               now: float) -> Tuple[bool, str]:
        owned, reason = self.require_owner(client_id, scope, now)
        if not owned:
            return False, reason
        if scope not in (LEFT, RIGHT):
            return False, "Cartesian jog supports one arm at a time"
        expected_tcp = "left_flange" if scope == LEFT else "right_flange"
        if tcp_link != expected_tcp or not frame_id.strip():
            return False, f"valid frame and tcp_link={expected_tcp} are required"
        if abs(now - stamp) > 0.25:
            return False, "Cartesian jog timestamp is stale or future-dated"
        values = list(linear) + list(angular)
        if len(linear) != 3 or len(angular) != 3 or not all(math.isfinite(x) for x in values):
            return False, "Cartesian jog contains incomplete or non-finite values"
        if math.sqrt(sum(x * x for x in linear)) > 0.03 + 1e-9:
            return False, "Cartesian linear speed exceeds 0.03 m/s limit"
        if math.sqrt(sum(x * x for x in angular)) > 0.2 + 1e-9:
            return False, "Cartesian angular speed exceeds 0.2 rad/s limit"
        if self.active_scope not in (None, scope):
            return False, "another arm scope has an active command"
        if self.control_state in (PLANNING, EXECUTING):
            return False, "planned command conflicts with Cartesian jog"
        return True, "accepted"

    def canonical(self, arm: int) -> Tuple[List[str], List[float], List[float]]:
        values = self.states[arm][1]
        return list(JOINTS[arm]), [values[n][0] for n in JOINTS[arm]], [values[n][1] for n in JOINTS[arm]]
