# Dual CRX unified control API — Phase 1

This API is the only supported client command boundary. The GUI, tests, and future
RL/teleoperation clients acquire a lease and use `/dual_crx/*`; they must never send
goals to a namespaced trajectory controller. All ROS-boundary joint values are radians.
The GUI converts display/input values to and from degrees.

## Start and verify

Run the complete non-interactive mock gate:

```bash
./scripts/test_control_gui_phase1_mock.sh
```

The script builds the relevant packages, selects an isolated `ROS_DOMAIN_ID`, starts
mock bring-up without RViz, runs core and headless GUI tests, exercises both arms
through the public API, and terminates only its own launch process group. Artifacts are
reported under `/tmp/dual_crx_phase1_*`.

For interactive mock use:

```bash
./scripts/launch_mock.sh
# In another terminal:
source scripts/codex_env.sh
ros2 run dual_crx_gui dual_crx_gui
```

The red **STOP** control is a software cancellation path. It is not a FANUC E-stop and
does not replace DCS or any controller safety function.

## Public names

| Name | Type | Purpose |
|---|---|---|
| `/dual_crx/state` | `dual_crx_interfaces/msg/SystemState` | Canonical state, freshness, controllers, owners, mode, active command and reason |
| `/dual_crx/acquire_control` | `AcquireControl` service | Atomically lease LEFT, RIGHT or BOTH |
| `/dual_crx/heartbeat` | `Heartbeat` service | Renew the deadman lease |
| `/dual_crx/release_control` | `ReleaseControl` service | Release owned arm scope |
| `/dual_crx/jog` | `JointJog` service | Submit one bounded, timestamped standard `control_msgs/JointJog` sample |
| `/dual_crx/joint_position` | `JointPosition` action | VALIDATE, PLAN, then EXECUTE a stored collision-aware plan |
| `/dual_crx/cartesian_pose` | `CartesianPose` action | LEFT/RIGHT TCP pose VALIDATE, PLAN, then EXECUTE by `plan_id` |
| `/dual_crx/cartesian_jog` | `CartesianJog` service | Gateway-validated `TwistStamped` stream to per-arm MoveIt Servo |
| `/dual_crx/workcell_info` | `GetWorkcellInfo` service | Exact loaded placement path, profile, validity, offsets and SHA-256 |
| `/dual_crx/calibration_check` | `CalibrationCheck` action | Plan-only or guarded sequential left/right five-point verification |
| `/dual_crx/stop` | `SoftwareStop` service | High-priority cancellation independent of normal workflow |

`SystemState` is not complete until each namespaced input contains its exact six
canonical names. `fresh` additionally requires both timestamps within the configured
timeout and both trajectory action servers available. Canonical order is
`left_J1..left_J6`, then `right_J1..right_J6`.

## Client sequence

1. Generate a stable `client_id` and acquire the required scope with source `GUI`,
   `TEST`, `RL`, or `TELEOP`.
2. Renew the lease before expiry. Loss of heartbeat invalidates active jog and releases
   ownership. `BOTH` acquisition is all-or-nothing and Phase 1 never preempts.
3. For jog, stream one canonical joint and bounded velocity while a deadman input is
   held. Send Stop on release, focus loss, client shutdown, stale state, or fault.
4. For position control, send `PLAN` with exactly 6 or 12 names/positions. On success,
   retain `plan_id`; send `EXECUTE` with that ID as a separate action goal.
5. Observe action feedback/results and `/dual_crx/state`; release ownership when done.

## Cartesian frames and pose control

The GUI Cartesian pose tab supports one arm at a time. The frame selector resolves to
either `world` (identical to the calibrated `table_frame`) or the selected arm's
`left_base_link/right_base_link`. Switching frames transforms the displayed pose so it
does not silently reinterpret the same numbers. `Copy current TCP` queries the selected
TF directly. Targets use metres and a normalized quaternion. Validate checks ownership,
arm/TCP pairing, timestamp, TF, finite values and workspace guard; Plan adds MoveIt
position/orientation constraints for collision-aware IK/planning; Execute dispatches
only the stored approved plan.

The same tab provides press-and-hold `X/Y/Z/Rx/Ry/Rz` jog using either the selected pose
reference or the selected tool/TCP frame. Linear speed is
selectable from 2–20 mm/s and angular speed from 1–10 deg/s. The gateway enforces hard
ceilings of 0.03 m/s and 0.2 rad/s, exact arm/TCP ownership, fresh timestamps and TF,
then forwards the accepted stream to the selected MoveIt Servo backend. Release,
focus loss, heartbeat timeout and software Stop end the stream; Servo additionally
applies its command timeout, joint-limit, singularity and collision handling.

MoveIt validates model bounds and collisions during PLAN. Only the gateway splits an
approved dual-arm `RobotTrajectory` and dispatches it to both namespaced controllers.
This preserves one policy-independent gateway while avoiding controller details in
clients.

## Loaded workcell identity and five-point check

The launch passes the resolved `robot_placement_file` to the gateway. The workcell-info
service reports what this running process actually loaded: world/table frame names,
absolute file path, profile name, generation time, validity, signed EEF Z offsets and
the file SHA-256. Changing or activating a file does not alter a running robot model;
restart the physical launch to apply it.

`CalibrationCheck` acquires BOTH arms atomically and enforces a 15 mm clearance floor.
It takes the five table points from `inferred_square.point_coordinates_m`, applies each
arm's measurement-frame EEF Z correction, and captures the current flange orientation.
Plan-only checks all target poses without controller execution. Execute uses low 5%
MoveIt velocity/acceleration scaling, approaches and retracts at a higher transit plane,
runs all left points, returns the left arm to its captured joints, then repeats for the
right arm. Missing/invalid calibration, unavailable TF, unsafe physical robot status,
lease loss, planning failure, controller failure, Cancel, focus loss or Stop terminates
the sequence without advancing to another point.

## Modes and extension boundary

`operating_mode=mock` enables validated writes. Physical writes require the launch-time
`allow_physical_control` opt-in and remain behind the same ownership and readiness
checks. Future RL and teleoperation nodes should implement the same
lease/heartbeat/action sequence; they do not receive a privileged controller path.
Force/torque and remote/browser clients remain outside this phase.

The legacy `scripts/send_*motion.sh` commands directly address controllers and are kept
only as guarded migration/diagnostic references. New clients must not copy that path.

## Current validation platform

Automated implementation validation was run on Ubuntu 24.04 / ROS 2 Jazzy. Humble
package build/runtime validation remains explicitly unverified until the Ubuntu 22.04 /
ROS 2 Humble target is available.
