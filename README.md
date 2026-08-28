# dual_crx_ros2

Two FANUC CRX-5iA robots in one ROS 2 / MoveIt environment.

- Mock development: Ubuntu 22.04 / ROS 2 Humble
- Physical deployment: Ubuntu 22.04 / ROS 2 Humble
- Physical-host details: [`docs/physical_bringup_ubuntu22.md`](docs/physical_bringup_ubuntu22.md)
- Unified API/GUI Phase 1: [`docs/control_gui_phase1.md`](docs/control_gui_phase1.md)

## Unified control API and GUI

The project-owned control server is the only supported command gateway. It enforces
complete/fresh dual-arm state, leases, validation, mock/physical mode, planning,
execution, cancellation, and stop semantics. The standalone PyQt5 GUI is a client and
contains no direct trajectory-controller command path.

The GUI also includes Phase 2 single-arm Cartesian TCP pose Validate/Plan/Execute and
press-and-hold MoveIt Servo jog for `X/Y/Z/Rx/Ry/Rz`. Both paths remain behind the same
gateway lease, deadman, validation, Stop and physical read-only enforcement.

Run the automated mock acceptance workflow:

```bash
./scripts/test_control_gui_phase1_mock.sh
```

### Interactive GUI quickstart (mock only)

Build once after pulling or changing the source:

```bash
cd /home/howard/dual_crx_ros2
./scripts/build.sh
```

Open two WSL terminals. In terminal 1, start both mock arms, MoveIt, the unified control
server, MoveIt Servo, and RViz:

```bash
cd /home/howard/dual_crx_ros2
./scripts/launch_mock.sh
```

Wait for both trajectory controllers to become active. In terminal 2, start the GUI:

```bash
cd /home/howard/dual_crx_ros2
source scripts/codex_env.sh
ros2 run dual_crx_gui dual_crx_gui
```

The status area should show `Mode: mock`, `Control state: READY`, both controllers as
`active`, and `Freshness: fresh/complete`. Do not command motion while the state is
offline, stale, read-only, faulted, or owned by another client.

#### Joint jog

1. Open the **Jog** tab and select `left` or `right`.
2. Start with `1 deg/s`.
3. Press and hold a `J1..J6` `+` or `-` button. Motion continues only while held.
4. Release the button and verify motion stops. Focus loss, lease timeout, window close,
   and the red **STOP** button also stop the jog.

#### Joint-position planning

1. Open **Joint position** and select `left_arm`, `right_arm`, or `both_arms`.
2. Click **Copy current** to begin from the live state. GUI values are degrees; the ROS
   API uses radians.
3. Click **Validate**, then **Plan**. Editing a field never executes motion.
4. After `plan ready`, click **Execute**. Use **Cancel** or the red **STOP** when needed.

#### Cartesian pose planning

1. Open **Cartesian pose** and select the left or right arm.
2. Click **Copy current TCP**. Position fields are metres; orientation is a normalized
   quaternion (`qx`, `qy`, `qz`, `qw`) in the `world` frame.
3. Click **Validate**, **Plan**, and then **Execute** as separate operations.
4. Begin with the copied current pose before trying a small offset. MoveIt performs IK,
   joint-limit, and collision-aware planning before execution is available.

#### MoveIt Servo Cartesian jog

The bottom of **Cartesian pose** provides press-and-hold world-frame controls:

- `X/Y/Z +/-` translate the selected TCP.
- `Rx/Ry/Rz +/-` rotate the selected TCP.
- Start with `2 mm/s` linear and `1 deg/s` angular speed.
- Release stops the command stream. The gateway and Servo command timeouts provide
  additional deadman behavior.

GUI speed choices are bounded to 2–20 mm/s and 1–10 deg/s. The gateway independently
enforces maximums of 0.03 m/s and 0.2 rad/s, validates ownership, timestamp, TF, and
arm/TCP pairing, and remains read-only in physical mode. MoveIt Servo also applies
joint-limit, singularity, and collision handling.

The red **STOP** is a software stop/cancel path. It is not a FANUC E-stop and does not
replace DCS or controller safety. Close the GUI, then press `Ctrl+C` in terminal 1 when
testing is complete. Do not use `launch_real.sh` or any `send_*motion.sh` command for
this mock GUI procedure.

## Important: mock and physical commands are different

| Goal | Command | Can a real robot move? |
|---|---|---|
| Start RViz with two simulated arms | `./scripts/launch_mock.sh` | No |
| Plan only in mock | `dual_crx_demo ... execute:=false` | No |
| Execute a trajectory in mock | `dual_crx_demo ... execute:=true` | No |
| Check a physical connection | `launch_real.sh --arm ...` | No; defaults to `motion_control=0` |
| Request physical motion authority | add `--enable-motion --yes-i-understand` | Authority only; the launch itself sends no target |

Never use the fixed mock demo target as the first physical-robot motion. The values in
`dual_crx_demo` and `robot_placement.yaml` have only been validated in mock hardware.

## 1. Clone this repository

GitHub is the primary installation method:

```bash
cd ~
git clone git@github.com:HaoFangHowFun/dual_crx_ros2.git
cd ~/dual_crx_ros2
```

If SSH is not configured on the physical PC, use HTTPS:

```bash
cd ~
git clone https://github.com/HaoFangHowFun/dual_crx_ros2.git
cd ~/dual_crx_ros2
```

Because the repository is private, HTTPS will require a GitHub username and personal
access token. A `.tar.gz` bundle is only an offline backup.

## 2. Install Ubuntu 22.04 / ROS 2 Humble environment

The physical PC should run native Ubuntu 22.04. First install ROS 2 Humble Desktop using
the official Ubuntu deb-package instructions:

- <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

Then install the project tools and ROS dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  git \
  git-lfs

git lfs install
```

Initialize `rosdep` once on a new machine. If `rosdep init` reports that it was already
initialized, continue with `rosdep update`:

```bash
sudo rosdep init
rosdep update
```

## 3. Download and build the official FANUC Humble dependencies

The FANUC driver must use its official `humble` branch. Its `main` branch targets newer ROS 2 releases
and must not be used on Ubuntu 22.04 / Humble.

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ws_fanuc/src
vcs import ~/ws_fanuc/src < ~/dual_crx_ros2/fanuc_humble.repos
git -C ~/ws_fanuc/src/fanuc_driver submodule update --init --recursive

cd ~/ws_fanuc
rosdep install --ignore-src --from-paths src -y --rosdistro humble
colcon build --symlink-install \
  --cmake-args -DBUILD_TESTING=1 -DBUILD_EXAMPLES=1
```

Build this project overlay:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_fanuc/install/setup.bash
cd ~/dual_crx_ros2
colcon build --symlink-install
source install/setup.bash
```

Verify the environment:

```bash
cd ~/dual_crx_ros2
./scripts/doctor.sh
ros2 pkg prefix fanuc_hardware_interface
ros2 pkg prefix dual_crx_bringup
```

Expected results:

- `ROS_DISTRO=humble`
- `ros2` and `colcon` are found
- `AMENT_PREFIX_PATH` contains `ws_fanuc/install` and `dual_crx_ros2/install`
- both package-prefix commands return paths without errors

The project scripts source ROS and both workspaces automatically. There is no need to
edit `.bashrc` for these tests.

## 4. Mock test: no physical robots required

Use this test after installation and before connecting physical robots.

### Terminal 1: start the two mock arms, MoveIt, and RViz

```bash
cd ~/dual_crx_ros2
./scripts/launch_mock.sh
```

Wait until RViz displays both arms and both trajectory controllers are active. Keep this
terminal running.

### Terminal 2: plan without executing

Open a second terminal:

```bash
cd ~/dual_crx_ros2
source scripts/codex_env.sh

ros2 run dual_crx_control dual_crx_demo --ros-args \
  -p group:=left_arm \
  -p execute:=false
```

Expected: `Planning succeeded for 'left_arm'`. Repeat for the other planning groups:

```bash
ros2 run dual_crx_control dual_crx_demo --ros-args \
  -p group:=right_arm \
  -p execute:=false

ros2 run dual_crx_control dual_crx_demo --ros-args \
  -p group:=both_arms \
  -p execute:=false
```

### Terminal 2: execute on mock hardware only

After all three planning-only commands pass, execute the same targets in simulation:

```bash
ros2 run dual_crx_control dual_crx_demo --ros-args \
  -p group:=left_arm \
  -p execute:=true

ros2 run dual_crx_control dual_crx_demo --ros-args \
  -p group:=right_arm \
  -p execute:=true

ros2 run dual_crx_control dual_crx_demo --ros-args \
  -p group:=both_arms \
  -p execute:=true
```

Expected: RViz shows the requested motion and each command reports `Execution succeeded`.
Return to Terminal 1 and press `Ctrl+C` when finished.

## 5. Physical test stage 1: network preflight

Recommended network:

| Link | Robot | PC NIC |
|---|---|---|
| Right | `192.168.1.100/24` | `192.168.1.101/24` |
| Left | `192.168.2.100/24` | `192.168.2.101/24` |

Run this before starting any ROS driver:

```bash
cd ~/dual_crx_ros2
./scripts/preflight_real.sh \
  --left-ip 192.168.2.100 \
  --right-ip 192.168.1.100
```

Resolve every `FAIL`. Review every `WARN`. This command only checks the computer,
routes, packages, and ping; it cannot move a robot.

## 6. Physical test stage 2: connection-only, one arm at a time

The following commands use `motion_control=0`. They connect and read state without
requesting robot motion authority.

### Right arm

Terminal 1:

```bash
cd ~/dual_crx_ros2
./scripts/launch_real.sh --arm right
```

Keep it running. In Terminal 2:

```bash
cd ~/dual_crx_ros2
source scripts/codex_env.sh

ros2 control list_controllers -c /right_arm/controller_manager
ros2 topic echo --once /right_arm/joint_states
ros2 topic echo --once /right_arm/fanuc_gpio_controller/robot_status
```

Expected:

- the controller manager responds
- six right-arm joint positions are received
- `in_error: false`
- motion authority may remain false because this is connection-only mode

Press `Ctrl+C` in Terminal 1 before testing the left arm.

### Left arm

Terminal 1:

```bash
cd ~/dual_crx_ros2
./scripts/launch_real.sh --arm left
```

Terminal 2:

```bash
cd ~/dual_crx_ros2
source scripts/codex_env.sh

ros2 control list_controllers -c /left_arm/controller_manager
ros2 topic echo --once /left_arm/joint_states
ros2 topic echo --once /left_arm/fanuc_gpio_controller/robot_status
```

Press `Ctrl+C` in Terminal 1 when finished.

### Both arms, still connection-only

Only after both independent connection tests pass:

```bash
cd ~/dual_crx_ros2
./scripts/launch_real.sh --arm both
```

This starts both drivers and MoveIt but still defaults to `motion_control=0`. It should
not command motion.

## 7. Physical test stage 3: request motion authority

Do this only after the controller software/options, payload, DCS, E-stop, workspace,
AUTO mode, alarms, teach-pendant state, and TP programs have been checked.

Start with one arm only:

```bash
cd ~/dual_crx_ros2
./scripts/launch_real.sh \
  --arm right \
  --enable-motion \
  --yes-i-understand
```

This launch requests motion authority but does not itself send a target. In a second
terminal, verify the status:

```bash
cd ~/dual_crx_ros2
source scripts/codex_env.sh
ros2 topic echo --once /right_arm/fanuc_gpio_controller/robot_status
```

Expected: `motion_possible: true` with no alarm or E-stop state.

Do not run `dual_crx_demo` with `execute:=true` on a physical arm. Use the guarded
scripts below so the target is generated from the robot's live joint state.

### First physical motion: single-joint step

The repository now includes a conservative joint-motion script:

```bash
cd ~/dual_crx_ros2
./scripts/send_small_joint_motion.sh \
  --arm right \
  --joint J6 \
  --delta 0.02 \
  --duration 3.0 \
  --yes-i-understand
```

This command reads the selected arm's `joint_states`, checks its
`fanuc_gpio_controller/robot_status`, and sends a single-point
`FollowJointTrajectory` goal to that arm's `joint_trajectory_controller`.
Use `--arm left`, `--arm right`, or `--arm both` depending on the target.
The older `send_small_right_motion.sh` entry point remains as a compatibility
wrapper around this generic script.

### Periodic physical motion: single joint

For bounded oscillation around the current joint state:

```bash
cd ~/dual_crx_ros2
./scripts/send_periodic_joint_motion.sh \
  --arm right \
  --joint J6 \
  --amplitude 0.01 \
  --period 5.0 \
  --cycles 2 \
  --samples-per-cycle 16 \
  --yes-i-understand
```

This script also accepts `--arm left` and `--arm both`. The legacy
`send_periodic_right_motion.sh` name is kept as a wrapper to preserve the old
workflow.

### Periodic physical motion: multiple joints

For coordinated motion with per-joint amplitudes and phase offsets:

```bash
cd ~/dual_crx_ros2
./scripts/send_multi_joint_periodic_motion.sh \
  --arm right \
  --joint-motion-deg J4:10:0 \
  --joint-motion-deg J5:10:90 \
  --joint-motion-deg J6:10:180 \
  --period 6.0 \
  --cycles 2 \
  --samples-per-cycle 16 \
  --allow-large-amplitude \
  --yes-i-understand
```

The multi-joint script now forces the first trajectory point to match the live joint
state exactly before the periodic motion starts. This avoids the startup jump that
occurred when a joint was phase-shifted away from zero at `t=0`.
It accepts `--arm left`, `--arm right`, and `--arm both`; the older
`send_multi_joint_periodic_right_motion.sh` wrapper forwards to it.

## Project layout

```text
dual_crx_ros2/
├── docs/
├── scripts/
├── fanuc_humble.repos
└── src/
    ├── dual_crx_bringup/
    ├── dual_crx_control/
    ├── dual_crx_description/
    └── dual_crx_moveit_config/
```

## Verified mock baseline

Verified on ROS 2 Humble mock hardware on 2026-08-25:

- planning and execution passed for `left_arm`, `right_arm`, and `both_arms`
- both namespaced trajectory controllers received the dual-arm trajectory
- merged `/joint_states` reached all 12 requested joint targets
- MoveIt accepted the demo target and rejected a deliberately colliding dual-arm state

Known mock warnings:

- the standalone joint-space demo does not load kinematics plugins; central `move_group`
  loads them correctly
- WSL mock shutdown can require forced termination of RViz slider processes
- the repeatable inter-arm avoidance scenario and RViz evidence are still pending

## Physical progress on 2026-08-26

- right-arm connection-only and motion-authority bringup passed on Ubuntu 22.04 / ROS 2 Humble
- the first physical small-motion test succeeded with a guarded single-joint trajectory
- periodic right-arm motion succeeded for single-joint oscillation
- coordinated multi-joint periodic motion succeeded
- a startup acceleration spike was observed with phase-shifted multi-joint motion, then mitigated by forcing the first trajectory point to the exact live joint state
