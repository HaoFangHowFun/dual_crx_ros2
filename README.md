# dual_crx_ros2

Phase 1 workspace for a dual FANUC CRX-5iA ROS 2 Jazzy mock bring-up.

The mock baseline is developed on ROS 2 Jazzy. Physical deployment is also
prepared for Ubuntu 22.04 / ROS 2 Humble by using the official FANUC `humble`
branch. See [`docs/physical_bringup_ubuntu22.md`](docs/physical_bringup_ubuntu22.md).

## Ubuntu 22.04 / ROS 2 Humble environment installation

The physical PC should run native Ubuntu 22.04. Install ROS 2 Humble Desktop by
following the official ROS 2 Ubuntu deb-package instructions first:

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

Initialize `rosdep` once on a new machine. If it was initialized previously,
skip `rosdep init` and only run `rosdep update`:

```bash
sudo rosdep init
rosdep update
```

Place or extract this repository at `~/dual_crx_ros2`. For example, when using
the prepared bundle:

```bash
cd ~
tar -xzf dual_crx_ros2_ubuntu22_bundle_2026-08-24.tar.gz
```

Import the official FANUC dependencies. The driver must use its `humble` branch;
`main` targets ROS 2 Jazzy and must not be used on Ubuntu 22.04 / Humble.

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ws_fanuc/src
vcs import ~/ws_fanuc/src < ~/dual_crx_ros2/fanuc_humble.repos
git -C ~/ws_fanuc/src/fanuc_driver submodule update --init --recursive
```

Install missing dependencies and build the official FANUC workspace:

```bash
cd ~/ws_fanuc
rosdep install --ignore-src --from-paths src -y --rosdistro humble
colcon build --symlink-install \
  --cmake-args -DBUILD_TESTING=1 -DBUILD_EXAMPLES=1
```

Build the project overlay:

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
- `ros2` and `colcon` resolve successfully
- `AMENT_PREFIX_PATH` contains both `~/ws_fanuc/install` and
  `~/dual_crx_ros2/install`
- both package-prefix commands return paths without errors

The helper scripts automatically source Humble or Jazzy plus both workspaces, so
normal project use does not require adding workspace sources to `.bashrc`.

## Layout

```text
dual_crx_ros2/
├── .repos
├── README.md
└── src/
    ├── dual_crx_bringup/
    ├── dual_crx_control/
    ├── dual_crx_description/
    └── dual_crx_moveit_config/
```

## Upstream dependency

This workspace expects the official FANUC packages to already be available in the environment.
On this machine they are currently built in `/home/howard/ws_fanuc`.

Before building or launching this workspace:

```bash
source /opt/ros/jazzy/setup.bash
source /home/howard/ws_fanuc/install/setup.bash
```

## Build

```bash
cd /home/howard/dual_crx_ros2
colcon build --symlink-install
source install/setup.bash
```

## Launch

Mock hardware:

```bash
ros2 launch dual_crx_bringup dual_crx.launch.py use_mock:=true
```

Real hardware placeholder interface:

```bash
./scripts/preflight_real.sh \
  --left-ip 192.168.2.100 \
  --right-ip 192.168.1.100

# Connection-only; does not request robot motion authority.
./scripts/launch_real.sh --arm right
./scripts/launch_real.sh --arm left
./scripts/launch_real.sh --arm both
```

The real-hardware helper defaults to `motion_control=0`. Requesting authority
requires both `--enable-motion` and `--yes-i-understand`, after the independent
connection and safety checks have passed.

The current focus is the mock dual-arm pipeline:

- two independent controller managers
- merged `/joint_states`
- shared `world` frame
- MoveIt planning groups `left_arm`, `right_arm`, `both_arms`

## Control demo

With the mock bring-up running, plan a small collision-aware joint-space motion without
executing it:

```bash
ros2 run dual_crx_control dual_crx_demo --ros-args \
  -p group:=left_arm \
  -p execute:=false
```

Supported groups are `left_arm`, `right_arm`, and `both_arms`. After planning-only
validation succeeds, set `execute:=true` to send the planned trajectory to the mock
controllers. The demo defaults to conservative velocity and acceleration scaling of
`0.10`.

### Verified mock result

Verified on 2026-08-13:

- planning-only succeeded for `left_arm`, `right_arm`, and `both_arms`
- execution succeeded for all three groups
- `both_arms` dispatched trajectories to both namespaced trajectory controllers
- the final merged `/joint_states` matched the 12 requested joint targets
- MoveIt's state-validity service accepted the demo target and rejected a deliberately
  intersecting dual-arm state with contacts between left- and right-arm links

The standalone demo currently emits a `No kinematics plugins defined` warning because
it does not load `kinematics.yaml` into its own node. This is non-blocking for the
joint-space targets used here; the central `move_group` process does load both KDL
plugins. Load the kinematics parameters into the demo before extending it to Cartesian
pose targets.

During Ctrl+C shutdown in the verified WSL session, `move_group` exited with `-11`
while tearing down and both slider GUI processes required `SIGKILL`. All ROS processes
were gone afterward. This did not affect planning or execution, but clean launch
shutdown remains a known issue to investigate.
