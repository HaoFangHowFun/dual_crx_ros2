# Physical bring-up on Ubuntu 22.04

This guide prepares an Ubuntu 22.04 / ROS 2 Humble PC for two physical FANUC
CRX-5iA robots. The first launch must be connection-only. Do not request motion
authority until each arm has independently passed the checks below.

## 1. Controller prerequisites

Verify on both teach pendants:

- R-30iB Mini Plus software V9.40P/77 or later
- J519 Stream Motion and R912 Remote Motion, or S636 External Control Package
- payload, tool, DCS, mastering, and joint limits are correct
- E-stop access and the reduced-speed test procedure are understood

For motion authority, the controller must be in AUTO, alarms must be cleared,
the teach pendant must be disabled, and all TP programs must be aborted.

## 2. Recommended network

| Link | Device | Address | Netmask | Gateway / DNS |
|---|---|---|---|---|
| Right dedicated Ethernet | right CRX | `192.168.1.100` | `/24` | none |
| Right dedicated Ethernet | PC NIC | `192.168.1.101` | `/24` | none |
| Left dedicated Ethernet | left CRX | `192.168.2.100` | `/24` | none |
| Left dedicated Ethernet | PC NIC | `192.168.2.101` | `/24` | none |

Use different physical NICs and subnets. Keep Internet access on Wi-Fi or a third
NIC. Do not configure a default gateway on either robot link.

Identify interface names before creating NetworkManager profiles:

```bash
nmcli device status
ip -brief link
```

Example only; replace both interface names after tracing the cables:

```bash
sudo nmcli connection add type ethernet ifname enp1s0 con-name crx-right \
  ipv4.method manual ipv4.addresses 192.168.1.101/24 \
  ipv4.never-default yes ipv6.method disabled

sudo nmcli connection add type ethernet ifname enp2s0 con-name crx-left \
  ipv4.method manual ipv4.addresses 192.168.2.101/24 \
  ipv4.never-default yes ipv6.method disabled
```

## 3. Install the Humble upstream workspace

Install ROS 2 Humble Desktop, MoveIt 2, ros2_control, colcon, vcstool, git-lfs,
and rosdep first. Then import the FANUC Humble branch:

```bash
mkdir -p ~/ws_fanuc/src
vcs import ~/ws_fanuc/src < ~/dual_crx_ros2/fanuc_humble.repos
cd ~/ws_fanuc
rosdep install --ignore-src --from-paths src -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=1 -DBUILD_EXAMPLES=1
```

Build this overlay:

```bash
cd ~/dual_crx_ros2
./scripts/build.sh
./scripts/doctor.sh
```

Expected: `ROS_DISTRO=humble`, both workspaces in `AMENT_PREFIX_PATH`, and all
four project packages built.

## 4. Network and software preflight

With both robot Ethernet cables connected:

```bash
cd ~/dual_crx_ros2
./scripts/preflight_real.sh \
  --left-ip 192.168.2.100 \
  --right-ip 192.168.1.100
```

Resolve every `FAIL`. Review every `WARN`. Confirm that the two robot routes use
the intended NICs:

```bash
ip route get 192.168.1.100
ip route get 192.168.2.100
```

## 5. Connection-only tests

Start the right arm without requesting motion authority:

```bash
./scripts/launch_real.sh --arm right
```

In a second terminal:

```bash
source ~/dual_crx_ros2/scripts/codex_env.sh
ros2 control list_controllers -c /right_arm/controller_manager
ros2 topic echo --once /right_arm/joint_states
ros2 topic echo --once /right_arm/fanuc_gpio_controller/robot_status
```

Stop the right-arm launch cleanly, then repeat for the left arm:

```bash
./scripts/launch_real.sh --arm left
```

If the first connection reports `RMIT-016 Please Cycle Power`, repower the robot
controller and restart the driver as directed by the FANUC documentation.

## 6. First controlled motion

Only after the connection-only test, controller checklist, workspace clearance,
and E-stop test have passed, request motion authority for one arm:

```bash
./scripts/launch_real.sh --arm right --enable-motion --yes-i-understand
```

Verify `motion_possible: true`. Use a conservative, small target and keep only
one arm enabled. Repeat independently on the left arm. Do not start with the
`both_arms` demo target because it was validated against approximate mock base
transforms, not calibrated physical transforms.

## 7. Dual-arm connection test

After both independent tests pass, connect both without motion authority:

```bash
./scripts/launch_real.sh --arm both
```

Requesting authority for both arms is deliberately explicit:

```bash
./scripts/launch_real.sh --arm both --enable-motion --yes-i-understand
```

Before executing any collision-aware dual-arm plan, measure and update
`src/dual_crx_description/config/robot_placement.yaml`. The current values are
simulation placeholders and are not safe physical calibration values.

## 8. Evidence to save

Record the following after the test:

- Ubuntu and ROS distribution
- FANUC driver commit or tag
- both controller software versions and installed option names
- NIC names, PC addresses, robot addresses, and routes
- controller list and robot-status output for each namespace
- actual base transforms and payload/tool schedules
- alarms, timing warnings, and whether each single-arm motion passed
