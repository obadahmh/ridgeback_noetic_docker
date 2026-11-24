# Ridgeback Noetic Docker

This container is for talking from this Ubuntu 24.04 workstation to a Ridgeback running Ubuntu 20.04 and ROS Noetic.

The container uses host networking because ROS 1 nodes advertise their own host address. It is configured for:

- Robot ROS master: `http://192.168.131.1:11311`
- Workstation wired IP: `192.168.131.100`
- Robot hostname mapping: `cpr-r100-0601 -> 192.168.131.1`

These values can be changed in `.env`. Start from:

```bash
cp .env.example .env
```

For Ethernet:

```bash
./scripts/use-ethernet
```

For wireless, pass the robot Wi-Fi IP and offboard Wi-Fi IP:

```bash
./scripts/use-wireless ROBOT_WIFI_IP OFFBOARD_WIFI_IP
```

Example:

```bash
./scripts/use-wireless 10.180.68.123 10.180.68.46
```

## Install Docker

Docker is not installed on this machine yet. Run:

```bash
sudo apt update
sudo apt install docker.io docker-compose-v2
sudo usermod -aG docker "$USER"
newgrp docker
```

If `newgrp docker` does not update the current terminal cleanly, log out and back in.

## Build

```bash
cd ~/ridgeback_noetic_docker
./scripts/build
```

## Check The Robot

```bash
cd ~/ridgeback_noetic_docker
./scripts/test-robot
```

You should see ROS 1 topics from the Ridgeback. Useful manual checks inside `./scripts/shell`:

```bash
rostopic list
rostopic echo /odom
rostopic echo /diagnostics
rosnode list
```

## Drive With Keyboard

Make sure the Ridgeback is enabled and safely lifted or in a clear area before publishing velocity commands.

```bash
cd ~/ridgeback_noetic_docker
./scripts/teleop
```

## Drive With Text Commands

The text-command executor is a small safety layer for future LLM control. It accepts only a tiny command language, checks hard limits, uses odometry to stop each motion, and publishes zero velocity before and after every command.

First test commands without publishing to the robot:

```bash
cd ~/ridgeback_noetic_docker
./scripts/text-control --dry-run "forward 0.5m"
./scripts/text-control --dry-run "turn left 45deg"
./scripts/text-control --dry-run "forward 5m"
```

Then run the interactive executor in a clear area:

```bash
cd ~/ridgeback_noetic_docker
./scripts/text-control
```

During a motion command, use `Ctrl-C`, the Ridgeback run-stop, or the physical E-stop if anything looks wrong. The `stop` command is useful between commands; motion commands are intentionally short and bounded.

Accepted commands:

```bash
status
stop
forward 0.5m
back 20cm
turn left 45deg
turn right 90deg
quit
```

Default safety limits are conservative: 1.0 m maximum translation, 90 deg maximum turn, 0.12 m/s linear speed, and 0.35 rad/s angular speed. You can make a single one-shot command too:

```bash
./scripts/text-control "forward 0.25m"
```

Structured JSON commands are also supported. This is the interface the LLM bridge uses:

```bash
./scripts/text-control --dry-run --json '{"action":"move","direction":"forward","distance_m":0.25}'
./scripts/text-control --dry-run --json '{"action":"turn","direction":"left","angle_deg":30}'
./scripts/text-control --dry-run --json '{"action":"stop"}'
```

Allowed JSON actions are `move`, `turn`, `stop`, and `status`. Unsupported fields are rejected.

## Drive With K2-Think

The LLM bridge asks K2-Think to translate natural language into one JSON command, then sends that JSON through the same safety validator used by `text-control`. The LLM never publishes directly to `/cmd_vel`.

Configure an OpenAI-compatible K2-Think endpoint:

```bash
export K2_THINK_BASE_URL=http://localhost:8000/v1
export K2_THINK_MODEL=LLM360/K2-Think-V2
export K2_THINK_API_KEY=your_key_if_needed
```

Use dry-run first:

```bash
cd ~/ridgeback_noetic_docker
./scripts/llm-control --dry-run "move forward a quarter meter"
./scripts/llm-control --dry-run "turn left thirty degrees"
./scripts/llm-control --dry-run "go explore the lab"
```

The bridge should print the LLM JSON and whether it was accepted:

```text
K2 JSON: {"action": "move", "direction": "forward", "distance_m": 0.25}
Accepted: move forward 0.25 m
Dry run only. Nothing published.
```

When dry-run behavior is boring and predictable, run live in a clear area:

```bash
./scripts/llm-control "move forward a quarter meter"
```

The same default safety limits apply: 1.0 m maximum translation, 90 deg maximum turn, slow speeds, fresh odometry required, and zero velocity before/after each command.

## Mapping First

For autonomous navigation, start by building a 2D occupancy-grid map of the environment. The usual ROS 1 flow is:

1. Verify laser, odometry, and TF.
2. Run SLAM while slowly teleoping the robot around the environment.
3. Save the map.
4. Use the saved map with AMCL localization and `move_base`.

Rebuild after navigation packages are added:

```bash
cd ~/ridgeback_noetic_docker
./scripts/build
```

Check that the robot has the topics and transforms needed for mapping:

```bash
cd ~/ridgeback_noetic_docker
./scripts/check-nav-prereqs
```

Confirm gmapping can start and publish a map without driving:

```bash
cd ~/ridgeback_noetic_docker
./scripts/slam-smoke-test
```

## Clearpath Ridgeback Mapping Baseline

Clearpath's Ridgeback navigation guide for ROS 1 says to run `ridgeback_navigation gmapping_demo.launch` and view it with `ridgeback_viz view_robot.launch config:=gmapping`. The Docker image already includes `ridgeback_viz`, and this workspace includes `ridgeback_navigation` from the Ridgeback source repo because the navigation package was not available from the configured apt repositories.

Build the workspace after cloning or changing ROS packages:

```bash
cd ~/ridgeback_noetic_docker
./scripts/build-ws
```

Stop any previous mapping/navigation nodes:

```bash
cd ~/ridgeback_noetic_docker
./scripts/stop-navigation
```

For the closest cautious first pass, run Clearpath's gmapping include without `move_base`:

Terminal 1:

```bash
cd ~/ridgeback_noetic_docker
./scripts/clearpath-gmapping
```

Terminal 2:

```bash
cd ~/ridgeback_noetic_docker
./scripts/clearpath-rviz-gmapping
```

Terminal 3:

```bash
cd ~/ridgeback_noetic_docker
./scripts/teleop
```

To run Clearpath's full tutorial launch exactly, use this instead of `./scripts/clearpath-gmapping`:

```bash
cd ~/ridgeback_noetic_docker
./scripts/clearpath-gmapping-demo
```

That full demo also starts `move_base`, matching Clearpath's tutorial.

Before starting SLAM, verify RViz can see the laser by itself:

```bash
cd ~/ridgeback_noetic_docker
./scripts/rviz-laser
```

You should see red laser points. This view uses `front_laser` as the fixed frame, so it is the simplest RViz test. If this is blank, debug RViz/display/laser before debugging SLAM.

The red laser points are not the map. They are recent sensor readings and they fade. The persistent map is the `/map` occupancy grid from gmapping. The default mapping scripts use Clearpath's `ridgeback_viz/rviz/gmapping.rviz` view.

Simple two-terminal SLAM session:

Terminal 1, start gmapping and RViz together:

```bash
cd ~/ridgeback_noetic_docker
./scripts/slam
```

Terminal 2, drive slowly:

```bash
cd ~/ridgeback_noetic_docker
./scripts/teleop
```

Save the map while `./scripts/slam` is still running:

```bash
cd ~/ridgeback_noetic_docker
./scripts/save-map first_floor
```

To inspect whether `/map` contains normal occupancy values while SLAM is running:

```bash
cd ~/ridgeback_noetic_docker
./scripts/map-stats
```

Alternative three-terminal flow:

```bash
./scripts/mapping
./scripts/rviz-mapping
./scripts/teleop
```

It is normal for `./scripts/rviz-mapping` to look empty until `./scripts/mapping` is running and publishing `/map`. For a live power view, open an optional terminal and run `./scripts/watch-power`.

Drive slow loops around the space. Avoid fast spins, wheel slip, bumping objects, and long featureless corridors without loop closures. When the map looks good, stop the robot with `k`, save while gmapping is still running, then stop gmapping with `Ctrl-C`:

```bash
cd ~/ridgeback_noetic_docker
./scripts/save-map first_floor
```

This creates:

```bash
~/ridgeback_noetic_docker/ws/maps/first_floor.yaml
~/ridgeback_noetic_docker/ws/maps/first_floor.pgm
```

To view a saved map later:

```bash
cd ~/ridgeback_noetic_docker
./scripts/view-map maps/first_floor.yaml
```

To inspect the raw saved map image without RViz coloring:

```bash
cd ~/ridgeback_noetic_docker
./scripts/open-map-image maps/first_floor.yaml
```

## Localize On A Saved Map

After saving a map, run AMCL localization before attempting autonomous navigation. Start with AMCL only:

Terminal 1:

```bash
cd ~/ridgeback_noetic_docker
./scripts/stop-navigation
./scripts/localize maps/lab_floor.yaml
```

Terminal 2:

```bash
cd ~/ridgeback_noetic_docker
./scripts/rviz-localization
```

In RViz, click `2D Pose Estimate`, then click and drag on the map where the robot actually is and which way it is facing. The laser scan should line up with the map. Then drive slowly with teleop in a third terminal:

```bash
cd ~/ridgeback_noetic_docker
./scripts/teleop
```

If the scan stays aligned with the map while you move, localization is working.

To run Clearpath's full localization/navigation demo, use this instead of `./scripts/localize`:

```bash
cd ~/ridgeback_noetic_docker
./scripts/localize-demo maps/lab_floor.yaml
```

That also starts `move_base`, so use it only after AMCL-only localization looks good.

## Slow Click-To-Go Navigation

After AMCL localization stays aligned while driving, try a cautious `move_base` goal. This uses Clearpath's costmaps with a slower DWA local planner, lower velocity and acceleration limits, and recovery spins disabled.

Terminal 1:

```bash
cd ~/ridgeback_noetic_docker
./scripts/stop-navigation
./scripts/navigate-slow maps/lab_floor.yaml
```

Terminal 2:

```bash
cd ~/ridgeback_noetic_docker
./scripts/rviz-localization
```

Terminal 3, keep this ready as an abort command:

```bash
cd ~/ridgeback_noetic_docker
./scripts/cancel-nav
```

In RViz:

1. Use `2D Pose Estimate` first and confirm the laser scan lines up with the map.
2. Click `2D Nav Goal`.
3. Pick a very nearby goal, around 0.5 m to 1.0 m away, in open space.
4. Keep a hand near the run-stop or physical E-stop.

Abort options:

```bash
./scripts/cancel-nav
./scripts/stop-navigation
Ctrl-C in the navigate-slow terminal
physical E-stop
```

Do not send long goals until short nearby goals are boring and repeatable.

## Diagnose Shutdowns Or Motor Faults

If the Ridgeback shuts down, disables, or faults when translating but can rotate in place, capture diagnostics before changing too many things. Common causes include battery voltage sag under load, motor controller faults, drivetrain overload, E-stop/enable state changes, or one side of the drivetrain drawing abnormal current.

Rebuild after pulling changes to this Dockerfile:

```bash
cd ~/ridgeback_noetic_docker
./scripts/build
```

Take a quick snapshot:

```bash
cd ~/ridgeback_noetic_docker
./scripts/diagnose
```

Record a one-minute bag while reproducing the failure:

```bash
cd ~/ridgeback_noetic_docker
./scripts/record-diagnostics 60
```

Then slowly reproduce the issue before the timer ends. The logs are written under:

```bash
~/ridgeback_noetic_docker/ws/logs/
```

For a live terminal view:

```bash
cd ~/ridgeback_noetic_docker
./scripts/monitor-diagnostics
```

For a compact power-only view while testing:

```bash
cd ~/ridgeback_noetic_docker
./scripts/watch-power
```

In a second terminal, drive very slowly:

```bash
cd ~/ridgeback_noetic_docker
./scripts/teleop
```

Watch these topics especially:

```bash
rostopic echo /diagnostics_agg
rostopic echo /diagnostics_toplevel_state
rostopic echo /mcu/status
rostopic echo /status
rostopic echo /feedback
rostopic echo /e_stop
```

Useful onboard checks over SSH:

```bash
ssh robot@192.168.131.1
rosnode list
rostopic echo /diagnostics_agg
rostopic echo /status
rostopic echo /feedback
journalctl -b --no-pager -n 300
ls -l ~/.ros/log/latest
grep -RinE "error|warn|fault|shutdown|battery|voltage|current|estop|puma|motor" ~/.ros/log/latest
dmesg -T | grep -Ei "can|usb|voltage|current|puma|motor|mcu"
```

## RViz

```bash
cd ~/ridgeback_noetic_docker
./scripts/rviz
```

## If Topics Do Not Appear

Confirm these on the host:

```bash
ip addr show enp132s0
ping 192.168.131.1
```

Confirm these on the robot:

```bash
echo "$ROS_MASTER_URI"
echo "$ROS_IP"
hostname -I
rosnode list
```

For ROS 1, both machines must be able to reach the addresses that nodes advertise. This setup assumes the robot is `192.168.131.1` and this workstation is `192.168.131.100`.
