# Ridgeback Noetic Docker

A ROS Noetic container for working with a Clearpath Ridgeback from a newer Ubuntu machine (tested on 24.04). It covers driving, mapping, localization, slow navigation, diagnostics, and a Gazebo simulator.

Every script in `scripts/` runs inside the container, so nothing ROS-related has to be installed on the host.

## Setup

Install Docker:

```bash
sudo apt update
sudo apt install docker.io docker-compose-v2
sudo usermod -aG docker "$USER"
```

Log out and back in so the group change applies.

Clone with the submodule:

```bash
git clone --recursive https://github.com/obadahmh/ridgeback_noetic_docker.git
cd ridgeback_noetic_docker
```

Set the network addresses:

```bash
cp .env.example .env
```

| Variable | Default | Meaning |
|---|---|---|
| `ROBOT_IP` | `192.168.131.1` | Robot address. The ROS master runs here. |
| `OFFBOARD_IP` | `192.168.131.100` | This computer's address on the robot network. |
| `ROBOT_HOSTNAME` | `cpr-r100-0601` | Robot hostname, mapped to `ROBOT_IP` in the container. |

Or use the helpers:

```bash
./scripts/use-ethernet
./scripts/use-wireless ROBOT_WIFI_IP OFFBOARD_WIFI_IP
```

Build the image and the workspace:

```bash
./scripts/build
./scripts/build-ws
```

The container uses host networking. ROS 1 nodes advertise their own address, so both machines must be able to reach each other at the IPs above.

## Check the connection

```bash
./scripts/test-robot
```

This should list the robot's topics. For a shell inside the container, run `./scripts/shell`.

## Drive

Make sure the area is clear, or the wheels are off the ground.

```bash
./scripts/teleop
```

### Text commands

`text-control` takes short, fixed commands and checks them against hard limits. It uses odometry to end each move and sends zero velocity before and after.

```bash
./scripts/text-control --dry-run "forward 0.5m"   # check only, nothing sent
./scripts/text-control "forward 0.25m"            # one command
./scripts/text-control                            # interactive
```

Commands: `status`, `stop`, `forward 0.5m`, `back 20cm`, `turn left 45deg`, `turn right 90deg`, `quit`.

Limits: 1.0 m per move, 90° per turn, 0.12 m/s, 0.35 rad/s.

It also takes JSON:

```bash
./scripts/text-control --dry-run --json '{"action":"move","direction":"forward","distance_m":0.25}'
./scripts/text-control --dry-run --json '{"action":"turn","direction":"left","angle_deg":30}'
```

Allowed actions are `move`, `turn`, `stop`, and `status`. Any other field is rejected.

To stop: `Ctrl-C`, the run-stop, or the E-stop.

### LLM commands

`llm-control` sends a plain-English request to an OpenAI-compatible endpoint (set up for K2-Think) and gets back one JSON command. That command goes through the same checks as `text-control`. The LLM never publishes to `/cmd_vel` directly.

```bash
export K2_THINK_BASE_URL=http://localhost:8000/v1
export K2_THINK_MODEL=LLM360/K2-Think-V2
export K2_THINK_API_KEY=your_key_if_needed

./scripts/llm-control --dry-run "move forward a quarter meter"
./scripts/llm-control "move forward a quarter meter"
```

You can keep these exports in `.llm.env`. Git ignores that file.

## Simulator

This uses Clearpath's ROS 1 Ridgeback Gazebo simulator. It runs a local ROS master and ignores `.env`.

```bash
./scripts/sim-ridgeback          # terminal 1: Gazebo (default config: base_sick)
./scripts/sim-rviz               # terminal 2: RViz
./scripts/sim-teleop             # terminal 3: keyboard driving
```

SLAM in simulation, with Gazebo still running:

```bash
./scripts/sim-slam
./scripts/sim-save-map sim_lab
```

If Gazebo or RViz fails to render, put `LIBGL_ALWAYS_SOFTWARE=1` in front of the command.

## Mapping

The order is: check sensors, run SLAM while driving slowly, save the map, then localize on it.

Check that laser, odometry, and TF are there:

```bash
./scripts/check-nav-prereqs
./scripts/slam-smoke-test       # starts gmapping without driving
./scripts/rviz-laser            # should show red laser points
```

If `rviz-laser` is empty, fix the laser or display first. SLAM won't work without it.

Run SLAM:

```bash
./scripts/stop-navigation       # clear any old nodes
./scripts/slam                  # terminal 1: gmapping + RViz
./scripts/teleop                # terminal 2: drive slowly
```

Drive slow loops. Avoid fast spins, wheel slip, and long corridors with no loop back. Save before stopping `slam`:

```bash
./scripts/save-map first_floor
```

This writes `ws/maps/first_floor.yaml` and `ws/maps/first_floor.pgm`.

Other mapping tools:

| Script | Does |
|---|---|
| `clearpath-gmapping` + `clearpath-rviz-gmapping` | Clearpath's gmapping without `move_base` |
| `clearpath-gmapping-demo` | Clearpath's full tutorial launch, including `move_base` |
| `mapping` + `rviz-mapping` | gmapping and RViz in separate terminals |
| `map-stats` | Shows the cell counts in `/map` |
| `view-map maps/X.yaml` | Opens a saved map in RViz |
| `open-map-image maps/X.yaml` | Opens the raw map image |

`ridgeback_navigation` is built from source (`ws/src/ridgeback`) because apt doesn't have it.

## Localization

```bash
./scripts/stop-navigation
./scripts/localize maps/lab_floor.yaml   # terminal 1: AMCL only
./scripts/rviz-localization              # terminal 2
./scripts/teleop                         # terminal 3
```

In RViz, use **2D Pose Estimate** to set where the robot is and which way it faces. If the laser stays lined up with the map while you drive, localization works.

`localize-demo` runs Clearpath's full demo, which also starts `move_base`.

## Navigation

Only do this once localization is reliable. `navigate-slow` uses Clearpath's costmaps with a slower planner, lower speed limits, and no recovery spins.

```bash
./scripts/stop-navigation
./scripts/navigate-slow maps/lab_floor.yaml   # terminal 1
./scripts/rviz-localization                   # terminal 2
./scripts/cancel-nav                          # terminal 3: ready to abort
```

In RViz, set the pose first, then use **2D Nav Goal** to pick a spot 0.5 to 1 m away in open space. Stay near the E-stop.

To abort: `cancel-nav`, `stop-navigation`, `Ctrl-C`, or the E-stop.

## Diagnostics

Use these if the robot faults or shuts down, for example when it can rotate but not translate. Common causes are battery sag, motor controller faults, overload, or E-stop changes.

| Script | Does |
|---|---|
| `diagnose` | One-time snapshot |
| `record-diagnostics 60` | Records a bag for 60 s. Reproduce the fault during it. Saved to `ws/logs/` |
| `monitor-diagnostics` | Live diagnostics view |
| `watch-power` | Live battery and current view |

Topics worth watching: `/diagnostics_agg`, `/diagnostics_toplevel_state`, `/mcu/status`, `/status`, `/feedback`, `/e_stop`.

On the robot:

```bash
ssh robot@192.168.131.1
journalctl -b --no-pager -n 300
grep -RinE "error|fault|battery|voltage|current|estop|motor" ~/.ros/log/latest
dmesg -T | grep -Ei "can|usb|voltage|motor|mcu"
```

## Troubleshooting

No topics? Check both sides.

On this computer:

```bash
ip addr          # is 192.168.131.100 assigned?
ping 192.168.131.1
```

On the robot:

```bash
echo "$ROS_MASTER_URI" "$ROS_IP"
hostname -I
rosnode list
```

## Notes

- The container user and paths are hardcoded to `obadah` (`compose.yaml`, scripts). Change them if your username is different.
- `ws/build`, `ws/devel`, `ws/maps`, and `ws/logs` are not tracked.
