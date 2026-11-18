# FOSSBot Control Interface

ROS 2 (Jazzy) interface for driving the [FOSSBot](https://github.com/eellak/fossbot) mobile robot —
by clicking in a camera window, by hand gestures, with a live dashboard of the robot's state.
Works the same in the Gazebo simulation and on the physical robot, because both expose the
same topics (`/cmd_vel`, `/odom`, `/scan`, `/camera/image_raw`).

Project for the *Introduction to Robotics* course at Poznań University of Technology.

## Features

- [x] **Click teleoperation** – click above the centre line to drive forward, below to drive
      backward; speed grows with the distance from the centre
- [x] **Launch file + YAML parameters** – one command starts all nodes with their settings
- [ ] **Live dashboard** – velocity, trajectory and camera image in real time
- [ ] **Gesture control** – two hand gestures (MediaPipe) trigger 360° rotations
- [ ] **Docker Compose** – driver container (sim *or* real robot + camera) and an interface container

## Architecture

```mermaid
flowchart LR
    cam[usb_cam<br/>laptop camera] -- /user_cam/image_raw --> click[click_teleop]
    click -- /cmd_vel --> robot[FOSSBot<br/>Gazebo sim or fossbot_bridge]
    click -. /fossbot/enable_motors .-> robot
```

## Requirements

- Ubuntu 24.04 + ROS 2 Jazzy (or the FOSSBotEduSim container)
- Python packages: `opencv`, `numpy` (installed with ROS desktop)
- Optional: [`usb_cam`](https://github.com/ros-drivers/usb_cam) for the laptop camera
  (`sudo apt install ros-jazzy-usb-cam`)

## Build

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/wiktor77887-bit/fossbot-control-interface.git
cd ~/ros2_ws
colcon build --symlink-install --packages-select fossbot_control_interface
source install/setup.bash
```

## Run

### In simulation

```bash
# terminal 1 – simulator (inside FOSSBotEduSim)
ros2 launch fossbot_educational_description single.launch.py world:=simple_shapes.sdf

# terminal 2 – interface (+ laptop camera)
ros2 launch fossbot_control_interface interface.launch.py start_camera:=true
```

### On the real robot

Start the [fossbot-ros2-bridge](https://github.com/LRMPUT/fossbot-ros2-bridge) as described in the lab
instructions, then in every terminal:

```bash
export ROS_DOMAIN_ID=1
ros2 launch fossbot_control_interface interface.launch.py use_real_robot:=true start_camera:=true
```

`use_real_robot:=true` calls `/fossbot/enable_motors` automatically. The node publishes
`/cmd_vel` at 20 Hz while moving, because the robot stops after 0.5 s without commands.

### Controls (click window)

| Action | Effect |
|---|---|
| Left click above the line | Drive forward for `move_duration` seconds |
| Left click below the line | Drive backward for `move_duration` seconds |
| `SPACE` / `s` | Stop |
| `q` / `ESC` / close window | Quit (sends a zero `Twist`) |

## Parameters

Defined in [`config/params.yaml`](fossbot_control_interface/config/params.yaml).

| Parameter | Default | Description |
|---|---|---|
| `image_topic` | `/user_cam/image_raw` | Image shown in the window |
| `cmd_vel_topic` | `/cmd_vel` | Velocity command output |
| `max_linear_speed` | `0.2` | Speed [m/s] at the window edge |
| `min_linear_speed` | `0.05` | Speed [m/s] just outside the dead zone |
| `dead_zone` | `0.05` | Ignored band around the centre (fraction of half-height) |
| `proportional_speed` | `true` | `false` = always `max_linear_speed` |
| `move_duration` | `1.0` | How long one click drives the robot [s] |
| `publish_rate` | `20.0` | `/cmd_vel` rate while moving [Hz] |
| `enable_motors_on_start` | `false` | Set by `use_real_robot:=true` |

## Tests

```bash
cd ~/ros2_ws
colcon test --packages-select fossbot_control_interface --event-handlers console_direct+
```

## Project structure

```
fossbot_control_interface/
├── config/params.yaml               # node parameters
├── launch/interface.launch.py       # starts all nodes
├── fossbot_control_interface/
│   ├── click_teleop.py              # ROS 2 node: click window -> /cmd_vel
│   ├── click_logic.py               # pure click -> velocity mapping (unit tested)
│   └── image_utils.py               # sensor_msgs/Image -> OpenCV without cv_bridge
└── test/                            # pytest unit tests
```

## License

MIT – see [LICENSE](LICENSE).
