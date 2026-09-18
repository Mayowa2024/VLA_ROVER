# Rover VLA ROS 2 scaffold

This workspace supports software-only testing, real wheel-encoder odometry,
raw VLA episode recording and an opt-in path to the rover's discovered UDP
teleoperation and serial motor interfaces.

For the currently connected Jetson, camera, Arduino and G29, go directly to
[Quick recording guide](#quick-recording-guide-current-tested-hardware).

## What works now

- Publishes a task instruction on `/vla/instruction`.
- Applies velocity limits and a command watchdog.
- Produces real `/odom` and `/vla/state` from the two Arduino wheel encoders.
- Keeps command-integrated fake odometry as an explicit testing fallback.
- Converts `/cmd_vel` into mock left/right wheel commands.
- Converts UDP teleop JSON into `/teleop/cmd_vel`.
- Provides a separate, opt-in `/cmd_vel` to serial motor driver.
- Records one episode as:
  - JPEG camera frames
  - instruction
  - current state
  - expert action
  - timestamp, episode index and frame index
- Supports `previous_action` as a temporary software-test state source.

## What is left

1. Final fine-tuning and evaluation
2. Recording 20 more episodes (currently have 30 recorded)


## Install dependencies

Source the installed ROS distribution first so `$ROS_DISTRO` expands to
`humble` in the package names:

```bash
source /opt/ros/humble/setup.bash
```

```bash
sudo apt update
sudo apt install \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-serial \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-tf2-ros \
  ros-$ROS_DISTRO-usb-cam \
  ros-$ROS_DISTRO-teleop-twist-keyboard
```

Initialize rosdep if this machine has not used it before:

```bash
sudo rosdep init || true
rosdep update
```

## Build

```bash
cd VLA_ROVER
./scripts/build_workspace.sh
source rover_vla_ws/install/setup.bash
```

The equivalent explicit ROS 2 build and test commands are:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd rover_vla_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select rover_vla_core
source install/setup.bash
colcon test --packages-select rover_vla_core --event-handlers console_direct+
colcon test-result --verbose
```

## Run the software-only stack

Fake action inputs are generated

```bash
./scripts/run_software_test.sh "Drive to the blue cone"
```

This launches:

```text
/vla/raw_cmd_vel
        ↓
safety_filter
        ↓
/cmd_vel
   ├── fake_odom → /odom and /vla/state
   └── mock_motor_driver → /rover/mock_wheel_command
```

This path never opens a serial device and cannot actuate the rover.

Inspect it:

```bash
ros2 topic echo /rover/mock_wheel_command
ros2 topic echo /odom
```

In another terminal:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source rover_vla_ws/install/setup.bash
./scripts/publish_test_actions.sh
```

## Test keyboard teleoperation in software

Remap keyboard teleoperation into the safety filter:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/vla/raw_cmd_vel
```

You should see mock wheel commands and fake odometry change.

## Test the episode recorder without rover hardware

The recorder needs an image topic and an action topic. Your USB-camera node can
provide the image topic. Use keyboard teleoperation as the action topic.

Recommended split:

```text
teleop_twist_keyboard → /teleop/cmd_vel
                           ├── frame_recorder (expert action)
                           └── remap/relay to safety stack later
```

For a no-movement home test, publish sample actions directly:

```bash
ros2 topic pub -r 5 /teleop/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.10}}"
```

Record an episode using the raw topic from `usb_cam`:

```bash
./scripts/record_episode.sh \
  0 \
  "Drive to the blue cone" \
  /image_raw \
  /teleop/cmd_vel \
  /odom \
  previous_action
```

Stop with `Ctrl+C`.

The data is written to:

```text
~/rover_vla_data/raw/episode_0000/
├── metadata.json
├── frames.jsonl
└── images/
```

Inspect it:

```bash
python3 tools/inspect_episode.py \
  ~/rover_vla_data/raw/episode_0000
```

## Use the real wheel encoders

The Arduino Nano must send a newline-terminated pair approximately every
100 ms:

```text
rear_left_ticks,front_right_ticks
```

The first valid pair establishes a baseline. Later pairs produce measured
linear and angular velocity plus an integrated differential-drive pose. The
node publishes:

```text
/odom       nav_msgs/msg/Odometry
/vla/state  std_msgs/msg/Float32MultiArray  [linear_velocity, angular_velocity]
```

Defaults are 115200 baud, 1224 counts per revolution, 0.15875 m wheel
diameter, 0.352425 m track width and direction multipliers of `1.0`. Counts
are expected to increase for forward wheel rotation. Override
`encoder_left_direction` or `encoder_right_direction` with `-1.0` only if a
wheel is reversed in practice.

Before launching, close the Arduino Serial Monitor. Find the stable device
name with:

```bash
ls -l /dev/serial/by-id/
```

The default is `/dev/ttyUSB0`; some Nano interfaces appear as
`/dev/ttyACM0`. A `/dev/serial/by-id/...` path is preferable when available.
If access is denied, add your user to the serial-device group, then log out
and back in:

```bash
sudo usermod -aG dialout "$USER"
```

For the current terminal only, you can instead run `newgrp dialout` by itself.
It opens a new shell. At the new prompt, type `id` and confirm that
`gid=20(dialout)` or `20(dialout)` appears before starting ROS. Group changes
do not affect a hardware launch that is already running; restart that launch
from the corrected shell.

These commands are intentionally not run by the repository scripts.

Launch measured encoder odometry with motor output disabled:

```bash
ros2 launch rover_vla_core hardware_teleop.launch.py \
  enable_motors:=false \
  use_fake_odom:=false \
  encoder_serial_port:=/dev/serial/by-id/usb-Arduino_LLC_Arduino_Nano_Every_B1DB339E51544E5450202020FF024738-if00 \
  encoder_baud_rate:=115200 \
  publish_tf:=false
```

Use `/dev/ttyACM0` only as a temporary fallback if no stable by-id path exists.
Never give the encoder node and motor driver the same serial device.

Use fake odometry instead:

```bash
ros2 launch rover_vla_core hardware_teleop.launch.py \
  enable_motors:=false \
  use_fake_odom:=true \
  publish_tf:=false
```

The launch conditions are mutually exclusive, so only one node publishes
`/odom`. Confirm that and inspect the measured output with:

```bash
ros2 topic echo /odom
ros2 topic echo /vla/state
ros2 topic hz /odom
ros2 topic info /odom --verbose
```

With the current Arduino sketch, `/odom` should normally be close to 10 Hz
because the Nano reports every 100 ms. If valid measurements stop for 0.5 s,
the node publishes zero velocity once and uses the next valid pair as a new
baseline. Malformed startup text is ignored, unreasonable count jumps are
rebased, and a disconnected serial port is reopened periodically.

Record measured state with:

```bash
./scripts/record_episode.sh \
  1 \
  "Drive to the red cone" \
  /image_raw \
  /teleop/cmd_vel \
  /odom \
  odom
```

Do not mix `previous_action` and `odom` state definitions in the final training
dataset. The `previous_action` mode is for software and pipeline testing. The
recorder pauses frame capture when `/odom` becomes stale rather than silently
reusing an old measured state.

## Use the rover's UDP teleoperation

The discovered controller sends JSON Twist-like packets to UDP port `6001`:

```json
{
  "linear": {"x": 0.05},
  "angular": {"z": 0.10}
}
```

`teleop_adapter.py` validates these packets and publishes:

```text
/teleop/cmd_vel
geometry_msgs/msg/Twist

linear.x  = forward command
angular.z = turning command
```

Start the hardware graph with motor output disabled:

```bash
ros2 launch rover_vla_core hardware_teleop.launch.py \
  enable_motors:=false \
  use_fake_odom:=false \
  encoder_serial_port:=/dev/serial/by-id/usb-Arduino_LLC_Arduino_Nano_Every_B1DB339E51544E5450202020FF024738-if00 \
  allowed_source_ip:=10.147.17.207
```

If the network changes, replace the tested controller IP with its address on
the same network used to reach the Jetson. Inspect both sides of the safety
filter:

```bash
ros2 topic echo /teleop/cmd_vel
ros2 topic echo /cmd_vel
```

The data and actuation paths are deliberately separated:

```text
UDP controller
      ↓
teleop_adapter → /teleop/cmd_vel ──→ frame_recorder action label
                         ↓
                   safety_filter
                         ↓
                     /cmd_vel
                         ↓
               serial_motor_driver
```

The adapter publishes one zero action when UDP input times out. The safety
filter has its own watchdog, and the serial driver independently writes zero
when `/cmd_vel` becomes stale.

### Send Logitech G29 controls from another Linux PC

The controller PC does not need ROS. Copy this standalone Python file to it:

```text
rover_vla_ws/src/rover_vla_core/rover_vla_core/g29_udp_sender.py
```

It uses only the Python standard library and the Linux joystick device. First
confirm the device and map each control:

```bash
ls -l /dev/input/js*
python3 g29_udp_sender.py monitor --device /dev/input/js0
```

Move the steering wheel, accelerator, brake and a suitable deadman button one
at a time. The tested G29 currently reports steering axis `0`, clutch axis
`1`, accelerator axis `2`, brake axis `3` and the right paddle as button `4`.
Verify these numbers after changing the controller PC, wheel mode or kernel.

The sender can target either the Jetson's local-network address or its
ZeroTier address. Start the motor-disabled Jetson graph with the
[quick recording guide](#quick-recording-guide-current-tested-hardware), then
run the sender on the controller PC. This example targets the Jetson's
ZeroTier address:

```bash
python3 g29_udp_sender.py send 10.147.17.41 \
  --steering-axis 0 \
  --accelerator-axis 2 \
  --brake-axis 3 \
  --steering-deadzone 0.06 \
  --deadman-button 4
```

It sends at 30 Hz and scales directly to the current safety limits of
`0.10 m/s` and `0.40 rad/s`. The accelerator requests forward motion and the
brake reduces it; the brake never requests reverse. The terminal prints the
actual source IP selected by Linux—this must equal `allowed_source_ip`.

If a pedal reads `1.00` while released and `0.00` while pressed, add its
corresponding `--accelerator-invert` or `--brake-invert` option. If steering
left produces a negative `angular.z`, set `--steering-sign 1`.

For the first motor-disabled packet test only, `--no-deadman` can replace
`--deadman-button`. Before any motor test, select a real deadman button. It
must be observed released once after sender startup before pressing it can arm
commands. Releasing it, stopping the program or losing the joystick produces
zero commands; the Jetson's independent watchdogs remain active.

On the Jetson, verify the received and filtered values:

```bash
ros2 topic echo /teleop/cmd_vel
ros2 topic echo /cmd_vel
```

Enable motor output only during the controlled
[lifted-wheel motor test](#tomorrow-lifted-wheel-motor-test). Never use the
encoder Arduino path as the motor-controller `serial_port`.

The motor controller currently receives the movement format found in the
existing script:

```text
000000000000000r<left>l<right>\n
```

The copied script contained conflicting stop formats, so all movement and stop
commands now use this one command format. Confirm it against the controller
firmware before driving on the ground. A physical emergency stop and a
controller-side watchdog are still required.

## Quick recording guide: current tested hardware

Use this section for normal data collection with the hardware tested on
27 July 2026. The commands below keep motor output disabled. They can test the
camera, G29 link, encoder measurements and recorder without driving the rover.

### Tested values

| Item | Tested value |
| --- | --- |
| Jetson ZeroTier address | `10.147.17.41` |
| Controller PC ZeroTier address | `10.147.17.207` |
| UDP port | `6001` |
| Camera topic | `/image_raw` |
| Camera format | 640 × 480 at about 30 Hz |
| Encoder topic | `/odom` |
| Encoder rate | about 10 Hz |
| Recorder rate | 10 Hz |
| Steering, accelerator, brake | axes `0`, `2`, `3` |
| Deadman paddle | button `4` |
| Encoder serial device | `/dev/serial/by-id/usb-Arduino_LLC_Arduino_Nano_Every_B1DB339E51544E5450202020FF024738-if00` |

Four terminals are used:

1. Jetson hardware graph
2. Jetson camera
3. Controller PC G29 sender
4. Jetson preflight and recorder

### 1. Check Arduino access on the Jetson

Close miniterm and the Arduino Serial Monitor. Then run:

```bash
id
ls -l /dev/serial/by-id/
```

The `id` output must contain `dialout`. If it does not, run this command by
itself:

```bash
newgrp dialout
```

At the new prompt, type:

```bash
id
```

Confirm that `gid=20(dialout)` or `20(dialout)` appears. A ROS process that was
started before this change still has the old permissions and must be restarted.

### 2. Jetson terminal 1: start the hardware graph

```bash
cd ~/Downloads/rover_vla_software-20260727T110435Z-1-001/rover_vla_software/VLA_ROVER/rover_vla_ws
source /home/nano1/ros2_humble/install/setup.bash
source install/setup.bash

ros2 launch rover_vla_core hardware_teleop.launch.py \
  enable_motors:=false \
  use_fake_odom:=false \
  encoder_serial_port:=/dev/serial/by-id/usb-Arduino_LLC_Arduino_Nano_Every_B1DB339E51544E5450202020FF024738-if00 \
  encoder_baud_rate:=115200 \
  allowed_source_ip:=10.147.17.207 \
  publish_tf:=false
```

Wait for:

```text
Connected to encoder Arduino
```

Keep this terminal running. `enable_motors:=false` means G29 commands cannot
drive the motors.

### 3. Jetson terminal 2: start the camera

```bash
source /home/nano1/ros2_humble/install/setup.bash
ros2 run usb_cam usb_cam_node_exe
```

Keep this terminal running. The tested raw image topic is `/image_raw`.

### 4. Controller PC terminal: start the G29 sender

Connect the G29 to the controller PC and check it:

```bash
ls -l /dev/input/js0
```

Start with paddle button `4` released:

```bash
python3 ~/g29_udp_sender.py send 10.147.17.41 \
  --steering-axis 0 \
  --accelerator-axis 2 \
  --brake-axis 3 \
  --steering-deadzone 0.06 \
  --deadman-button 4
```

Hold paddle `4` to enable commands. Release it to command zero. UDP does not
have a persistent connection; if the Jetson hardware graph is restarted, a
sender that is still running resumes automatically. Restart the sender if it
has stopped.

### 5. Jetson terminal 4: check the streams

Open the fourth terminal. This same terminal can be reused for recording:

```bash
cd ~/Downloads/rover_vla_software-20260727T110435Z-1-001/rover_vla_software/VLA_ROVER/rover_vla_ws
source /home/nano1/ros2_humble/install/setup.bash
source install/setup.bash

ros2 topic echo /image_raw --field header --once
ros2 topic echo /teleop/cmd_vel --once
ros2 topic echo /odom --field twist.twist --once
ros2 topic info /odom --verbose
```

Each `echo` command must print a message. `/odom` must have exactly one
publisher.

To test the two encoders with motors disabled:

```bash
ros2 topic echo /odom
```

Rotate one wheel and then the other. With the current direction settings:

- Rotating the right wheel forward produces positive `angular.z`.
- Rotating the left wheel forward produces negative `angular.z`.
- Forward wheel rotation produces positive `linear.x`.

Press `Ctrl+C` after the test.

### 6. Jetson terminal 4: record an episode

First list the IDs already used:

```bash
ls -1 ~/rover_vla_data/raw
```

The recorder does not choose an ID automatically and will not overwrite an
existing directory. Choose the next unused number. This example uses `4`:

```bash
cd ~/Downloads/rover_vla_software-20260727T110435Z-1-001/rover_vla_software/VLA_ROVER
source /home/nano1/ros2_humble/install/setup.bash

./scripts/record_episode.sh \
  4 \
  "Drive to the blue cone" \
  /image_raw \
  /teleop/cmd_vel \
  /odom \
  odom
```

The six arguments are:

```text
episode ID
instruction
camera topic
expert-action topic
odometry topic
state source
```

The recorder waits until the instruction, camera, action and fresh odometry
are all available. It prints:

```text
Saved 20 frames
```

approximately every two seconds. If this never appears, see
[Simple troubleshooting](#simple-troubleshooting).

### 7. Stop and inspect the episode

For a controller test, release paddle `4` and wait about one second. Then press
`Ctrl+C` in the recorder terminal. Wait for the `Finalized episode` message.

Inspect the episode, replacing `4` with the ID used:

```bash
cd ~/Downloads/rover_vla_software-20260727T110435Z-1-001/rover_vla_software/VLA_ROVER

python3 tools/inspect_episode.py \
  ~/rover_vla_data/raw/episode_0004

cat ~/rover_vla_data/raw/episode_0004/metadata.json
```

Check that:

- Row count equals image count.
- `finalized` is `true`.
- `state_source` is `odom`.
- Images are clear and correctly oriented.
- Actions change when the G29 is used.
- State changes when the wheels move.

A motor-disabled controller test can contain changing actions and zero state.
That is a valid pipeline test but not training data. A hand-turned encoder test
can contain changing state and zero action. That is also a test, not a driving
demonstration. In a real training episode, action and measured motion should
change together.

### Tomorrow: lifted-wheel motor test

Do not test the motors on the ground first.

1. Securely support the rover so every driven wheel is clear of the ground.
2. Keep the physical emergency stop or main power switch within reach.
3. Use `ls -l /dev/serial/by-id/` to identify the motor controller.
4. Confirm that the motor controller and encoder Arduino have different
   serial paths.
5. Confirm that the motor-controller firmware stops on command timeout.
6. Start the G29 sender with paddle `4` released.
7. Start motor output only after replacing
   `REPLACE_WITH_MOTOR_CONTROLLER_ID` in the detailed launch command below.
8. Apply a very small accelerator input and confirm wheel directions and
   `/odom` signs.
9. Verify three separate stop tests: release paddle `4`, stop the G29 sender,
   and interrupt the network link. The actual wheels must stop each time.
10. If any stop test fails, use the emergency stop or remove power and do not
    place the rover on the ground.

### Simple troubleshooting

- **Only the two startup messages appear in the recorder:** one or more of
  `/image_raw`, `/teleop/cmd_vel`, `/odom` or `/vla/instruction` is missing.
  Run the one-message `ros2 topic echo` checks above.
- **Encoder says `Permission denied`:** stop the old hardware launch, enter a
  shell whose `id` output contains `dialout`, and launch it again.
- **`/odom` stays zero while a wheel is moving:** check the Arduino output,
  encoder wiring and tick direction.
- **No `/image_raw`:** confirm `/dev/video0` exists and that
  `usb_cam_node_exe` is still running.
- **UDP port 6001 is already in use:** stop the older `teleop_adapter` or
  hardware launch before starting another one.
- **Episode directory already exists:** choose the next unused ID.
- **`finalized` is false:** the recorder did not shut down cleanly; keep that
  episode out of the training set.
- **Recording rate is below 10 Hz:** check for camera pauses, high CPU load and
  slow storage. The recorder skips missed frames instead of duplicating them.

## Detailed recording reference

This is the end-to-end collection path:

```text
G29 on controller PC
        ↓ UDP at 30 Hz
/teleop/cmd_vel ───────────────────────────→ action [linear.x, angular.z]
        ↓
safety_filter → /cmd_vel → motor driver

camera image topic ────────────────────────→ JPEG image
encoder_odom /odom ────────────────────────→ state [linear.x, angular.z]
instruction_publisher /vla/instruction ───→ instruction
```

The recorder writes one raw episode at a time. Each selected camera frame is
stored with the latest instruction, expert action and measured encoder state.
It does not perform exact timestamp synchronization between those topics.

### Safety and data-quality distinction

Use `enable_motors:=false` for the first recording test. That verifies the
files, topics and controller link without actuating the rover. Do not use that
stationary test episode for training because its expert action does not cause
the measured motion.

A real driving episode requires all of the following first:

- The motor protocol, channel order, signs and scale have been verified with
  the wheels lifted.
- The motor controller and encoder Arduino have distinct serial devices.
- Releasing G29 paddle button `4` has been verified to command zero.
- With the rover wheels lifted, releasing the paddle, terminating the G29
  sender and interrupting UDP input have each been verified to stop the actual
  wheels.
- The motor-controller firmware has its own command-timeout watchdog, so it
  stops the wheels even if the Jetson or serial link fails after a nonzero
  command.
- The rover has a physical emergency stop and enough clear space.
- The encoder dimensions, direction signs and counts per revolution have been
  checked against measured motion.

Keep one instruction and one attempt per episode. Reset the rover and scene to
a known starting condition before each new demonstration.

### Tested network and device values

The current tested ZeroTier path is:

```text
Controller PC source: 10.147.17.207
Jetson destination:    10.147.17.41
UDP destination port:  6001
```

The current stable encoder path is:

```text
/dev/serial/by-id/usb-Arduino_LLC_Arduino_Nano_Every_B1DB339E51544E5450202020FF024738-if00
```

If the hardware or network changes, find the new values instead of assuming
these remain valid:

```bash
ip -br -4 addr
ls -l /dev/serial/by-id/
ls -l /dev/input/js*
ls -l /dev/video*
```

### Terminal 1 on the Jetson: start the hardware graph

Stop any standalone `teleop_adapter` first. Only one process can bind UDP port
`6001`, and only one process can open the encoder serial port.

Open a new Jetson terminal:

```bash
cd ~/Downloads/rover_vla_software-20260727T110435Z-1-001/rover_vla_software/VLA_ROVER/rover_vla_ws
source /home/nano1/ros2_humble/install/setup.bash
source install/setup.bash

ros2 launch rover_vla_core hardware_teleop.launch.py \
  enable_motors:=false \
  use_fake_odom:=false \
  encoder_serial_port:=/dev/serial/by-id/usb-Arduino_LLC_Arduino_Nano_Every_B1DB339E51544E5450202020FF024738-if00 \
  encoder_baud_rate:=115200 \
  allowed_source_ip:=10.147.17.207 \
  publish_tf:=false
```

This starts:

- `teleop_adapter`: UDP to `/teleop/cmd_vel`
- `safety_filter`: `/teleop/cmd_vel` to `/cmd_vel`
- `encoder_odom`: Arduino counts to `/odom` and `/vla/state`

It does not start the camera, instruction publisher or frame recorder. With
`enable_motors:=false`, it also does not start `serial_motor_driver`.

For a real driven collection only after completing every motor validation
listed above, stop the motor-disabled graph and relaunch it with a separately
identified motor-controller device:

```bash
ros2 launch rover_vla_core hardware_teleop.launch.py \
  enable_motors:=true \
  use_fake_odom:=false \
  serial_port:=/dev/serial/by-id/REPLACE_WITH_MOTOR_CONTROLLER_ID \
  baud_rate:=9600 \
  encoder_serial_port:=/dev/serial/by-id/usb-Arduino_LLC_Arduino_Nano_Every_B1DB339E51544E5450202020FF024738-if00 \
  encoder_baud_rate:=115200 \
  allowed_source_ip:=10.147.17.207 \
  publish_tf:=false
```

Do not run this command while the motor-controller placeholder is unresolved.
The currently connected encoder Nano already owns its own serial device.

### Terminal 2 on the Jetson: start the camera

Connect the camera and confirm its device appears. For a USB camera:

```bash
ls -l /dev/video*
```

The installed `usb_cam` node was tested with `/dev/video0`, 640 by 480 and
about 30 Hz. Start it directly:

```bash
source /home/nano1/ros2_humble/install/setup.bash
ros2 run usb_cam usb_cam_node_exe
```

Find the exact raw image topic rather than assuming its name:

```bash
source /home/nano1/ros2_humble/install/setup.bash
ros2 topic list -t | grep 'sensor_msgs/msg/Image'
```

With the tested direct command, the topic is:

```text
/image_raw
```

If the camera does not support the default format or resolution, configure
`usb_cam` for the actual device before collecting data. A CSI or RealSense
camera requires its own ROS driver; pass whichever raw
`sensor_msgs/msg/Image` topic that driver publishes to the recorder.

### Terminal 1 on the controller PC: start the G29 sender

If the sender is not already on the controller PC, copy it from the Jetson
over the tested ZeroTier route:

```bash
scp nano1@10.147.17.41:/home/nano1/Downloads/rover_vla_software-20260727T110435Z-1-001/rover_vla_software/VLA_ROVER/rover_vla_ws/src/rover_vla_core/rover_vla_core/g29_udp_sender.py ~/
```

Confirm that the wheel is still available:

```bash
ls -l /dev/input/js0
```

Start with paddle button `4` released:

```bash
python3 ~/g29_udp_sender.py send 10.147.17.41 \
  --steering-axis 0 \
  --accelerator-axis 2 \
  --brake-axis 3 \
  --steering-deadzone 0.06 \
  --deadman-button 4
```

The status must show `deadman=off` while the paddle is released. Hold the
paddle to enable commands. Releasing it must immediately return the command to
zero. Never use `--no-deadman` for a driven recording.

### Terminal 3 on the Jetson: perform the preflight

Source ROS and the workspace:

```bash
cd ~/Downloads/rover_vla_software-20260727T110435Z-1-001/rover_vla_software/VLA_ROVER/rover_vla_ws
source /home/nano1/ros2_humble/install/setup.bash
source install/setup.bash
```

Check the topic types:

```bash
ros2 topic type /image_raw
ros2 topic type /teleop/cmd_vel
ros2 topic type /cmd_vel
ros2 topic type /odom
```

Expected results:

```text
sensor_msgs/msg/Image
geometry_msgs/msg/Twist
geometry_msgs/msg/Twist
nav_msgs/msg/Odometry
```

Check each rate for several seconds and stop each command with `Ctrl+C`:

```bash
ros2 topic hz /image_raw
ros2 topic hz /teleop/cmd_vel
ros2 topic hz /cmd_vel
ros2 topic hz /odom
```

Expected approximate rates are:

```text
camera:              30 Hz
G29 expert action:   30 Hz
filtered command:    20 Hz
encoder odometry:    10 Hz
recorder target:     10 Hz
```

Confirm that exactly one node publishes `/odom`:

```bash
ros2 topic info /odom --verbose
```

Inspect one message from each data stream:

```bash
ros2 topic echo /image_raw --field header --once
ros2 topic echo /teleop/cmd_vel --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /odom --field twist.twist --once
```

With the current G29 scaling and safety limits, `/teleop/cmd_vel` and
`/cmd_vel` should contain the same active command. If the safety filter clips
them differently, decide deliberately whether the training label should
remain the requested expert action or become the executed filtered action
before collecting the final dataset.

Do not begin an episode if any required topic is absent, the encoder port is
reconnecting, or the controller link is dropping packets.

### Terminal 4 on the Jetson: record one episode at 10 Hz

List existing episodes before choosing an ID:

```bash
ls -1 ~/rover_vla_data/raw 2>/dev/null || true
df -h ~
```

Episode IDs are explicit and do not auto-increment. ID `0` creates
`episode_0000`, ID `1` creates `episode_0001`, and so on. The recorder refuses
to append to or overwrite an existing episode directory.

From the repository root:

```bash
cd ~/Downloads/rover_vla_software-20260727T110435Z-1-001/rover_vla_software/VLA_ROVER
source /home/nano1/ros2_humble/install/setup.bash

./scripts/record_episode.sh \
  4 \
  "Drive to the blue cone" \
  /image_raw \
  /teleop/cmd_vel \
  /odom \
  odom
```

Arguments, in order, are:

```text
episode ID
instruction
image topic
expert-action topic
odometry topic
state source
```

Use `odom` for real encoder state. `previous_action` is only a software-test
fallback and must not be mixed into the final encoder dataset.

The script starts `instruction_publisher` automatically. The recorder waits
until it has:

- A nonempty instruction
- At least one finite expert action
- A finite `/odom` sample no older than one second
- A camera image allowed by the recording rate

At the default 10 Hz limit, the recorder prints `Saved 20 frames` after every
20 successfully written rows. That is approximately every two seconds under
ideal load but can be slower because saving is driven by camera callbacks. If
that message never appears, stop the attempt and check all required topics.

### Perform and stop the demonstration

1. Begin with the rover stationary and paddle button `4` released.
2. Wait until the recorder is saving frames.
3. Hold paddle `4` and perform the instruction using the G29.
4. Complete the task and release paddle button `4` to command an immediate
   stop, then release the pedals.
5. Verify the actual wheels are stopped. If they are not, use the physical
   emergency stop.
6. Wait about one second so the episode contains the final zero action.
7. Press `Ctrl+C` in the recorder terminal.
8. Stop the G29 sender, camera and hardware graph only after the recorder has
   logged that the episode was finalized.

Clean shutdown flushes `frames.jsonl` and updates `metadata.json` with
`"finalized": true`. A power loss or forced kill can leave a partial episode
with `"finalized": false`.

### Inspect and accept the episode

For example, episode `4` is written as:

```text
~/rover_vla_data/raw/episode_0004/
├── metadata.json
├── frames.jsonl
└── images/
    ├── frame_000000.jpg
    ├── frame_000001.jpg
    └── ...
```

Run the integrity checker:

```bash
cd ~/Downloads/rover_vla_software-20260727T110435Z-1-001/rover_vla_software/VLA_ROVER

python3 tools/inspect_episode.py \
  ~/rover_vla_data/raw/episode_0004

cat ~/rover_vla_data/raw/episode_0004/metadata.json

head -n 1 ~/rover_vla_data/raw/episode_0004/frames.jsonl | \
  python3 -m json.tool
```

Verify:

- Row count equals JPEG count.
- The instruction is correct.
- `state_source` is `odom`.
- Action ranges contain the controls used in the demonstration.
- The duration and frame count are plausible.
- `metadata.json` reports `"finalized": true`.
- Sample images are clear, correctly oriented and show the required scene.
- The controller, camera or encoder did not disconnect during the attempt.

Keep failed or interrupted attempts out of the training set. Do not reuse an
existing episode ID; move a rejected directory out of `raw` or choose a new
ID for the next attempt.

### Recording rate and synchronization

The camera and G29 sender run at about 30 Hz, while the Arduino reports encoder
counts at about 10 Hz. The recorder uses a fixed 10 Hz deadline and therefore
selects roughly one out of every three 30 Hz camera frames. A camera pause,
heavy CPU load or slow storage can lower the observed rate. Missed frames are
skipped rather than duplicated. Every saved frame still contains an
instruction, expert action and encoder state.

Ten hertz is the recommended initial rate because it matches the measured
encoder update rate and uses less storage. The recorder stores the latest
action and odometry available when each selected camera frame arrives; it does
not interpolate or synchronize messages by timestamp.

To record up to 30 Hz instead, start the instruction publisher in one terminal:

```bash
cd ~/Downloads/rover_vla_software-20260727T110435Z-1-001/rover_vla_software/VLA_ROVER
source /home/nano1/ros2_humble/install/setup.bash
source rover_vla_ws/install/setup.bash

ros2 run rover_vla_core instruction_publisher --ros-args \
  -p instruction:="Drive to the blue cone"
```

Then start the recorder directly in another terminal:

```bash
cd ~/Downloads/rover_vla_software-20260727T110435Z-1-001/rover_vla_software/VLA_ROVER
source /home/nano1/ros2_humble/install/setup.bash
source rover_vla_ws/install/setup.bash

ros2 run rover_vla_core frame_recorder --ros-args \
  -p episode_id:=5 \
  -p image_topic:=/image_raw \
  -p action_topic:=/teleop/cmd_vel \
  -p odom_topic:=/odom \
  -p state_source:=odom \
  -p sample_rate_hz:=30.0 \
  -p output_root:="$HOME/rover_vla_data/raw"
```

At 30 Hz, consecutive frames can contain the same 10 Hz encoder measurement.
Stop both processes cleanly with `Ctrl+C`. Do not combine 10 Hz and 30 Hz
episodes in one training dataset without deliberately accounting for the
different sampling periods.

### Recording troubleshooting

- **`Episode directory already exists`**: choose the next unused episode ID.
- **No `Saved 20 frames` message**: check the instruction, image, action and
  odometry topics. The recorder deliberately waits when a required stream is
  missing.
- **No camera topic**: confirm `/dev/video*`, start the correct camera driver
  and pass its actual raw image topic.
- **`Address already in use` on UDP port 6001**: stop the standalone
  `teleop_adapter` before launching `hardware_teleop.launch.py`.
- **Encoder permission denied or serial busy**: close miniterm and the Arduino
  Serial Monitor, confirm membership in `dialout`, and ensure only
  `encoder_odom` owns the encoder port.
- **Low or irregular frame count**: check camera rate, encoder freshness,
  system load and available disk space.
- **Controller or network failure during recording**: release the deadman,
  stop the rover and reject the episode. The recorder has no independent
  action-freshness timestamp.
- **`finalized` is false**: treat the episode as incomplete unless it is
  carefully inspected and intentionally recovered.

## Data conversion to LeRobot

Keep this raw dataset first. LeRobot's dataset API evolves, so convert only
after pinning the LeRobot version on the training machine. The important raw
fields are already present:

```text
image
instruction
state[2]
action[2]
timestamp_sec
episode_index
frame_index
```

For LeRobot v3, a converter should create a `LeRobotDataset`, call
`add_frame()` for each row, `save_episode()` after each episode and
`finalize()` before uploading or training.

## Final hardware architecture

```text
Existing teleoperation
        ↓
/teleop/cmd_vel ───────────────→ dataset action label
        ↓
safety_filter
        ↓
/cmd_vel
        ↓
serial_motor_driver
        ↓
motor controller

camera image topic ─────────────→ dataset image
wheel encoders → encoder_odom
                    ├── /odom ──→ dataset state
                    └── /vla/state
instruction → /vla/instruction → dataset task
```
