# Lam Research Challenge 2025 - ROS 2 Integration

This directory contains the ROS 2 nodes and message definitions for the LRC 2025 competition arena devices.

## Structure

```
ROS2/
├── lrc_arena_nodes/          # Python nodes for arena devices
│   ├── lrc_arena_nodes/
│   │   ├── __init__.py
│   │   ├── pump_controller.py      # Peristaltic pump control
│   │   ├── led_controller.py       # LAM LED array control
│   │   └── loadcell_lcd_controller.py  # Load cell + LCD
│   ├── launch/
│   │   └── arena_devices.launch.py
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
└── lrc_arena_msgs/           # Custom message definitions
    ├── msg/
    │   └── IRArray.msg
    ├── CMakeLists.txt
    └── package.xml
```

## Prerequisites

### System Requirements
- Ubuntu 22.04 (Jammy) or 20.04 (Focal)
- ROS 2 Humble (recommended) or Foxy
- Python 3.8+

### Install ROS 2

Follow the official installation guide: https://docs.ros.org/en/humble/Installation.html

Quick install (Ubuntu 22.04):
```bash
sudo apt update && sudo apt install -y ros-humble-desktop
source /opt/ros/humble/setup.bash
```

### Install Dependencies

```bash
# ROS 2 dependencies
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# Hardware libraries (for Raspberry Pi)
pip3 install RPi.GPIO hx711 rpi-ws281x RPLCD
```

## Build Instructions

1. **Create workspace** (if not already done):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. **Copy packages** to workspace:
```bash
# From this repository
cp -r /path/to/lrc-final/ROS2/* ~/ros2_ws/src/
```

3. **Build packages**:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

4. **Verify build**:
```bash
ros2 pkg list | grep lrc_arena
# Should show: lrc_arena_msgs, lrc_arena_nodes
```

## Running the System

### Terminal 1: ROS-TCP-Endpoint Bridge

This bridges Unity with ROS 2:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

Expected output: `[INFO] [timestamp]: Starting server on 0.0.0.0:10000`

### Terminal 2: Arena Device Controllers

Launch all arena device nodes:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch lrc_arena_nodes arena_devices.launch.py
```

Or run individually:
```bash
# Pump controller
ros2 run lrc_arena_nodes pump_controller

# LED controller
ros2 run lrc_arena_nodes led_controller

# Load cell + LCD controller
ros2 run lrc_arena_nodes loadcell_lcd_controller
```

### Terminal 3: SARM Teleoperation

Control the SARM robot with keyboard:

```bash
sudo apt install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/sarm/cmd_vel
```

Controls:
- `i` / `k` : forward / backward
- `j` / `l` : left / right
- `u` / `o` : rotate left / right
- `q` / `z` : increase / decrease speed
- `space` : stop

### Terminal 4: Monitor Topics

```bash
# Watch IR array data
ros2 topic echo /alfr/ir_array

# Watch gate crossings
ros2 topic echo /gates/g1_crossed
ros2 topic echo /gates/g2_crossed
ros2 topic echo /gates/g3_crossed

# Watch pump status
ros2 topic echo /pump/state
ros2 topic echo /pump/volume_dispensed

# Watch load cell
ros2 topic echo /load_cell/weight_g

# Watch LCD display
ros2 topic echo /lcd/display

# List all topics
ros2 topic list
```

## Unity Setup

### Install Unity Packages

In Unity Package Manager (Window → Package Manager), add:

1. ROS TCP Connector:
   ```
   https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
   ```

2. URDF Importer:
   ```
   https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
   ```

### Configure ROS Settings

In Unity: **Robotics → ROS Settings**
- **ROS IP Address**: Your ROS 2 machine IP (or `127.0.0.1` for localhost)
- **ROS Port**: `10000`
- **Protocol**: Select **ROS2**

### Attach Scripts to GameObjects

#### ALFR Robot
- Add `IRArrayPublisher.cs` (requires `IRsensorArray.cs`)
- Add `SonarPublisher.cs` (requires `SonarSensor.cs`)
- Tag: `ALFR`

#### Gate Objects
- Add `GateTriggerPublisher.cs` to Gate1, Gate2, Gate3
- Set `gateNumber` parameter (1, 2, or 3)
- Ensure GameObject has Collider with `isTrigger = true`

#### SARM Robot
- Add `CmdVelSubscriber.cs`
  - Assign wheel ArticulationBodies
  - Configure wheel radius and robot dimensions
- Add `GripperSubscriber.cs`
  - Assign left/right finger ArticulationBodies
  - Configure open/close positions

### Generate C# Messages

For custom messages (if using IRArray.msg):

1. **Robotics → Generate ROS Messages**
2. Click **Browse**, select `~/ros2_ws/src/lrc_arena_msgs`
3. Expand folders and click **Build 1 msg** for `IRArray.msg`
4. Generated file appears in `Assets/RosMessages/LrcArena/msg/IRArrayMsg.cs`

## Configuration

### Pump Controller Parameters

Edit in `arena_devices.launch.py`:

```python
'step_pin': 17,              # GPIO pin for STEP
'dir_pin': 27,               # GPIO pin for DIR
'steps_per_ml': 80.0,        # Calibration: steps per ml
'target_volume_ml': 125.0    # Volume to dispense
```

### LED Controller Parameters

```python
'num_pixels': 30,            # Number of LEDs
'brightness': 0.5            # 0.0 - 1.0
```

### Load Cell + LCD Parameters

```python
'expected_weight_g': 125.0,  # Expected weight in grams
'tolerance_g': 5.0,          # Tolerance ± grams
'team_name': 'Team LRC',     # Your team name for LCD
'hx711_dout': 5,             # GPIO pin for HX711 DOUT
'hx711_sck': 6               # GPIO pin for HX711 SCK
```

## Testing

### Test Individual Components

```bash
# Test pump (simulated)
ros2 topic pub /gates/g1_crossed std_msgs/msg/Bool "{data: true}" --once

# Test LED
ros2 topic pub /led/lam std_msgs/msg/Bool "{data: true}" --once

# Test gate crossing
ros2 topic pub /gates/g2_crossed std_msgs/msg/Bool "{data: true}" --once
```

### Check Node Status

```bash
ros2 node list
ros2 node info /pump_controller
ros2 node info /led_controller
ros2 node info /loadcell_lcd_controller
```

## Hardware Setup (Raspberry Pi)

### GPIO Pinout

```
Pump Controller:
- GPIO 17 (Pin 11) → A4988 STEP
- GPIO 27 (Pin 13) → A4988 DIR

LED Controller:
- GPIO 18 (Pin 12) → WS2812B DIN

Load Cell:
- GPIO 5 (Pin 29) → HX711 DOUT
- GPIO 6 (Pin 31) → HX711 SCK

LCD (I2C):
- GPIO 2 (Pin 3) → SDA
- GPIO 3 (Pin 5) → SCL
```

### Enable I2C

```bash
sudo raspi-config
# Select: Interface Options → I2C → Enable
sudo reboot
```

## Troubleshooting

### ROS-TCP-Endpoint Connection Failed

- Check IP address in Unity ROS Settings matches endpoint IP
- Verify firewall allows port 10000
- Test with: `telnet <ROS_IP> 10000`

### Messages Not Found in Unity

- Regenerate C# messages via **Robotics → Generate ROS Messages**
- Check message package is built: `ros2 interface list | grep lrc_arena`

### GPIO Permission Denied

```bash
sudo usermod -a -G gpio $USER
sudo reboot
```

### Load Cell Returns Zero

- Check wiring connections
- Verify HX711 power (3.3V or 5V)
- Calibrate: Place known weight and adjust calibration factor

### LCD Not Displaying

- Check I2C address: `sudo i2cdetect -y 1`
- Try alternate address (0x3F): Edit node parameter

## Competition Workflow

1. **Start ROS 2 nodes** (Terminal 1-2)
2. **Open Unity scene** (`Assets/Scenes/Main.unity`)
3. **Press Play** in Unity
4. **Teleop SARM** to clear obstacles (Terminal 3)
5. **ALFR autonomously navigates**:
   - Gate 1 → Pump dispenses 125 ml
   - Gate 2 → LED "LAM" illuminates
   - Gate 3 → Load cell checks weight → LCD shows result
6. **Monitor in Terminal 4**

## License

Apache 2.0 - See LICENSE file

## Support

For issues or questions, contact: team@example.com
