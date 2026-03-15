# Tutorial 4 Notes

**Tutorial 4:** https://docs.google.com/presentation/d/1UCK95gx2mgJhX8YmabORjkEsaA3jR8FudmndzV3sHY4/edit?slide=id.p#slide=id.p

## 1. LiPo Batteries
**Battery**  
![Battery](/assets/module-b/battery.png)
- **Capacity:** Amount of power a battery can hold.
- **C-rating:** Max safe discharge → Burst current = C × Capacity (e.g., 50C × 5Ah = 250A)
- **Cell voltage:** 3.0–4.2V; nominal/storage 3.7V

**Charger Tips:**
- Ensure charger detects LiPo.
- Use **Balance Mode** for full charge
- Use **Storage Mode** for long-term storage
- Never use **Fast Mode**

**Safety:**
- Never over-discharge (use low-voltage buzzer)
- Store in safety cabinet; discharge to storage voltage if >1 week
- Avoid impacts (soft-cell LiPo)

## 2. Car Parts
**Visual: Car Component Levels**
**Car Levels**  
![Car Levels](/assets/module-b/car-levels.png)

**Chassis**  
![Chassis](/assets/module-b/chassis.png)

**Jetson Xavier NX**  
![Jetson Xavier NX](/assets/module-b/Jetson-Xavier-NX.png)

**VESC**  
![VESC](/assets/module-b/VESC.png)

**Power Board 1/2**  
![Power Board 1](/assets/module-b/power-board-1.png)

**Power Board 2/2**  
![Power Board 2](/assets/module-b/power-board-2.png)

**Battery Connection**  
![Battery Connection](/assets/module-b/battery-connection.png)


## 3. Powering the Car
- **Battery:** Plug the battery into the connector (with correct polarity) → ensure the power selector switch is at the battery position → power on
- **Power supply:** Plug the power supply into the wall and the jack on the power board → Make sure the power selector switch is at the power supply position → switch the power on (9–16V **ONLY**)
- Turn off all devices before switching
- VESC cannot run on supply alone; can run alone via battery (by unplugging the connector from the battery to the power board)

## 4. Connecting to Car
- Connect **mouse, keyboard, and monitor** to the Jetson
- Login: User: `nvidia` / pwd: `nvidia`
- Connect to the provided router in network settings: SSID: `NETGEAR57` / pwd: `grandowl947`

## 5. ROS 2 Humble Installation
1. Install [humble-desktop](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
2. Install `rosdep` [alternatives](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Install-Binary.html#installing-and-initializing-rosdep)

**udev check:**
```bash
ls /dev/sensors/
# expect: vesc, hokuyo (if USB LiDAR 30LX)
```

- If missing, set up udev rules ([F1TENTH guide](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/firmware/drive_workspace.html#udev-rules-setup))

## 6. F1TENTH Stack Setup
```bash
# Create workspace
cd $HOME && mkdir -p f1tenth_ws/src

# Clone repo
cd f1tenth_ws/src
git clone -b humble-devel https://github.com/f1tenth/f1tenth_system.git

# Update submodules
cd f1tenth_system
git submodule update --init --force --remote

# Install dependencies
cd $HOME/f1tenth_ws
rosdep update --rosdistro=humble
rosdep install --from-paths src -i -y

# Build
colcon build
```

## 7. LiDAR Configuration
- For USB LiDAR 30LX: comment `ip_address`, uncomment `serial_port`
- Config: `$HOME/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/sensors.yaml`

## 8. ROS_DOMAIN_ID
- Set unique ID to prevent cross-talk:

```bash
export ROS_DOMAIN_ID=<team_number>
```

- Info: [ROS Domain ID](https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html)

## 9. Start Teleop & LiDAR
```bash
# Source ROS
source /opt/ros/humble/setup.bash
cd $HOME/f1tenth_ws
source install/setup.bash

# Launch bringup
ros2 launch f1tenth_stack bringup_launch.py

# RVIZ
rviz2

# Should be able to launch RVIZ with rviz2 in the command line, add a scan topic, and change the frame to /laser.

```

## 10. Teleop & Deadman’s Switch (Joystick)
**Logitech Joystick**  
![Logitech Joystick](/assets/module-b/joystick.png)

- Use **D mode** (mode light off).
- Axis/buttons may need remapping depending on joystick.

### Connecting the Joystick (First Time)
1. Open **Bluetooth settings** and filter for joystick devices.
2. Hold **Share + PS** buttons until the light bar flashes.
3. Pair the device (check surroundings to avoid pairing the wrong joystick).

### PS4 Joystick
- Uses the **same button areas** as the Logitech joystick.

### Remapping Joystick Controls
1. Echo the joystick topic:
```bash
ros2 topic echo /joy
```

2. Identify desired **button/axis indices**.
    
3. Modify the following to **change the mapping:**

```
f1tenth_system/f1tenth_stack/config/joy_teleop.yaml
```

## 11. Remote Desktop

### Install NoMachine
1. Install on Jetson Xavier NX  
    [https://knowledgebase.nomachine.com/AR02R01074](https://knowledgebase.nomachine.com/AR02R01074)  
    (Xfce4 not required)
    
2. Install on laptop  
    [https://www.nomachine.com/download](https://www.nomachine.com/download)
    
3. Connect **laptop and car to the router**.
    
4. Find the car’s IP:

```bash
ifconfig
```

**Laptop Address**  
![Laptop Address](/assets/module-b/laptop-address.png)

## 12. Parameter Tuning
Guide:  
[https://f1tenth.readthedocs.io/en/foxy_test/getting_started/driving/drive_calib_odom.html](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/driving/drive_calib_odom.html)

You’ll have to find the right speed to erpm gain, and the right steering angle to servo offset so that the odometry is somewhat accurate.

Parameter file:

```
$HOME/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml
```

Parameters to tune:

- `speed_to_erpm_gain`
- `steering_angle_to_servo_offset`
- `steering_angle_to_servo_gain`

### vesc.yaml
```yaml
/**:
  ros__parameters:
    # erpm (electrical rpm) = speed_to_erpm_gain * speed (m/s) + speed_to_erpm_offset
    speed_to_erpm_gain: 4614.0
    speed_to_erpm_offset: 0.0

    # servo value (0 to 1) = steering_angle_to_servo_gain * steering_angle (radians) + steering_angle_to_servo_offset
    steering_angle_to_servo_gain: -1.2135
    steering_angle_to_servo_offset: 0.5304
```

### Tuning Steering Offset
Loop:

1. Start teleop.
2. Drive the car straight several times.
3. Adjust:

```
steering_angle_to_servo_offset
```

4. Rebuild:
    
```bash
colcon build
```

5. Repeat until car drives mostly straight.

### Tuning Steering Gain
Using the kinematic model:

- Wheelbase (L ≈ 0.33 m)
- Max steering angle (δ ≈ 0.36 rad)

Formulas:

```
β = arctan(0.5 tan(δ))
R = L / (2 sin(β))
```

Result:

- Turning radius ≈ **0.947 m**
- Half-circle diameter target ≈ **1.784 m**
    
#### Setup
- Place tape measure on floor.
- Align **rear axle** with 0 mark.

#### Loop
1. Start teleop.
2. Turn steering fully toward tape.
3. Drive slowly until rear axle crosses tape again.
4. Measure distance with car’s x-axis.

Target: **1.784 m**

Adjustment:

- Overshoot → **increase gain**
- Undershoot → **decrease gain**

Increment: **~0.1**

Rebuild after changes:

```bash
colcon build
```

### Tuning Speed-to-ERPM Gain (Odometry)

#### Setup
- Tape measure **4–5 m** on floor.
- Rear axle aligned with **0**.

#### Loop
1. Start teleop.
2. Echo odometry:

```bash
ros2 topic echo --no-arr /odom # if not working, remove --no-arr
```

Monitor:

```
pose.pose.position.x
```

3. Confirm start value = **0.0** (restart teleop if not).
4. Drive straight ~4 m (no steering).
5. Measure actual distance from rear axle.
6. Compare with odometry.

Adjustment:

- Reported distance **too large** → decrease gain
- Reported distance **too small** → increase gain

Typical magnitude: **thousands**

7. Rebuild:

```bash
colcon build
```

Repeat until error **< 2–3 cm**.
