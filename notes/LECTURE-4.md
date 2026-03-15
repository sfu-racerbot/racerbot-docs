# Lecture 4 Notes

**Lecture 4:** https://www.youtube.com/watch?v=qIpiqhO3ITY

## Tracking a Reference Signal

- **Goal:** Drive the RoboRacer **along a centerline** or maintain a desired trajectory.
- Requires **well-tuned steering and speed controllers** for stable and smooth motion.

Key questions:

- **How much should we steer?**
- **How much countersteer is needed for smooth maneuvers?**
- **How can the car drive around a track reliably?**

### Learning Outcomes

- Understand the **basics of PID control**
- Learn how to **compute control error**
- Recognize **controller failure modes** (e.g., oscillation, overshoot)

### Assignment

- Implement **wall following**
    - In **simulation**
    - On the **physical RoboRacer vehicle**

## Control Systems

### Open Loop Control

- **Open-loop controllers** send commands **without observing system output**.
- No measurement of system state → **no error correction**.

Example: Sending a **fixed motor command** to the wheels.

Works when:

- System dynamics are **simple**
- **No stability concerns**
- **Disturbances are minimal**

Limitations:

- Cannot correct for **disturbances, noise, or modeling errors**.

### Feedback Control

- **Feedback controllers continuously monitor system output**.
- Compute an **error signal**: `e(t) = setpoint - process_variable`

Where:

- **Setpoint (reference):** Desired value
- **Process Variable (PV):** Measured output
- **Error (e(t)):** Difference between desired and actual value

The controller uses this error to **adjust the control input**.

Example:

- Desired distance from wall = **1 m**
- Measured distance = **0.8 m**
- Error = **0.2 m**
- Controller adjusts **steering angle**.

### Controller Design Goals

A good controller should:

- **Maintain desired speed**
- Respond with **minimal delay**
- Avoid **large overshoot**
- Apply **smooth corrections**

Example: If speed drops below the setpoint, the controller **increases motor power smoothly** until the target speed is reached.

### High-Level vs Low-Level Control

#### High-Level Control

High-level control specifies **desired vehicle behavior**.

Examples:

- Drive wheels at **60 RPM**
- Maintain **distance from wall**
- Follow a **centerline**

#### Low-Level Control

Low-level control **converts high-level** commands into **motor signals**.

Process:

1. High-level command

   - Example: **wheel speed = 60 RPM**

2. Sent to the **Electronic Speed Controller (ESC)**
3. ESC converts command to **PWM duty cycle**
4. **PWM signal controls motor armature current**
5. Motor current determines **torque and wheel speed**

As the **armature current stabilizes**, the **motor speed settles around the target RPM**.

- **Visual: Motor Settling**
	![Motor Settling](/assets/module-b/motor-settling.png)

The ESC is configured so the programmer interacts only with **high-level commands** such as:

- velocity
- steering angle

instead of **PWM signals**.

### Feedback Control and Wall Following

- **Feedback controllers** continuously monitor system output and compute error: `e(t) = reference - current_state`.
- Core idea: **drive the error to zero**.

Examples:

- Speed control: Reference = target speed, Current state = measured wheel speed
- Wall following: Reference = desired wall distance, Current state = measured distance

Goal: **Maintain a fixed distance from a wall**.

- Inputs: Distance from LiDAR.
- Controller output: Steering angle.
- Control loop: Measure distance → Compute error → Convert to steering command → Update continuously.
- The controller adjusts steering to minimize error.

## PID control for wall following

### Control Objective

- **Goal:** keep the RoboRacer
    
    - **on the centerline**
    - **parallel to the walls**

These objectives are **not redundant**:

- Car can be **parallel to the walls but off-center**
- Car can be **near the centerline but misaligned**

### Mathematical Formulation

Define a **global reference frame**:

- Origin located on the **centerline**
- `θ` = angle between the car and the **x-axis**

Control goals:

- Centerline tracking  
    `y = 0`
- Alignment with walls  
    `θ = 0`

Velocity is **kept constant**; we only control **steering angle**.

### Lookahead Alignment Term

Instead of forcing `θ = 0` directly, introduce a **lookahead distance `L`**.

If the car continues at its **current heading** for `L` meters:

- predicted deviation from centerline: `L sin(θ)`

Properties:

- Larger `L` → **smoother corrections**
- Smaller `L` → **more aggressive steering**
    
We use: `L = 1.5 m`

### Error Definition

We want:

- `y = 0`
- `L sin(θ) = 0`
    
Define the tracking error:

`e(t) = -(y + L sin(θ))`

- `e(t) = 0` when the car is **centered and aligned**
- sign ensures **correct steering direction**
    
Control objective: `e(t) → 0`

### PID Components

- **Proportional (P):** `θ_d = Kp e(t)`. Larger error → larger correction. High Kp risks oscillation, low Kp slow response. Reduces steady-state error.
- **Derivative (D):** `Kd de(t)/dt`. Reacts to error change rate. Reduces overshoot, improves stability.
- **Integral (I):** `Ki ∫ e(t) dt`. Accumulates past error, reduces persistent offset. Risk of windup (clamp integral).

Default: PD controller with `Kp = 14`, `Kd = 0.09`. No I term to avoid destabilization.

## PID Tuning

Goal: determine suitable controller gains:

`Kp`, `Ki`, `Kd`

PID tuning depends on:

- **Plant dynamics** (what the system can tolerate)
- **Safety constraints**
- **Desired performance**
    
    - low steady-state error
    - fast settling time
    - minimal overshoot

Common approaches:

- **Manual tuning**
- **Heuristic rules** (e.g. **Ziegler–Nichols**)
- **Optimization algorithms**
    
PID is **model-free control**:

- the plant is **not explicitly modeled**
- gains are found through **experimentation and analysis**
    
In this course:

- vehicles differ slightly → **manual tuning required**
- the **F1Tenth simulator** was designed to have **similar dynamics to the real car**
- this reduces the **sim-to-real gap** and provides a **good starting point for tuning**

## Distance Finder and PID (Wall Following)

Objective:

- Use **LiDAR** to measure **distance from a wall**
- Control steering so the car **drives parallel to the wall at a fixed distance**

### Wall Distance Estimation

- **Visual: Instantaneous Error**
	![Instantaneous Error](/assets/module-b/instantaneous-error.png)
- LiDAR scan: 270° field, angles −135° to +135°.
- Choose rays at 0° (`b`) and θ° (`a`, 0°–70°).
- Wall angle: `α = atan((a cos(θ) - b) / (a sin(θ)))`
- Distance: `AB = b cos(α)`
- Instantaneous error: `desired - AB`, but leads to oscillations due to motion.
- Projected error: `AB + AC sin(α)` for lookahead.
- Use projected distance for error.

- **Visual: Projected Error**
	![Projected Error](/assets/module-b/projected-error.png)

### Applying PID Control

Use the standard PID form:

`u(t) = Kp e(t) + Ki ∫ e(t) dt + Kd de(t)/dt`

In this system we use **PD control only**:

`u(t) = Kp e(t) + Kd (e(t) - e(t-1))`

Where:

- `e(t)` = projected wall-following error
- `u(t)` = steering correction
    
Controller behavior:

- **P term:** corrects distance from wall
- **D term:** smooths response and reduces oscillation
    
The computed control value is used to **adjust the steering angle**.

### ROS Implementation

Wall following is implemented using **two ROS nodes**.

**Node 1: Distance Estimator**

- input: **LiDAR scan**
    
- computes:
    
    - wall orientation `α`
    - projected distance error 

**Node 2: PID Controller**

- input: **error from node 1**
- computes steering command using **PD control**
    
Control pipeline:

`LiDAR scan → compute distance/error → PD controller → drive command`

Requires using ROS concepts:

- **nodes**
- **topics**
- **publishers**
- **subscribers**

## Learn to Drive Straight in ROS

Example skeleton implementation for **centerline / wall following** in ROS.

Two nodes must be completed:

- `dist_finder.py`
- `control.py`

## Distance Finder Node (`dist_finder.py`)

Purpose: compute **wall distance error** from LiDAR.
Subscribes to:

- `LaserScan` (`sensor_msgs`)
    
Publishes:

- `pid_input` message

`pid_input` fields:

- `pid_error` → error for PID controller
- `pid_vel` → vehicle velocity
    
### LaserScan Data

Important field:

- `ranges[]` → array of distance measurements (meters)
    
Structure:

- `ranges[0]` → distance at `angle_min`
- last element → distance at `angle_max`
- spacing defined by `angle_increment`
    
### Functions to Implement

`getRange()`

- returns distance at a specified **LiDAR angle**
    
`callback()`

- called when a new **LaserScan message** arrives

Steps inside callback:

1. Select two rays on the **right side** of the car:
    
    - `0°`
    - `θ°`
        
2. Use distances `a` and `b` to estimate:
    
    - wall orientation
    - projected wall distance
        
3. Compute **PID error**
    
4. Publish `pid_input` message
    
Velocity is kept **constant**.

Quick validation:

- move the car **closer/farther from the wall**
- rotate the car
- verify error changes correctly.

## PID Control Node (`control.py`)

Purpose: convert **error → steering command**.

Subscribes to:

- `pid_input`
    
Publishes:

- `drive_param`
    
`drive_param` fields:

- `angle` → steering angle `[-100, 100]`
- `velocity` → throttle `[-100, 100]`
    
At startup the node requests:

- `Kp`
- `Kd`
- velocity
    
This allows **easy PID tuning**.

### Controller Implementation

Inside the `control()` callback:
1. Scale / amplify the error
2. Apply **PD control**
    
`u(t) = Kp * e(t) + Kd * (e(t) - e(t-1))`

3. Add `servo_offset`
    
    - compensates for **mechanical steering misalignment**
        
4. Clamp steering angle to:
    
`-100 ≤ angle ≤ 100`

## Running the System

Required nodes:

```
roscore
rosrun hokuyo_node hokuyo_node
rosrun race dist_finder.py
rosrun race control.py
```

Recommended:

- create a **ROS launch file** to start all nodes together.

## Application to the F1Tenth Race Car (Sim → Real)

Goal: control the vehicle using **physical units** (velocity, steering).

The car uses a **VESC motor controller**.

Benefits:

- ROS sends **velocity + steering commands**
- VESC handles **low-level motor control**
- avoids direct **PWM control**
    
### Motor Controller Setup

1. Connect to VESC using **VESC Tool**
2. Load provided **motor configuration XML**
3. Click **Write Motor Configuration** to apply changes

## Motor PID Tuning

To observe motor response:

1. Open **Realtime Data → RPM**
2. Start streaming data
3. Apply **target RPM** (≈ 2000–10000)
    
This creates a **step response**.

Desired behavior:

- fast **rise time**
- minimal **overshoot**
- near **zero steady-state error**
    
Adjust gains under:

`Motor Settings → PID Controllers`

If oscillations occur:

- adjust **Kd filter**
    
## Key Takeaways

- Reference tracking drives **error → 0**
- **PD control** used for wall following
- ROS nodes separate **sensing and control**
- **F1Tenth simulator** reduces sim-to-real gap
- **VESC** simplifies motor control and tuning
