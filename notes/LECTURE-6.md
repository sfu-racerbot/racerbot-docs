# Lecture 6 Notes

**Lecture 6:** https://www.youtube.com/watch?v=8zr5NUS05cM

## Vehicle States

### Position ($p_1$, $p_2$)
- Translation in global/local frame
- Relative to the vehicle's center of gravity (CoG) or base frame
- Coordinates: (x, y) in meters

**Visual: Position**

![Position](/assets/module-b/lecture-6/position.png)

### Heading (Orientation) ($\Psi$)
- Rotation relative to x-axis in global/local frame
- Representations:
  - Angle: [0, 2π] or [-π, π]
  - RPY, rotation matrix, quaternion
- Pure rotation (no translation)

**Visual: Heading**

![Heading](/assets/module-b/lecture-6/heading.png)

**Visual: Representing angle with [0, 2π] vs. [-π, π]**

![Heading Angle Representation](/assets/module-b/lecture-6/heading-angles.png)

### Frenet Frame
- A way to describe position relative to a curved reference path instead of global (x, y) coordinates

- Coordinate system attached to the path:
  - $T$: tangent → direction the path is currently going (forward)
  - $N$: normal → direction perpendicular to the path (left/right)
  - $B = T \times N$: binormal → perpendicular to both (only meaningful in 3D)

- Coordinates:
  - $s$: longitudinal progress (distance traveled along the path, starting at $s = 0$)
    - moving in $s$ means moving along the tangent $T$
  - $d$: lateral offset from the path (measured along $N$)
    - $d > 0$: left of path
    - $d < 0$: right of path

- Intuition:
  - $s$ tells you *where you are along the road*
  - $d$ tells you *how far off the centerline you are*
  
**Visual: Frenet Frame**

![Frenet Frame](/assets/module-b/lecture-6/frenet-frame.png)

### Velocity ($v$)
- Linear velocity in vehicle frame $(v_x, v_y)$
- Absolute speed: $v_{abs} = \sqrt{v_x^2 + v_y^2}$
- Units: m/s
- Sensors: GPS, IMU, wheel speed sensors 
  - Limitations: Wheel slip not captured unless using a pivot sensor

### Acceleration ($a$)
- Linear acceleration in vehicle frame: $(a_x, a_y)$
- Units: m/s²

- $a_x$: longitudinal acceleration (speeding up / braking)
- $a_y$: lateral acceleration (turning left / right)

- Sensor: IMU (Inertial Measurement Unit)

### Rotational Motion
- Yaw (z-axis): ωz, αz
- Pitch (y-axis): ωy, αy
- Roll (x-axis): ωx, αx

**Visual: Rotation**

![Rotation](/assets/module-b/lecture-6/rotation.png)

### Steering Angle (δ)
- Angle between front wheel direction and vehicle x-axis
- Units: radians/degrees

**Visual: Steering Angle**

![Steering Angle](/assets/module-b/lecture-6/steering-angle.png)

### Tire & Slip Concepts

#### Slip Angles
- Sideslip angle (β): angle between vehicle velocity and chassis (longitudinal axis)
  - Car is moving partly sideways relative to where it’s pointed
  - Whole vehicle “slides” laterally (body misaligned with motion)
  - Large β is what you see visually in drifting
- Slip angle (α): angle between wheel heading and actual wheel motion direction
  - Tire is not rolling exactly where it’s pointed
  - Occurs at the contact patch (tire vs road interaction)
  - This is what generates lateral tire force for turning and drifting
- Slip ratio (s): normalized longitudinal difference between wheel speed and vehicle speed (traction/braking)
  - Tire is spinning faster or slower than the ground speed
  - Positive → wheelspin (acceleration, burnouts)
  - Negative → wheel lock tendency (braking, ABS)

**Visual: Slip Angles**

![Slip Angles](/assets/module-b/lecture-6/slip-angles.png)

### Visualizing your States
**Visualize signals and behaviors to:**
- Detect trends
- Compute and analyze errors
- Interpret sensor/data signals

- Tools:
  - PlotJuggler
  - matplotlib
  - rqt_plot

**Visual: Visualizing States**

![Visualizing States](/assets/module-b/lecture-6/visualizing-states.png)

## Vehicle Dynamics

### Types
- Longitudinal: acceleration/braking
- Lateral: turning behavior
- Vertical: road interaction (bumps, load transfer)

### Purpose of Modeling
- Predict trajectory
- Understand vehicle response
- Enable control and planning

**Visual: Purpose of Modeling**

![Purpose of Modeling](/assets/module-b/lecture-6/purpose-of-modeling.png)

### System Types
- Time-invariant: constant dynamics
- Time-variant: changing dynamics (e.g., tire friction)

## Vehicle Dynamics Models

### Model Spectrum
- Single Track (simple)
- Double Track (detailed)
- Multi-body (high fidelity)
- Finite element (very complex)

### Model Tradeoff
- **↑ Parameters → ↑ Accuracy**

**Visual: Dynamic Model Tradeoffs**

![Dynamic Model Tradeoffs](/assets/module-b/lecture-6/dynamic-model-tradeoffs.png)

### Model Types
- Analytical (equations)
- Data-driven
- Hybrid / residual physics

**Visual: Dynamic Model Types**

![Dynamic Model Types](/assets/module-b/lecture-6/dynamic-model-types.png)

## System Dynamics
Vehicle dynamics is the study of the vehicle in motion and its behavior during interaction with driver inputs.
  - Understanding the behavior of the vehicle under certain driver inputs (e.g. turning the steering wheel, pressing the brake pedal, accelerating on a curve, etc.)
  - Study and verify if the vehicle response is safe and comfortable to the passenger

Described via Ordinary Differential Equations (ODEs)
- Components:
  - State: $x$ (internal system variables)
  - Input: $u$ (external input / control)
  - Output: $y$ (measured response)

- System Dynamics:
  - The behavior over time of the states as a reaction to the inputs and the initial state
$$
\dot{x} = Ax + Bu
$$
- $A$: system matrix (how states affect each other)
- $B$: input matrix (how input affects the state)
- $\dot{x}$: time derivative of the state

- Output Equation:
  - Describes the relation between the state and the output
$$
y = Cx
$$
- $C$: output matrix (maps state to measured output)

## Single Track Model

### Assumptions
- Wheels of each axle are combined
- Center of gravity is at road level
- No roll or pitch motion
- No vertical dynamics
- No left/right wheel load differences
- Constant longitudinal speed
- Small steering angles (linear approximation)

### Kinematic Single Track Model (Bicycle Model)
- Simplest motion model (kinematics = geometry only, no forces)

Simplifications:
  - 2D motion (planar; no roll or pitch)
  - No tire dynamics (no slip angles, no lateral forces)

Best for:
  - Low speeds
  - Slow cornering
  - Negligible lateral acceleration

#### Math
- Steering angle is the Ackermann steering angle $\delta_A$

$$
\tan(\delta_A) = \frac{L}{R} \quad \Rightarrow \quad \delta_A \approx \frac{L}{R} \;\; (\text{small angles})
$$

- $L$: wheelbase, $R$: turning radius
- Assumes perfect tracking: vehicle follows the steering angle exactly (no slip)

- State and input:

$$
x = \begin{bmatrix} p_1 \\ p_2 \\ \Psi \end{bmatrix}, 
\quad
u = \begin{bmatrix} \delta \\ v \end{bmatrix}
$$

- Dynamics:

$$
\dot{p}_1 = v \cos(\Psi)
$$

$$
\dot{p}_2 = v \sin(\Psi)
$$

$$
\dot{\Psi} = \frac{v \tan(\delta)}{L}, \quad L = l_F + l_R
$$

- Small-angle approximation:

$$
\tan(\delta) \approx \delta \quad \Rightarrow \quad \dot{\Psi} \approx \frac{v}{L}\,\delta
$$

**Visual: Kinematic Single Track Model 1/2**

![Kinematic Single Track Model 1/2](/assets/module-b/lecture-6/kinematic-single-track-model-1.png)

**Visual: Kinematic Single Track Model 2/2**

![Kinematic Single Track Model 2/2](/assets/module-b/lecture-6/kinematic-single-track-model-2.png)

### Linear Single Track Model
- Captures vehicle behavior closer to physical limits (includes tire forces)

Simplifications:
  - 2D motion (planar; no roll or pitch)
  - Constant longitudinal velocity ($v = \text{const}$)
  - Small slip angles (linear tire model)

Best for:
  - Higher speeds
  - Aggressive / evasive maneuvers
  - Studying understeer / oversteer behavior

**Visual: Linear Single Track Model**

![Position](/assets/module-b/lecture-6/linear-single-track-model.png)

#### Linear Single Track Model

- State and input:

$$
x = \begin{bmatrix} \beta \\ \dot{\Psi} \end{bmatrix}, 
\quad
u = \delta
$$

- $\beta$: chassis side slip angle  
- $\dot{\Psi}$: yaw rate  
- $\delta$: steering angle

- State-space model:

$$
\dot{x} = A x + B u
$$

- System matrices:

$$
A =
\begin{bmatrix}
-\frac{c_F + c_R}{m v} & -\left(1 + \frac{c_R l_R - c_F l_F}{m v^2}\right) \\
\frac{c_R l_R - c_F l_F}{I_z} & -\frac{c_R l_R^2 + c_F l_F^2}{I_z v}
\end{bmatrix}
$$

$$
B =
\begin{bmatrix}
\frac{c_F}{m v} \\
\frac{c_F l_F}{I_z}
\end{bmatrix}
$$

---

- Tire forces (linear tire model):

$$
F_{y,F} = c_F \alpha_F, \quad F_{y,R} = c_R \alpha_R
$$

- Cornering stiffness definition:

$$
c_R = \frac{\partial F_{y,R}}{\partial \alpha_R}, \quad
c_F = \frac{\partial F_{y,F}}{\partial \alpha_F}
$$

- Slip angles (small-angle approximation):

$$
\alpha_F \approx \delta - \beta - \frac{l_F \dot{\Psi}}{v}
$$

$$
\alpha_R \approx -\beta + \frac{l_R \dot{\Psi}}{v}
$$

---

- Friction limit (tire-road interaction):

$$
|F_y| \leq \mu F_z
$$

- $\mu$: tire-road friction coefficient  
- $F_z$: normal force on the tire  

- Implication:
  - Higher $\mu$ $\Rightarrow$ larger achievable lateral forces  
  - Linear model is only valid **before saturation** (i.e., $F_y$ still proportional to $\alpha$)

---

- Improvements over kinematic model:
  - Accounts for **lateral tire forces** instead of pure geometry  
  - Captures **side slip ($\beta$)** of the vehicle  
  - Models **yaw dynamics** via inertia $I_z$  
  - Explains **understeer / oversteer** behavior:

$$
\text{Understeer if } c_F < c_R, \quad
\text{Oversteer if } c_F > c_R
$$

- Includes dependence on:
  - Mass $m$
  - Velocity $v$
  - Geometry ($l_F, l_R$)
  - Tire properties ($c_F, c_R$, $\mu$)

#### Assumptions
- Linear relation between lateral force and slip angle  
- Constant velocity ($\dot{v} = 0$)  
- Small angles: $\sin(\theta) \approx \theta$, $\tan(\theta) \approx \theta$  
- No longitudinal dynamics (no braking/acceleration)  
- Tire forces below friction saturation

#### Tire Model: Cornering Stiffness

**Visual: Cornering Stiffness**

![Cornering Stiffness](/assets/module-b/lecture-6/cornering-stiffness.png)

### Nonlinear Single Track Model
- We want to plan evasive maneuvers closer to physical limits
- Consider important effects such as understeer or oversteer

- Includes:
  - Velocity dependent on longitudinal forces and resistances
  - Longitudinal + lateral forces
  - No small-angle approximations
  - Considers longitudinal tire forces and longitudinal slip on the front and rear wheels
  - Nonlinear relation between tire force and side slip angle
- Models the tire dynamics with empirical models:
  - Pacejka (Magic Formula)
  - Fiala model

**Visual: Nonlinear Single Track Model**

![Nonlinear Single Track Model](/assets/module-b/lecture-6/nonlinear-single-track-model.png)

#### Math

- Nonlinear state-space model:

$$
\dot{x} = f(x, u)
$$

- State vector:

$$
x =
\begin{bmatrix}
p_1 \\
p_2 \\
\Psi \\
v_x \\
v_y \\
\dot{\Psi}
\end{bmatrix}
$$

- $p_1, p_2$: position  
- $\Psi$: heading  
- $v_x$: longitudinal velocity  
- $v_y$: lateral velocity  
- $\dot{\Psi}$: yaw rate  

- Input vector:

$$
u =
\begin{bmatrix}
\delta \\
F_x
\end{bmatrix}
$$

- $\delta$: steering angle  
- $F_x$: longitudinal force  

- Front slip angle:

$$
\alpha_F = \delta - \arctan\left(\frac{v_y + l_F \dot{\Psi}}{v_x}\right)
$$

- Rear slip angle:

$$
\alpha_R = -\arctan\left(\frac{v_y - l_R \dot{\Psi}}{v_x}\right)
$$

- Lateral tire forces:

$$
F_{y,F} = F_{y,F}(\alpha_F)
$$

$$
F_{y,R} = F_{y,R}(\alpha_R)
$$

- Wheelbase:

$$
L = l_F + l_R
$$

- Velocity magnitude:

$$
v = \sqrt{v_x^2 + v_y^2}
$$

- Sideslip angle:

$$
\beta = \arctan\left(\frac{v_y}{v_x}\right)
$$

## Double Track Model
- More detailed vehicle model (full lateral + longitudinal dynamics)

- Features:
  - Each wheel modeled individually
  - Individual wheel loads
  - Individual lateral and longitudinal tire forces

- Captures:
  - Center of gravity at height
  - Roll and pitch dynamics
  - Left/right load transfer
  - Front/rear (axle) load transfer
  - Variable longitudinal speed
  - Vertical dynamics

- Tire modeling:
  - More detailed (can include nonlinear tire behavior)

**Visual: Double Track Model**

![Double Track Model](/assets/module-b/lecture-6/double-track-model.png)

## Multi-Body Model
- Most detailed vehicle model (high-fidelity)

- Features:
  - Each wheel modeled individually
  - Includes sprung and unsprung masses
  - Individual wheel loads and forces

- Captures:
  - Roll and pitch dynamics
  - Left/right and front/rear load transfer
  - Full vertical dynamics (suspension effects)

- Tire modeling:
  - Nonlinear tire models (more accurate at limits)

**Visual: Multi-Body Model**

![Multi-Body Model](/assets/module-b/lecture-6/multi-body-model.png)

## Understeer vs Oversteer

**Visual: Understeer vs Oversteer**

![Understeer vs Oversteer](/assets/module-b/lecture-6/understeer-vs-oversteer.png)

### Understeer ("Plowing")
- Front tires reach friction limit first

- Physics:
  - Front tires are saturated (sliding instead of gripping)
  - Not enough lateral force to follow steering input

- Result:
  - Car turns less than intended
  - Path becomes wider than desired

- Stability:
  - Generally stable
  - Often self-corrects by slowing down (grip recovers)

### Oversteer ("Fish-tailing")
- Rear tires reach friction limit first

- Physics:
  - Front tires still grip and steer the car
  - Rear tires lose lateral grip and slide outward

- Result:
  - Car rotates more than intended
  - Rear swings outward, nose points into corner

- Stability:
  - Unstable
  - Can lead to spin if not corrected quickly (counter-steering required)

## Vehicle Dynamics Applications
- Vehicle Dynamics Simulation:
  - Provide realistic vehicle behavior in simulation

- State Estimation:
  - Estimate how the vehicle has moved (position, velocity, states)

- Behavior Prediction:
  - Predict how other vehicles will move / behave

- Model-Based Algorithms:
  - Model Predictive Control (MPC)
  - Model-based Reinforcement Learning

## Vehicle Dynamics Simulation

- Vehicle dynamics must be modeled/simulated to produce realistic vehicle behavior

- Simulation environments:
  - MATLAB
  - IPG CarMaker
  - Unreal Engine / Unity (real-time simulation)
  - AVL VSM

- Lightweight / research tools:
  - CommonRoad Vehicle Models (Python)
    - Used in F1TENTH Gym
    - Includes different vehicle model types
    - Based on ODE-based dynamics
    - Install via pip

### Maneuvers

- Open-loop maneuver:
  - Fixed inputs (no feedback control)
  - Vehicle reacts freely
  - Used to measure pure system behavior

- Closed-loop maneuver:
  - Driver/AI controls vehicle to follow a trajectory
  - Behavior depends on controller quality
  - Used for handling evaluation and realism

### Model trade-off
- Simple models → fast, fewer parameters, less accurate
- Complex models → realistic, many parameters, harder to calibrate

### Key Insight
- Tire dynamics = most critical factor

## Map Representations
### Purpose
- **Localization**: estimating where the vehicle is in the environment (pose on the map)
- **Path planning**: computing a safe and feasible path from current position to a goal
- **Decision making**: choosing the next action based on the situation (e.g., stop, overtake, yield)

### Process
- We can use different map representations to describe our surroundings (what and where obstacles are around us)

### Types
#### Occupancy Grid Map
- Creates a 2D grid covering the whole sensor area
- Each cell is (occupied / free / unknown)
- Mainly used for 2D mapping with LiDAR sensors
- $+$ Easy to create, easy to interpret, directly usable for path planning
- $-$ Difficult in 3D, limited localization precision, fixed discretization (discretization = dividing space into fixed grid cells or steps)
- Focus: small-scale robots (e.g. RoboRacer)

**Visual: Occupancy Grid Map**

![Occupancy Grid Map](/assets/module-b/lecture-6/occupancy-grid-map.png)

#### Point Cloud
- 3D set of points representing surfaces in space
- Mainly used for 3D mapping with LiDAR sensors
- $+$ Rich geometric detail, directly usable for perception and planning
- $-$ Expensive LiDAR sensors, large data volume, includes irrelevant points/noise
- Focus: real-world passenger vehicles

**Visual: Point Cloud Map**

![Point Cloud Map](/assets/module-b/lecture-6/point-cloud-map.png)

#### Feature Map
- Map built from extracted landmarks / key features in the environment
- Features are distinctive, repeatable points (e.g. poles, corners, edges)
- Used to match sensor data to known landmarks
- $+$ Low memory usage, works in 2D and 3D, no fixed grid discretization
- $-$ Not human-readable, no direct occupancy information
- Focus: localization

**Visual: Feature Map**

![Feature Map](/assets/module-b/lecture-6/feature-map.png)

#### Semantic Map
- Map that assigns meaning (labels) to objects in the environment
- Includes object classes (e.g. car, pedestrian, road, traffic sign)
- Combines perception + spatial information
- $+$ Adds semantic understanding, improves decision making and planning
- $-$ Depends on object detection quality, computationally expensive
- Focus: object detection and scene understanding

**Visual: Semantic Map**

![Semantic Map](/assets/module-b/lecture-6/semantic-map.png)

#### HD Map
- High-definition, highly detailed map of the driving environment
- Built from multiple sensors (LiDAR, camera, radar, GPS)
- Includes: lane geometry, road markings, traffic signs, barriers, etc.
- $+$ Very high precision, rich information, strong support for autonomous driving
- $-$ Expensive to create and maintain, high memory/bandwidth requirements
- Focus: real-world passenger vehicles

**Visual: HD Map**

![HD Map](/assets/module-b/lecture-6/hd-map.png)

## Key Takeaways
- Vehicle state = position + orientation + motion
- Models trade simplicity vs accuracy
- Kinematic → simple, low-speed
- Dynamic → realistic, high-speed
- Tire behavior dominates dynamics
- Map choice depends on application scale & needs
