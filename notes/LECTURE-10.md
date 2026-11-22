# Lecture 10 Notes

**Lecture 10**: https://www.youtube.com/watch?v=x9s8J4ucgO0

## Pure Pursuit

### Overview

* **Pure Pursuit** is a geometric path-tracking algorithm used for lateral vehicle control.
* The vehicle follows a reference path by steering toward a **goal point** located some distance ahead.
* It relies on the **kinematic vehicle model** and assumes **no wheel slip**, ignoring vehicle dynamics.
* It is popular because it is simple and effective for robotics and autonomous driving.


### Intuition

* Pure Pursuit calculates the **curvature** that will move the vehicle from its current position to a goal position.
* The key idea is choosing that goal point **some distance ahead of the vehicle on the path**. The vehicle is always *chasing* a point that moves forward as it drives, hence "pursuit".
* This mirrors how humans drive: we look some distance down the road and steer toward that spot. That distance changes with the bends in the road and with what we can see.

### Autonomous Vehicle Stack

The system follows a **Sense → Plan → Act** loop:

* **Perception:** processes sensor data, builds/uses maps, and localizes the vehicle.
* **Planning:**

  * **Mission planner:** determines the overall goal.
  * **Behavioral planner:** decides what rules/actions to follow.
  * **Local planner:** finds an optimal local trajectory.
* **Control:** tracks the trajectory by generating steering and acceleration commands.

This loop typically runs at **20–50 Hz**.

## Core Idea

1. The vehicle is given a sequence of 2D **waypoints**.
2. It selects a **goal point** approximately a fixed **lookahead distance** $L$ ahead.
3. It computes a circular arc from the vehicle to the goal point.
4. The required curvature determines the steering command.
5. As the vehicle moves, the goal point and steering command are continuously updated.

The vehicle can be viewed as **chasing a moving point** along the path.

## Assumptions

* A sequence of waypoints is available.
* The vehicle can localize itself in the global map.
* Waypoints can be transformed into the vehicle's local coordinate frame.
* The vehicle follows a kinematic bicycle model.
* The vehicle's local frame is centered at the **rear axle**, with:

  * $x$: forward direction
  * $y$: left direction

## Geometric Interpretation

The goal point is represented as:

$$
(x, y)
$$

The distance from the rear axle to the goal point is the **lookahead distance**:

$$
L^2 = x^2 + y^2
$$

Infinitely many arcs pass through the vehicle and the goal point, so we need a constraint to choose one.

The car can't move sideways (no-slip), so it must leave its current position **tangent to its heading** (the $x$-axis). In the kinematic bicycle model, the car turns about its **instantaneous center of rotation (ICR)**. The ICR lies on the line through the rear axle, which is the vehicle's $y$-axis. Each steering angle $\delta$ gives a different ICR and radius $R$. The right $\delta$ is the one whose circle around the ICR passes through the goal point, so the distance from the ICR to the goal point equals $R$.

![Find Arc to Waypoint](/assets/module-c/lecture-10/find-arc-to-waypoint.png)

To obtain a unique circular arc, its center is constrained to lie on the vehicle's **y-axis**.

![Finding R](/assets/module-c/lecture-10/finding-r.png)

Let the arc's center be at $(0, R)$ on the $y$-axis, and let $d = R - |y|$ be the distance from the center to the goal point measured along $y$.

From the right triangle formed by the center, the goal point, and $d$:

$$
d^2 + x^2 = R^2
$$

Substitute $d = R - |y|$:

$$
(R - |y|)^2 + x^2 = R^2
$$

$$
R^2 - 2R|y| + y^2 + x^2 = R^2
$$

Use $x^2 + y^2 = L^2$ and cancel $R^2$:

$$
L^2 = 2R|y|
$$

The resulting radius is:

$$
R = \frac{L^2}{2|y|}
$$

Therefore, curvature is:

$$
\kappa = \frac{1}{R} = \frac{2|y|}{L^2}
$$

In code, use the **signed** $y$, i.e. $\kappa = 2y/L^2$, so the sign tells you which way to turn ($y>0$ means the goal is to the left, so steer left). The $|y|$ form only gives the size.

**Steering angle.** From the bicycle model with wheelbase $W$:

$$
\delta = \arctan(W\kappa) = \arctan\!\left(\frac{2Wy}{L^2}\right)
$$

For small angles, $\delta \approx W\kappa$, so steering is **proportional to curvature**. This is a **P-controller** on the lateral offset $y$ of the goal point:

$$
\delta \approx K\cdot\frac{2}{L^2}\, y
$$

The effective gain on $y$ is $\dfrac{2K}{L^2}$, so changing $L$ retunes the controller.
* Small $L$ means a high gain: aggressive steering that can oscillate.
* Large $L$ means a low gain: smooth steering that responds slowly.

Because $L$ is often set from speed (see *Speed and Lookahead Tuning*), the gain $2/L^2$ effectively changes with speed. You can also add an extra tunable gain $K$ on the curvature.

![Find Steering Angle](/assets/module-c/lecture-10/find-steering-angle.png)

## Picking the Goal Point

Ideally, select the point on the path exactly $L$ away from the vehicle.

If no waypoint lies exactly at that distance:

* **Don't** just pick the closest waypoint inside the circle. It may need an infeasible steering angle or point the wrong way.
* Option 1: pick the first waypoint **just beyond** $L$.
* Option 2: **interpolate** between the waypoint just inside and the waypoint just outside the circle, and use where that segment crosses the circle as the goal point.
* If the goal isn't exactly $L$ away, use its **actual** distance $\sqrt{x^2+y^2}$ in the curvature formula, not the nominal $L$.
* Search **forward from the last goal index**, not over the whole path. This stops the car from latching onto a waypoint on another part of the track that happens to be nearby.

### Basic Update Algorithm

1. Determine the current vehicle position.
2. Find the closest point on the path.
3. Find a goal point approximately $L$ ahead.
4. Transform the goal point into vehicle coordinates.
5. Calculate curvature.
6. Convert curvature into a steering command.
7. Update the vehicle position and repeat.

## ROS Pipeline

For each new vehicle pose:

**LiDAR → Particle Filter → Pose → Pure Pursuit → Steering Command**

1. The particle filter estimates the vehicle pose.
2. Pure Pursuit receives the pose.
3. A new goal waypoint is selected.
4. Curvature and steering angle are calculated.
5. The steering command is published.
6. The process repeats whenever a new pose is received.

## Effect of Lookahead Distance $L$

### Small $L$

* More aggressive steering.
* Higher curvature.
* Better ability to follow tight corners.
* Can cause oscillation or swerving, because the servo doesn't reach the commanded angle instantly. The car overshoots, then corrects.
* May generate dynamically infeasible trajectories.

### Large $L$

* Smoother trajectory.
* Less aggressive steering.
* Larger tracking error.
* May cut corners or get too close to obstacles.

**The lookahead distance is the main tuning parameter of Pure Pursuit.**

## Speed and Lookahead Tuning

* Use **lower speeds in high-curvature regions** such as corners.
* Use **higher speeds on straights**.
* **Lookahead from speed.** Set $L$ with a constant P-gain on velocity, then clamp it:

$$
L = \text{clip}(k_v v,\; L_{\min},\; L_{\max})
$$

* $k_v$ is tuned. A faster car looks further ahead, so it steers more gently: the gain $2/L^2$ drops as $v$ rises.
* $L_{\min}$ prevents twitchy, oscillating steering at low speed.
* $L_{\max}$ prevents cutting corners at high speed.
* **Corners need short $L$.** If $L$ is too big, the goal point jumps around a 90° corner before the car reaches it, so the car turns early and clips the inner wall.
* **Velocity per waypoint.** Each waypoint can carry a target speed (a velocity lookup table): slower where curvature or steering angle is high, faster on straights. A simple version is to scale speed down as $|\delta|$ goes up.

## Limitations

* Pure Pursuit does **not account for vehicle dynamics**.
* It assumes the no-slip condition.
* Aggressive maneuvers can violate this assumption and produce infeasible paths.
* It works best at slower speeds and when tires remain in the **linear tire region**.

## Practical Workflow

1. Create a map using SLAM.
2. Generate or record a set of waypoints.
    - Drive the track with teleop, or let a reactive method (e.g. follow-the-gap) drive it, while the particle filter runs. Save its pose estimates (e.g. to a CSV) as the waypoint list.
    - Later, waypoints can come from a global planner or an optimized raceline instead.
3. Smooth and evenly space the path. Recorded poses are noisy and unevenly spaced, so fit a spline (e.g. `scipy.interpolate.splprep` / `splev`) and resample at a fixed spacing.
    - Optionally store a **target velocity at each waypoint** (a velocity lookup table).
4. Localize the vehicle using a particle filter.
5. Subscribe to the vehicle pose.
6. Select a lookahead goal point.
7. Compute curvature and steering.
8. Repeat continuously to track the path.

![Pure Pursuit Waypoints](/assets/module-c/lecture-10/pure-pursuit-waypoints.png)

### Why check the path between waypoints?

On a test circle with a 5 m radius, the club's offline planner can move a waypoint sideways by at most 0.25 m in one pass. This limit is called the **trust region**. The old planner respected it at every waypoint, but the smooth curve joining them bulged outward: one new point landed at a radius of 5.250001622 m.

The planner now smooths the sideways move separately from the starting path, in coordinates that keep it within its limit. It spaces the new waypoints along that curve. The largest radius in the same test was 5.250000000 m.

This limit applies to one planning step. It does not replace the wall-clearance and steering checks on the finished line.

## Key Takeaway

**Pure Pursuit tracks a path by continuously selecting a point ahead of the vehicle and steering along the circular arc that reaches that point.**

Its behavior is primarily controlled by the **lookahead distance $L$**:

* **Smaller $L$** → tighter tracking but aggressive and prone to oscillation.
* **Larger $L$** → smoother but larger tracking error; cuts corners.
