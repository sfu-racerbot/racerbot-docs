# Lab 5: SLAM and Pure Pursuit

## I. Learning Goals

- SLAM
- Localization with Particle Filter
- Pure Pursuit Algorithm

## II. Running slam_toolbox on the car

Follow the instructions in class to run `slam_toolbox` to make a map of Levine second floor. Save the map as `levine_2nd.pgm` and `levine_2nd.yaml`.

## III. Localization with Particle Filter

Follow the instructions in class to run `particle_filter` on the car using the new map you've made on Levine second floor.

## IV. Pure Pursuit Implementation

We have provided a skeleton for the pure pursuit node: [pure_pursuit_node.cpp](./pure_pursuit_node.cpp). As per usual, test your algorithm first in the simulator before you test it on the car. When you're testing in the simulator, use the ground truth pose provided by the sim as the localization. When you move to the car, use particle filter to provide localization.

As shown in the lecture, the curvature of the arc to track can be calculated as:

$$\gamma = \frac{2|y|}{L^2}$$

## V. Logging Waypoints

There are several methods you can use to create waypoints for a specific map.

- **Recording a trajectory of a joystick driven path.** You can write a node that subscribes to the pose provided by the particle filter localization, and save the waypoints to a CSV file. A similar script is provided by the F1TENTH team. Note that the original script is in ROS 1 and you'll have to write a ROS 2 node.

- **Find key points in the map** (e.g. in the Levine loop, the four corner centers of the loop) and create an interpolated spline that goes through all four corners. You can use functions such as `scipy.interpolate.splprep` and `scipy.interpolate.splev`.

Usually, you'll just save the waypoints as `.csv` files with columns such as `[x, y, theta, velocity, arc_length, curvature]`. With pure pursuit, the bare minimum is `[x, y]` positions of the waypoints. Another trick is that you can also smooth the waypoints if you decided to record them with the car. You can subsample the points you gathered and re-interpolate them with the scipy functions mentioned above to find better waypoints.

## VI. Visualizing Waypoints

To visualize the list of waypoints you have, and to visualize the current waypoint you're picking, you'll need to use the `visualization_msgs` messages and RViz.

## VII. Deliverables

You don't need to hand anything in for this lab — just show each part working. A video is optional.

**Deliverable 1**: Run `slam_toolbox` and show the map you generated of Levine second floor (`levine_2nd.pgm` and `levine_2nd.yaml`).

**Deliverable 2**: Run `particle_filter` localization on the car using the new map you made.

**Deliverable 3**: Show your pure pursuit node running smoothly in simulation, with the car following the waypoints.

**Deliverable 4** (optional): Take a video of the real car following waypoints in the Levine hallway. Show a screen recording of RViz.

## Credits

Adapted from [f1tenth lab 5](https://github.com/f1tenth/f1tenth_lab5_template).
