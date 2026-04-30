# Lab 4: Follow the Gap

## I. Learning Goals

- Reactive methods for obstacle avoidance

## II. Overview

In this lab, you will implement a reactive algorithm for obstacle avoidance. While the base starter code defines an implementation of the F1TENTH Follow the Gap Algorithm, you are encouraged to try different reactive algorithms or a combination of several.

## III. Review of F1TENTH Follow the Gap

The lecture slides on F1TENTH Follow the Gap are the best visual resource for understanding every step of the algorithm. However, the steps are outlined below:

1. Obtain laser scans and preprocess them.
2. Find the closest point in the LiDAR ranges array.
3. Draw a safety bubble around this closest point and set all points inside this bubble to 0. All other non-zero points are now considered “gaps” or “free space”.
4. Find the max length “gap”, in other words, the largest number of consecutive non-zero elements in your ranges array.
5. Find the best goal point in this gap. Naively, this could be the furthest point away in your gap, but you can probably go faster if you follow the “Better Idea” method as described in lecture.
6. Actuate the car to move towards this goal point by publishing an `AckermannDriveStamped` to the /drive topic.

### IV. Implementation

Implement a gap follow algorithm to make the car drive autonomously around the Levine Hall map. There are two extra test maps `levine_blocked.png`, which is empty, and `levine_obs.png`, which has obstacles that are relatively hard to navigate through for you to evaluate your code on.

To change the map in the simulation, adjust `f1tenth_gym_ros/config/sim.yaml` for your desired map. You will need to do `colcon build` and relaunch the simulator after doing so.

The skeleton code for the node: [reactive_node.cpp](./reactive_node.cpp). Although you are highly encouraged to write it entirely yourself.

### V. Deliverables

**Deliverable 1**: Your code should start and run in simulation smoothly. The basic requirement is that your car should be able to navigate entire loops in `levine_blocked` map, and through at least 3 corners of the `levine_obs` map. A bonus would be if your implementation is able to complete all 4 corners and keeps lapping through the map.

**Deliverable 2** (optional): If possible, take a video of the algorithm running on the real car. You may use different parameters (i.e., a separate launch file) for the on-car deployment. 

### VI. Extra Resources

UNC Follow the Gap Video: https://youtu.be/ctTJHueaTcY

## Credits
Adapted from [f1tenth lab 4](https://github.com/f1tenth/f1tenth_lab4_template).
