# Roadmap

Throughout using `ROS 2`, you may find this [workflow](/resources/WORKFLOW.md) document to be very helpful.

## Module A: Introduction to ROS, RoboRacer & the Simulator

Introduces ROS 2, RoboRacer, autonomous vehicle fundamentals, and the simulation tools used throughout the course.

1. [Lecture 1](https://www.youtube.com/watch?v=v6w_zVHL8WQ)
    - [Lecture 1 Notes](notes/LECTURE-1.md)
2. [Tutorial 1](https://www.youtube.com/watch?v=EU-QaO6xTv4)
    - [Tutorial 1 Notes](notes/TUTORIAL-1.md)
3. [Lecture and Tutorial 1 Kahoot](https://create.kahoot.it/share/lecture-and-tutorial-1/dc25149f-1b0f-433c-aa85-e827dbcf4edd)

<hr style="border-top: 1px dotted #bbb; background: transparent;">

Build practical ROS 2 skills by creating packages, publisher/subscriber nodes, launch files, and working with Docker-based development environments.

4. [Lab 1](labs/lab-1/)
5. [Lab 1 Kahoot](https://create.kahoot.it/share/lab-1/f10a2047-54e6-4f0b-bb74-f979af09748a)

<hr style="border-top: 1px dotted #bbb; background: transparent;">

Introduces range sensors and Time-to-Collision (TTC) based safety methods for detecting obstacles and implementing Automatic Emergency Braking (AEB).

6. [Lecture 2](https://www.youtube.com/watch?v=k4FQ-dZ0Lp8)
    - [Lecture 2 Notes](notes/LECTURE-2.md) ([This](notes/LECTURE-2.md#working-with-laser-scan-data) part is very helpful for lab 2)
7. [Tutorial 2](https://www.youtube.com/watch?v=eeV9XHYXMlI)
    - [Tutorial 2 Notes](notes/TUTORIAL-2.md)
8. [Lecture and Tutorial 2 Kahoot](https://create.kahoot.it/share/lecture-and-tutorial-2/ead9421e-e5a4-47a2-b341-b16bc6b0b2c8)

<hr style="border-top: 1px dotted #bbb; background: transparent;">

Develop a ROS 2 safety node that uses LiDAR-based iTTC to detect imminent collisions and trigger Automatic Emergency Braking (AEB).

9. [Lab 2](labs/lab-2/)
    - Make sure you set up the [Official Workspace](https://github.com/sfu-racerbot/racerbot_ws) for this lab and for the future.

<hr style="border-top: 1px dotted #bbb; background: transparent;">

Introduces state estimation, coordinate frames, rigid-body transformations, and ROS 2 TF2 for tracking and transforming poses and sensor data between reference frames.

10. [Lecture 3](https://www.youtube.com/watch?v=XwNsJFcSQx4)
    - [Lecture 3 Notes](notes/LECTURE-3.md) ([This](notes/LECTURE-3.md#rigid-body-transformations) part is very helpful for understanding the math)
11. [Tutorial 3](https://docs.google.com/presentation/d/1GjGW39ufWI3bDcy7rMcaRYin_11soNIzCh2CtzO7580/edit?slide=id.p#slide=id.p)
    - [Tutorial 3 Notes](notes/TUTORIAL-3.md)
12. [Lecture and Tutorial 3 Kahoot](https://create.kahoot.it/share/lecture-and-tutorial-3/5f7bdca4-fc09-4610-9e83-a0bd9d6a16f2)

## Module B: Reactive Methods

Introduces feedback control and PID-based wall following, along with the practical setup, calibration, and operation of the F1TENTH race car.

1. [Lecture 4](https://www.youtube.com/watch?v=qIpiqhO3ITY)
    - [Lecture 4 Notes](notes/LECTURE-4.md)
2. [Tutorial 4](https://docs.google.com/presentation/d/1UCK95gx2mgJhX8YmabORjkEsaA3jR8FudmndzV3sHY4/edit?slide=id.p#slide=id.p)
    - [Tutorial 4 Notes](notes/TUTORIAL-4.md)

3. [Lecture 4 Kahoot](https://create.kahoot.it/share/lecture-4/a7d332d5-2cd8-4a25-846b-4a85c3156185)

4. [Tutorial 4 Kahoot](https://create.kahoot.it/share/tutorial-4/1dbf75df-11f3-4b2b-a37b-bccf62f48eb8) 

<hr style="border-top: 1px dotted #bbb; background: transparent;">

Develop a PID-based autonomous wall-following controller that uses LiDAR measurements to estimate wall distance, compute steering, and adjust speed for safe driving.

5. [Lab 3](labs/lab-3/)

<hr style="border-top: 1px dotted #bbb; background: transparent;">

Introduces reactive obstacle avoidance using Follow-the-Gap, safety bubbles, disparity extension, and LiDAR-based speed control for fast, map-free navigation.

6. [Lecture 5](https://www.youtube.com/watch?v=5asfD-_Z9x8)
    - [Lecture 5 Notes](notes/LECTURE-5.md)

7. [Lecture 5 Kahoot](https://create.kahoot.it/details/c0387e6b-b4cc-4c85-8e1d-2793871de368?drawer=)

<hr style="border-top: 1px dotted #bbb; background: transparent;">

Implement a reactive Follow-the-Gap algorithm that processes LiDAR scans, identifies safe driving gaps, selects a target point, and autonomously navigates the car through obstacles.

8. [Lab 4](labs/lab-4/)

<hr style="border-top: 1px dotted #bbb; background: transparent;">

Introduces vehicle states, dynamics models, tire behavior, understeer/oversteer, and map representations for modeling, simulation, planning, and autonomous driving.

9. [Lecture 6](https://www.youtube.com/watch?v=8zr5NUS05cM)
    - [Lecture 6 Notes](notes/LECTURE-6.md)

## Module C: Mapping & Localization

Introduces probabilistic localization and state estimation using Bayes filters, Kalman filters, and particle filters to fuse odometry and LiDAR measurements for estimating vehicle pose.

1. [Lecture 7](https://youtu.be/kQi5IGzvr0c?si=89jK2Q1ja-OfM-Ym) & [Lecture 8](https://youtu.be/SRBdpoPl57Q?t=1266) (from time 21:05)
    - [Lecture 7 Notes](notes/LECTURE-7.md)

<hr style="border-top: 1px dotted #bbb; background: transparent;">

Introduces SLAM, occupancy grid mapping, Hector SLAM scan matching, particle filter localization, resampling, and AMCL with KLD sampling for accurate map-based vehicle localization.

2. [Lecture 8](https://www.youtube.com/watch?v=SRBdpoPl57Q)
    - [Lecture 8 Notes](notes/LECTURE-8.md)

<hr style="border-top: 1px dotted #bbb; background: transparent;">

Introduces graph-based SLAM, pose graphs, spatial constraints, loop closure, least-squares optimization, and scan matching methods for correcting accumulated localization and mapping errors.

3. [Lecture 9](https://docs.google.com/presentation/d/1F92eaq1z98yY0OkGX_mlp3luqChNwqkPMNsiXZcayqM/edit)
    - [Lecture 9 Notes](notes/LECTURE-9.md)

<hr style="border-top: 1px dotted #bbb; background: transparent;">

Set up and run `slam_toolbox` for 2D mapping, then configure and run the particle filter for map-based localization, including RViz2 visualization, map selection, and initial pose setup.

4. [Tutorial 5](https://docs.google.com/presentation/d/1NSLurHQVMVvQxcS9Ak_V12aZkTbiSgpLeReDgo2snoU/edit?slide=id.p#slide=id.p)
    - [Tutorial 5 Notes](notes/TUTORIAL-5.md)

## Module D: Planning & Control

Introduces Pure Pursuit path tracking, including lookahead-based goal selection, curvature and steering control, speed-dependent tuning, ROS integration, and the limitations of the kinematic model.

1. [Lecture 10](https://www.youtube.com/watch?v=x9s8J4ucgO0)
    - [Lecture 10 Notes](notes/LECTURE-10.md)

<hr style="border-top: 1px dotted #bbb; background: transparent;">

2. Lab 5

<hr style="border-top: 1px dotted #bbb; background: transparent;">

3. Lecture 11

<hr style="border-top: 1px dotted #bbb; background: transparent;">

4. Lab 6

## Module E: Vision

1. Lecture 12

<hr style="border-top: 1px dotted #bbb; background: transparent;">

2. Lecture 13

<hr style="border-top: 1px dotted #bbb; background: transparent;">

3. Lab 7

## Module F: Special Topics
