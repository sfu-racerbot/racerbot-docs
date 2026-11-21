# How Racerbot works

> **Who this is for:** anyone who wants to know how the car sees, decides and moves.
> **Read first:** [Your first Racerbot session](GETTING_STARTED.md) if you want to see messages in the simulator first.
> **You'll be able to:** follow a reading from a sensor to a steering command and find the right repository for your task.
> **Time:** about 10 minutes.

## Follow one message

Imagine a wall ahead of the car. The **LiDAR** (a laser range sensor) measures how far away it is. A **node** is a running program with a specific job. The LiDAR driver node publishes (sends) its measurements to a **topic**, a named channel called `/scan`. A driving node subscribes (listens) to that channel and can use the data to decide how to steer.

| Part | What it does on the club car |
|---|---|
| Sensors and estimates | `/scan` carries distances. `/odom` carries **odometry**, an estimate of motion and position. The course simulator calls its odometry topic `/ego_racecar/odom` instead. |
| Controller | Manual control sends speed and steering requests on `/teleop`. Autonomous control (software making the driving decisions) sends them on `/drive`. |
| Command selector | The **mux** (short for multiplexer) chooses a request and sends it on `/ackermann_cmd`. The **VESC** motor controller and the servo then handle speed and steering via the hardware drivers. |
| Dashboard | The dashboard displays what the car sees and what the driving code plans to do. Watching doesn't send driving commands. |

A **launch file** is a recipe that starts a set of nodes together. On the physical car, the first launch starts the sensors, gamepad reader, hardware drivers and mux. It doesn't drive on its own.

A second launch starts **one** control layer: either manual or autonomous. Don't run both. The manual layer can keep sending a neutral command even when you aren't touching the sticks, so the mux keeps choosing it over autonomy.

## Know these terms before a test

- **Deadman:** the left bumper (LB) on the Logitech F710 gamepad. The operator has to hold it to allow motion, even in autonomy. Letting go requests a stop. It isn't a substitute for a supervised test and a way to cut power.
- **Pose and frame:** a pose is a position and heading. A frame is the reference you measure from (such as the car or the map). ROS 2's **TF** keeps track of how to translate between frames.
- **Mapping and localization:** mapping builds a picture of the track; localization estimates where the car is on that picture. **SLAM** (simultaneous localization and mapping) estimates both together. Follow-the-Gap steers towards open space using the live scan without a map; Pure Pursuit follows a planned line using a pose estimate.
- **IMU:** an inertial measurement unit measures acceleration and turning rate. Calibration corrects sensor errors and helps interpret readings in the car's coordinate frame.

To try IMU calibration without hardware, use the simulator in the [standalone IMU project](https://github.com/sfu-racerbot/imu-calibration). Use the [car workspace's calibration tools](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/tree/main/tools/imu_calibration) as the reference for the physical cars. Don't copy calibration files between cars.

## Know where to look next

| Need | Go to |
|---|---|
| Coursework, examples and exercises | [Course roadmap](../ROADMAP.md) and [Humble course workflow](WORKFLOW.md). |
| How the actual cars are wired and configured | [Car architecture](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/main/docs/architecture.md) and [car profiles](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/main/docs/car-profiles.md). One workspace serves both `rb1` and `rb2`; each has its own hardware and calibration. |
| Beginner articles and club information | [Club website](https://sfuracerbot.ca/resources/docs/beginners/). |
| Watch live with an approved team login | [Racerbot dashboard](https://dashboard.sfuracerbot.ca/). Choose the right car and start in Simple view without opening a control link. The [dashboard repository](https://github.com/sfu-racerbot/web-dashboards) explains Simple versus Advanced and has a mock car for development. |

Before you run anything that can move the car, read [Work safely with the club cars](SAFE_TESTING.md).
