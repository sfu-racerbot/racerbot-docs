# Your first Racerbot session

> **Who this is for:** new SFU Racerbot members. You don't need a car or any robotics experience.
> **Read first:** nothing. If you're new to the terminal, start with the club's [terminal foundations guide](https://sfuracerbot.ca/resources/docs/beginners/terminal-foundations/).
> **You'll be able to:** choose the right workspace, start the course simulator and see what its virtual sensor reports.
> **Time:** plan for one session; the first downloads and build can take a while.

Racerbot builds and races small autonomous cars. Start on your laptop, where you can see how the software works without anything physical moving.

## Choose your workspace

A **workspace** (the folder containing the software you build and run) is where you'll work. **ROS 2** is how the pieces of that software send data to each other. Humble and Jazzy are two releases of ROS 2; the club uses them for different jobs.

| If you want to… | Use this |
|---|---|
| Follow the course, do the labs or try the simulator for the first time | [`racerbot_ws`](https://github.com/sfu-racerbot/racerbot_ws), ROS 2 **Humble** in Docker. Docker runs a separate Linux environment on your laptop. |
| Change the software that runs on the real cars, or test that same software in simulation | [`Racerbot-Car-Workspace`](https://github.com/sfu-racerbot/Racerbot-Car-Workspace), ROS 2 **Jazzy**. Both cars use this workspace on their onboard Jetson computers, which run Ubuntu 24.04. |

Don't source both releases in one terminal or copy one workspace's build output into the other. Don't use the course simulator or lab code on the physical car.

## Set up on your laptop

1. **Install Git and get Docker running.** Follow the [development environment guide](https://sfuracerbot.ca/resources/docs/beginners/dev-environment-setup/) for your operating system. Build and open the course container. The prompt should now be inside the container, not in your laptop's normal shell.
2. **Try the [ROS 2 primer](https://sfuracerbot.ca/resources/docs/beginners/ros2-primer/).** You'll see one small program send a message and another receive it. Then [Lab 1](../labs/lab-1/LAB-1.md) gives you a C++ exercise using the same idea.
3. **Start the [course simulator](F1TENTH_GYM_ROS.md).** Stay on a laptop away from any live car network. Follow the setup and launch steps in the Humble container. Leave the simulator running in its own terminal. Connect the visualizer as described there; you can check the data from the terminal even if you don't use the visualizer.

## Check that the simulator is sending data

On your laptop, open a new terminal and run `docker exec -it racerbot bash` to get a **second Bash terminal inside the same course container**. Run the following there:

```bash
source /racerbot_ws/.venv/bin/activate
source /opt/ros/humble/setup.bash
source /racerbot_ws/install/local_setup.bash
echo "$ROS_DISTRO"
ros2 topic list
ros2 topic info /scan
ros2 topic echo /scan --once --field header --qos-reliability best_effort
```

**It worked if** you see `humble`, `/scan` (the stream of distance readings) in the list, and `sensor_msgs/msg/LaserScan` with a publisher count of at least one. The last command prints when one reading was taken and the frame (the reference point for its measurements), then exits. These commands only read data.

**If it doesn't work:** If a `source` line says the file doesn't exist, check that you're inside the container and that the simulator guide's setup and build finished. If `/scan` is missing or the last command waits, press Ctrl+C and check the simulator terminal for errors. Make sure both terminals use the same container and ROS environment.

**When you're done:** press Ctrl+C in the simulator terminal and in any separate keyboard-control terminal you started.

If you used [Racerbot Utility](https://github.com/sfu-racerbot/racerbot_utility), run `racerbot-utility end-sim` on your laptop instead; add `--session-only` to leave the container running. This optional utility expects the Humble Docker container named `racerbot`. Never use its reset or keyboard-control commands on a ROS network shared with a car.

Next: [How Racerbot works](HOW_RACERBOT_WORKS.md), then the [course roadmap](../ROADMAP.md) or [safe testing with the car workspace](SAFE_TESTING.md).
