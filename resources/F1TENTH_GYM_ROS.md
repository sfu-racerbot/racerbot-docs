# F1TENTH gym environment ROS2 communication bridge
These are instructions for using a containerized ROS communication bridge for the F1TENTH gym environment that turns it into a simulation in ROS2. These instructions are adapted from [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros/tree/dev-humble).

## Setting up the Simulation

### Entering your workspace
1. To set up the simulation, these instructions assume you are inside your `racerbot_ws` in a Humble Docker container with **port forwarding** enabled to communicate with the simulator (i.e., `-p 127.0.0.1:8765:8765`). It is **recommended** to use the [Official Workspace](https://github.com/sfu-racerbot/racerbot_ws). If you want to set up the simulator in its own workspace (**not recommended**), follow the instructions [here](https://github.com/sfu-racerbot/f1tenth_gym_ros) instead.

### Set up Virtual Environment (once per workspace)
1. Setup the virtual environment in your workspace folder:
```bash
sudo apt install python3.10-venv
python3 -m venv --system-site-packages /racerbot_ws/.venv
```
2. Activate the workspace:
```bash
source /racerbot_ws/.venv/bin/activate
```
3. Upgrade pip:
```bash
python3 -m pip install -U pip
```

### Adding the GYM (skip if already included)

If you do not have the f1tenth_gym_ros package inside `/racerbot_ws/src` then follow these steps:
1. Clone the GYM ROS:
```bash
cd /racerbot_ws/src
git clone -b dev-humble https://github.com/f1tenth/f1tenth_gym_ros.git
```
2. Clone `f1tenth_gym` inside `f1tenth_gym_ros`:
```bash
cd /racerbot_ws/src/f1tenth_gym_ros
rm -rf f1tenth_gym
git clone -b dev-humble https://github.com/f1tenth/f1tenth_gym.git
```

### Installing the Simulator
1. Install the `f1tenth_gym` with the `.venv` active (once per workspace):
```bash
cd /racerbot_ws/src/f1tenth_gym_ros/f1tenth_gym
source /racerbot_ws/.venv/bin/activate
pip install -e .
```
2. Install Dependencies and Build:
```bash
cd /racerbot_ws
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
colcon build
```

## Launching the Simulation
1. Ensure all environments are sourced (do this every time):
```bash
source /racerbot_ws/.venv/bin/activate
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```
2. Launch the simulator:
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
3. Open [Foxglove](https://app.foxglove.dev/)
4. Create an account
5. Click on `Open Connection`, then click `Foxglove WebSocket` and paste in the websocket's address printed in the terminal in blue i.e `[INFO] [launch.user]: Foxglove WebSocket: ws://localhost:8765`. **Just put `ws://localhost:8765` as that is almost always the address.** If you can see the topics on the left, then the connection is complete.
6. To visualize the simulation, you will need to change layout by importing the layout file `gym_bridge_foxglove.json` in the launch folder of this package.
   - Click on the top right button `default` (Might be named something different but it is the closest button on the right to "ws://localhost:8765" in the top middle)
   - Click import from file and open the `gym_bridge_foxglove.json` inside `racerbot/src/f1tenth_gym_ros/launch` folder.
7. You can then run another node by creating another bash session in `tmux` or a separate terminal.
8. Extra: [Foxglove Documentation](https://docs.foxglove.dev/docs) and [Slides showing how to use the simulator](https://docs.google.com/presentation/d/1ZH0S_Dn8dQs1g3JOH7Dz7HEdvfNYPRSv/edit?slide=id.g392d82339e7_2_105#slide=id.g392d82339e7_2_105). Foxglove is the recommended setup, but if you prefer RViz (old Gym setup), you can also use rviz to visualize the sim, with `gym_bridge.rviz` available in the launch folder of this package too.

## Configuring the simulation
- The configuration file for the simulation is at `f1tenth_gym_ros/config/sim.yaml`.
- Topic names and namespaces can be configured but is recommended to leave unchanged.
- The map can be changed via the `map_path` parameter. It can be a package-relative path like `maps/levine` or a built-in gym track name like `Spielberg`. The map follows the ROS convention; the image file and the `yaml` file should live together.
- The `num_agent` parameter can be changed to either 1 or 2 for single or two agent racing. Multi-agent racing (>2) is planned, but not yet supported by the gym_ros.
- The ego and opponent starting pose can also be changed via parameters, these are in the global map coordinate frame.

The entire directory of the repo is mounted to a workspace `/sim_ws/src` as a package. All changes made in the repo on the host system will also reflect in the container. After changing the configuration, run `colcon build` again in the container workspace to make sure the changes are reflected.

## Topics published by the simulation

In **single** agent:

`/scan`: The ego agent's laser scan

`/ego_racecar/odom`: The ego agent's odometry

`/map`: The map of the environment

A `tf` tree is also maintained.

In **two** agents:

In addition to the topics available in the single agent scenario, these topics are also available:

`/opp_scan`: The opponent agent's laser scan

`/ego_racecar/opp_odom`: The opponent agent's odometry for the ego agent's planner

`/opp_racecar/odom`: The opponent agents' odometry

`/opp_racecar/opp_odom`: The ego agent's odometry for the opponent agent's planner

## Topics subscribed by the simulation

In **single** agent:

`/drive`: The ego agent's drive command via `AckermannDriveStamped` messages

`/initalpose`: This is the topic for resetting the ego's pose via RViz's Foxglove's 2D Pose Estimate tool.

In **two** agents:

In addition to all topics in the single agent scenario, these topics are also available:

`/opp_drive`: The opponent agent's drive command via `AckermannDriveStamped` messages. Note that you'll need to publish to **both** the ego's drive topic and the opponent's drive topic for the cars to move when using 2 agents.

`/goal_pose`: This is the topic for resetting the opponent agent's pose via RViz's or Foxglove's 2D Goal Pose tool.

## Keyboard Teleop

The keyboard teleop node from `teleop_twist_keyboard` is also installed as part of the simulation's dependency. To enable keyboard teleop, set `kb_teleop` to `True` in `sim.yaml`. After launching the simulation, in another terminal, with ROS sourced, run:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Then, press `i` to move forward, `u` and `o` to move forward and turn, `,` to move backwards, `m` and `.` to move backwards and turn, and `k` to stop in the terminal window running the teleop node.

## Developing and creating your own agent in ROS 2

There are multiple ways to launch your own agent to control the vehicles.

- The first one is creating a new package for your agent in the `/sim_ws` workspace inside the sim container. After launch the simulation, launch the agent node in another bash session while the sim is running.
- The second one is to create a new ROS 2 container for you agent node. Then create your own package and nodes inside. Launch the sim container and the agent container both. With default networking configurations for `docker`, the behavior is to put The two containers on the same network, and they should be able to discover and talk to each other on different topics. If you're using noVNC, create a new service in `docker-compose.yml` for your agent node. You'll also have to put your container on the same network as the sim and novnc containers.


## FAQ & Debugging
### I have Python < 3.9
The current `f1tenth_gym` requires Python 3.9+. Use Ubuntu 22.04 with ROS 2 Humble or update your Python environment to 3.9+.

### This package is managed externally, PEP 668
You are trying to install the package using the system python. This is outdated and not recommended as per PEP 668. Please ensure you install `f1tenth_gym` inside a virtual environment as instructed with `.venv` above.

### Pyqt6 6.10 cached, fails to install
In rare cases, you might have a newer cached version of pyqt6 which breaks the .toml install. To resolve this, first install pyqt6 first using ```pip3 install pyqt6==6.7.1``` and then install the f1tenth_gym using ```pip3 install -e .```.

### Gym install hangs on PyQt6>6.7.1 installation
This has been documented happening on VMWare Fusion for Mac. This stems from using a server image of Ubuntu 22.04. Specifically, this happens because PyQt6 prompts you to accept its GPL license which you can not see/accept from a standard pip install. Please resort to installing PyQt6 6.7.1 with license as that is the maximum supported version for the Ubuntu 22.04 Arm Server Image. To resolve this, first install pyqt6 first using ```pip3 install pyqt6==6.7.1 --config-settings --config-license= --verbose``` and then install the f1tenth_gym using ```pip3 install -e .```.

### AttributeError: module 'coverage' has no attribute 'types'
This is due to an outdated coverage package. To resolve this, run ```pip3 install --upgrade coverage```. The minimum coverage version required is 7.6.1.

### ImportError: cannot import name 'Transpose' from 'PIL.Image'
This is due to an outdated pillow version. To resolve this, run ```pip3 install --upgrade pillow```. The minimum pillow version required is 9.1.0.

### ValueError: numpy.dtype size changed, may indicate binary incompatibility
This is due to an outdated scipy version. To resolve this, run ```pip3 install --upgrade scipy```. The minimum scipy version required is 1.13.0.

### "opencv>=3. invalid" error on pip install
This indicates that you have outdated pip, wheel or setuptools. To resolve this, you can run ```python3 -m pip install --upgrade pip wheel setuptools```. This will upgrade all tools used by python when pip installing packages.
