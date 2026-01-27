# Lab 1

## Learning Goals
- Become familiar with ROS 2 workflow
- Understand how to create nodes with publishers, and subscribers
- Understand ROS 2 package structure, files, and dependencies
- Create launch files

## 1: Set Up Docker
- For everything docker related, refer to [Docker Docs](https://docs.docker.com/reference/) for help.
1. Install docker [here](https://docs.docker.com/get-started/get-docker/)
2. Create a workspace directory on your computer. 
    - Open your terminal
    - Navigate to a folder to build your workspace in. Example: `ls` (to see current files) & `cd <folder_you_want_to_go_into>`.
    - Once at your desired folder location, create your workspace directory: `mkdir lab1_ws`
    - Go into your workspace: `cd lab1_ws`
3. Start the docker container in interactive mode (on the current terminal) with a bind mount to the workspace you just created: `docker run -it -v ./lab1_ws/src/:/lab1_ws/src/ --name f1tenth_lab1 ros:jazzy` (`./` is from your current directory, you could also use the absolute path to your workspace instead)
4. Your terminal is now inside the container you built!
5. If you close your terminal/container you can re-run it using `docker start -i f1tenth_lab1`. If you want to delete your container do: `docker rm f1tenth_lab1`.

## 2: Set Up TMUX
1. `tmux` is recommended when you're working inside a container. It allows you to have multiple `bash` session in the same terminal window which is very convinient when working inside containers.
2. It could be installed in the container via: `apt update && apt install tmux`. `tmux` allows you to have multiple `bash` session in the same terminal window.
3. Start a session with `tmux`.
4. You can call different `tmux` commands by pressing `ctrl+B` first and then the corresponding key. For example, to add a new window, press `ctrl+B` first and release and press `c` (create) to create a new window. You can also move around with `ctrl+B` then `n` (next) or `p` (previous). 
3. `tmux` guide can be found [here](https://www.redhat.com/sysadmin/introduction-tmux-linux).

## 3: Ros 2 Basics
In your container terminal (in general, always use your container terminal unless specifically told not to), and run:
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
```
You should see two topics listed:
```bash
/parameter_events
/rosout
```

## 4: Creating a Package
**Deliverable 1**: Create a package named `lab1_pkg` in the workspace we created. The package needs to meet this criteria:
- The package supports `C++`.
- The package needs to have the `ackermann_msgs` dependency.
- Both of these can be done by declaring the correct dependencies in `package.xml`.
- If declared properly the dependencies can be installed using `rosdep`.
- Your package folder should be neat. You shouldn't have multiple 'src' folders or unnecessary 'install' or 'build' folders.

## 5: Creating nodes with publishers and subscribers
**Deliverable 2**: Create two nodes in the package we just created. Use `C++` for these nodes.

The first node will be named `talker.cpp` and needs to meet this criteria:
- `talker` listens to two ROS parameters `v` and `d`.
- `talker` publishes an `AckermannDriveStamped` message with the `speed` field equal to the `v` parameter and `steering_angle` field equal to the `d` parameter, and to a topic named `drive`.
- `talker` publishes as fast as possible.
- To test node, set the two ROS parameters through command line, a launch file, or a yaml file.

The second node will be named `relay.cpp` and needs to meet this criteria:
- `relay` subscribes to the `drive` topic.
- In the subscriber callback, take the speed and steering angle from the incoming message, multiply both by 3, and publish the new values via another `AckermannDriveStamped` message to a topic named `drive_relay`.

## 6: Creating a launch file and a parameter file
**Deliverable 3**: Create a launch file `lab1_launch.cpp` that launches both of the nodes we've created. If you want, you could also set the parameter for the `talker` node in this launch file.

## 7: ROS 2 Commands
After you've finished all the deliverables, launch the two nodes and test out these ROS 2 commands:
```bash
ros2 topic list
ros2 topic info drive
ros2 topic echo drive
ros2 node list
ros2 node info talker
ros2 node info relay
```

## Commands

- `ls`: lists the file and directories
- `cd <path_to_go_to>` changes directory to the path given

## Questions

Q1: During this assignment, you've probably ran these two following commands at some point: `source /opt/ros/jazzy/setup.bash` and `source install/local_setup.bash`. Functionally what is the difference between the two?

Q2: What does the `queue_size` argument control when creating a subscriber or a publisher? How does different `queue_size` affect how messages are handled?

Q3: Do you have to call `colcon build` again after you've changed a launch file in your package? (Hint: consider two cases: calling `ros2 launch` in the directory where the launch file is, and calling it when the launch file is installed with the package.)

## Credits
- Adapted from [f1tenth lab 1](https://github.com/f1tenth/f1tenth_lab1_template/blob/24f7320ebf1b9a325a2039d4208204da4454d1ab).