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
1. `tmux` is recommended when you're working inside a container. It allows you to have multiple `bash` sessions in the same terminal window which is very convinient when working inside containers.
2. It can be installed in the container via: `apt update && apt install tmux`.
3. Start a session with `tmux`.
4. You can call different `tmux` commands by pressing `ctrl+B` first and then the corresponding key. For example, to add a new window, press `ctrl+B` first and release and press `c` (create window) to create a new window. You can also move around with `ctrl+B` then `n` (next) or `p` (previous). 
3. `tmux` guide can be found [here](https://www.redhat.com/sysadmin/introduction-tmux-linux).

## 3: ROS 2 Basics
In your container terminal (in general, always use your container terminal unless specifically told not to), run:
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

**Instructions**:
1. Source ROS 2: `source /opt/ros/jazzy/setup.bash`
2. Create the `lab1_pkg` with `ackermann_msgs` as a dependency: `cd /lab1_ws/src` and run `ros2 pkg create lab1_pkg --build-type ament_cmake --dependencies ackermann_msgs`
3. Install dependencies with `rosdep`:
```bash
cd /lab1_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
4. Build: `cd /lab1_ws` and run `colcon build`
5. After build, your workspace should look like:
```bash
lab1_ws/
├── src/
│   └── lab1_pkg/
├── build/
├── install/
└── log/
```

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

**Instructions**:
1. We will now need to code! Since the container is mounted to your host machine, you can make changes on your host machine and they will change in your container. Thus, I recommend you use `VSCode` and open the folder `lab1_ws` in `VSCode`. If you want to code in the terminal instead, you can use `vim` by doing `apt update` and `apt install -y vim` or you can use `nano` by doing `apt update` and `apt install -y nano`.
2. To send and receive our message, we use the `ackermann_msgs` dependency but later we may use others like `std_msgs` and `geometry_msgs`. The core concepts should remain the same. For this lab, we already have that dependency working when we created our package. However, we **do need** another dependency called `rclcpp`. To add a dependency in an existing package, you need to add `<depend>rclcpp</depend>` to the `package.xml` file and add `find_package(rclcpp REQUIRED)` to the `CMakeLists.txt` file. You should add them below the existing ones for the other dependencies, keeping the file organized. **Remember how to do this, so in the future you can be confident in adding new dependencies**.
3. Create `talker.cpp` and `relay.cpp` in your src folder. These will be the two nodes we code. The source code: [talker](talker.cpp) and [relay](relay.cpp). **You likely don't know what each line of this code does. I recommend finishing the rest of this lab and then coming back to read and understand what each line does. You will probably need to ask AI to explain each step and learn inheritance, shared pointers, etc. Once you think you get it, try explaining it to your group mates or if you're alone (like me fr) then to a wall**.
4. Now you just need to add the two node executables we created into our `CMakeLists.txt` file. I'll do one, and you do the other.
```
add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp ackermann_msgs)
install(TARGETS talker DESTINATION lib/${PROJECT_NAME})
```
- The first line creates an executable named `talker` from the given C++ source files (in this case just `src/talker.cpp`).
- The second line links the executable against its ROS 2 dependencies. Any non-standard libraries used in the code (for example, packages included via `#include` that are not part of the C++ standard library) must be listed here, or you will get build/linker errors.
- The third line installs the executable into the ROS 2 install space (this is why we will be able to run using `ros2 run`).
5. Like before, (and always for the future) we install our dependencies with `rosdep` and build with `colcon`.
6. To run, we will use `tmux` and make 3 new terminals, giving us 4 in total to work with. The reason we need multiple terminals is that we cannot run commands on a terminal that is actively running a node.
- In first terminal **run the talker node**, setting your velocity and direction parameters:
    - `cd /lab1_ws && source install/local_setup.bash`
    - `ros2 run lab1_pkg talker --ros-args -p v:=2.0 -p d:=0.5`
- In the second terminal **run the relay node**:
    - `cd /lab1_ws && source install/local_setup.bash`
    - `ros2 run lab1_pkg relay`
- In the third terminal **listen to the drive talker**:
    - `cd /lab1_ws && source install/local_setup.bash`
    - `ros2 topic echo /drive`
- In the fourth terminal **listen to the drive relay**:
    - `cd /lab1_ws && source install/local_setup.bash`
    - `ros2 topic echo /drive_relay`
7. From the drive talker (third terminal) you should see:
```
drive:
  steering_angle: 0.5
  steering_angle_velocity: 0.0
  speed: 2.0
  acceleration: 0.0
  jerk: 0.0
```
8. From the drive relay (fourth terminal) you should see which should be 3 times speed and angle:
```
drive:
  steering_angle: 1.5
  steering_angle_velocity: 0.0
  speed: 6.0
  acceleration: 0.0
  jerk: 0.0
```
9. To stop a node (or anything) from running press `ctrl+c`. To delete a `tmux` terminal press `ctrl+b` and then `&` and then `y`.

## 6: Creating a launch file and a parameter file
**Deliverable 3**: Create a launch file `lab1_launch.py` (or `lab1_launch.xml`) that launches both of the nodes we've created (ROS 2 does not support C++ for launch scripts). If you want, you could also set the parameter for the `talker` node in this launch file. The launch file should be placed in a `launch/` folder inside your package.

**Instructions**:
1. Clearly adding a new terminal for each node is a little crazy. This is where building our launch file comes in handy, so it can launch all our nodes (with set parameters too) all at once.
2. Make a directory called `launch` inside our package (not in src - next to it). Now make the file `lab1_launch.py` inside this folder. You can copy the source code from [here](lab1_launch.py) and the Node parameters should make sense to you.
3. Now we need to add this to our `CMakeLists.txt` file. Hopefully from our last visit in `CMakeLists.txt` you now understand what this is doing.
```
# add launch
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```
4. Build using `colcon` (shouldn't need `rosdep` since no dependencies were added)
5. Now in one terminal launch it:
```
cd /lab1_ws && source install/local_setup.bash
ros2 launch lab1_pkg lab1_launch.py
```
6. In another terminal (always `tmux` so we stay in the container environment) you can run these two one at a time (`ctrl+c` to stop the first and then run the second so we only use two terminals!)
```
cd /lab1_ws && source install/local_setup.bash
ros2 topic echo /drive
ros2 topic echo /drive_relay
```

## 7: ROS 2 Commands
After you've finished all the deliverables, launch the two nodes and test out these ROS 2 commands:
```bash
ros2 topic list
ros2 topic info /drive
ros2 topic echo /drive
ros2 node list
ros2 node info /talker
ros2 node info /relay
```

**Bonus**: Your finished package structure should look like this
```
lab1_pkg/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── lab1_pkg/
│       └── (optional header files go here)
├── launch/
│   └── lab1_launch.py
└── src/
    ├── talker.cpp
    └── relay.cpp
```

## 8: Questions
Try thinking of your own answer before pressing details for mine.

- Q1: During this assignment, you've probably ran these two following commands at some point: `source /opt/ros/jazzy/setup.bash` and `source install/local_setup.bash`. Functionally what is the difference between the two?

- A1: <details>The first command sets up the core ROS 2 environment. The second command sets up your local terminal to find and run your ROS 2 packages. That's why we must do it for each new terminal even though it's the same container.</details>

- Q2: What does the `queue_size` argument control when creating a subscriber or a publisher? How does different `queue_size` affect how messages are handled? (Try answering without looking back at the code)

- A2: <details>Directly told in the code</details>

- Q3: Do you have to call `colcon build` again after you've changed a launch file in your package? (Hint: consider two cases: calling `ros2 launch` in the directory where the launch file is, and calling it when the launch file is installed with the package.)

- A3: <details>If you run the file directly (not using `ros2 launch`) then no rebuild needed because ROS 2 will use the launch file from the source space with your changes. If you use `ros2 launch` then rebuild is needed because it will run using the ROS 2 install space. In practice, you should just rebuild and always use `ros2 launch`.</details>

## Credits
Written by Milad Abdi and adapted from [f1tenth lab 1](https://github.com/f1tenth/f1tenth_lab1_template/blob/24f7320ebf1b9a325a2039d4208204da4454d1ab).
