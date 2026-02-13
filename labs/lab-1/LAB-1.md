# Lab 1

- Quick note before starting: There are many ROS distributions and every few years, some get outdated. Whether you need to learn `foxy`, `humble`, `jazzy`, or some future ROS distribution, this lab will work. However, **you need to change all `humble` words with the `name` you are learning**. E.g., `/opt/ros/humble/setup.bash` -> `/opt/ros/jazzy/setup.bash` to make it work for `jazzy`.

## Learning Goals
- Become familiar with ROS 2 workflow
- Understand how to create nodes with publishers, and subscribers
- Understand ROS 2 package structure, files, and dependencies
- Create launch files

## 1: Set Up Docker
- For everything docker related, refer to [Docker Docs](https://docs.docker.com/reference/) for help.
- It is also **highly** recommended you look through this repo to understand common terminal commands from this [Tutorial](https://gist.github.com/bradtraversy/cc180de0edee05075a6139e42d5f28ce)
1. Install docker [here](https://docs.docker.com/get-started/get-docker/)
2. Create a workspace directory on your computer. 
    - Open your terminal
    - Navigate to a folder to build your workspace in. Example: `ls` (to see current files) & `cd <folder_you_want_to_go_into>`.
    - Once at your desired folder location, create your workspace directory and src folder: `mkdir -p ./racerbot_ws/src`
    - Go into your workspace: `cd racerbot_ws`
3. Start the docker container in interactive mode (on the current terminal) with a bind mount to the workspace you just created: `docker run -it -v ./racerbot_ws/src/:/racerbot_ws/src/ --name racerbot_ws ros:humble` (`./` is from your current directory, you could also use the absolute path to your workspace instead)
    - It is key you understand `-v ./racerbot_ws/src/:/racerbot_ws/src/` **creates a bind mount from your host folder** (`./racerbot_ws/src/`) **to the container folder** (`/racerbot_ws/src/`), so any changes you make on the host or inside the container are immediately reflected in the other.
4. Your terminal is now inside the container you built!
5. If you close your terminal/container you can re-run it using `docker start -i racerbot_ws`. If you want to delete your container do: `docker rm racerbot_ws`.

## 2: Set Up TMUX
1. `tmux` is recommended when you're working inside a container. It allows you to have multiple `bash` sessions in the same terminal window which is very convenient when working inside containers.
2. It can be installed in the container via: `apt update && apt install tmux`.
3. Start a session with `tmux`.
4. You can call different `tmux` commands by pressing `ctrl+B` first and then the corresponding key. For example, to add a new window, press `ctrl+B` first and release and press `c` (create window) to create a new window. You can also move around with `ctrl+B` then `n` (next) or `p` (previous). Lastly, you can delete your current terminal window with `ctrl+B` then `&` then `y`.
5. `tmux` guide can be found [here](https://www.redhat.com/sysadmin/introduction-tmux-linux).

## 3: ROS 2 Basics
In your container terminal (in general, always use your container terminal unless specifically told not to), source your **underlay** (system ROS):
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```
You should see two topics listed:
```bash
/parameter_events
/rosout
```

## 4: Creating a Package
**Deliverable 1**: Create a package named `lab1_pkg` in the workspace we created. The package needs to meet this criteria:
- The package supports C++.
- The package needs to have the `ackermann_msgs` dependency.
- Both of these can be done by declaring the correct dependencies in `package.xml`.
- If declared properly the dependencies can be installed using `rosdep`.
- Your package folder should be neat. You shouldn't have multiple 'src' folders or unnecessary 'install' or 'build' folders.

**Instructions**:
1. Make sure you are in the right directories for each command (I have included the `cd` needed for each). If your workspace doesn't look like step `5.` then instead of deleting directories or moving stuff, please restart the whole process (delete your container). Be careful your new container isn't nested, as there is a common bug that causes this.
2. Create the `lab1_pkg` with `ackermann_msgs` as a dependency:
```bash
cd /racerbot_ws/src
ros2 pkg create lab1_pkg --build-type ament_cmake --dependencies ackermann_msgs
```
3. Install dependencies with `rosdep`:
```bash
cd /racerbot_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
4. Build:
```bash
cd /racerbot_ws
colcon build
```
5. After build, your workspace should look like:
```bash
racerbot_ws/
├── src/
│   └── lab1_pkg/
├── build/
├── install/
└── log/
```

## 5: Creating nodes with publishers and subscribers
**Deliverable 2**: Create two nodes in the package we just created. Use C++ for these nodes.

The first node will be named `talker.cpp` and needs to meet this criteria:
- `talker` listens to two ROS parameters `v` and `d`.
- `talker` publishes an `AckermannDriveStamped` message with the `speed` field equal to the `v` parameter and `steering_angle` field equal to the `d` parameter, and to a topic named `drive`.
- `talker` publishes at regular intervals.
- To test the node, set the two ROS parameters through command line, a launch file, or a yaml file.

The second node will be named `relay.cpp` and needs to meet this criteria:
- `relay` subscribes to the `drive` topic.
- In the subscriber callback, take the speed and steering angle from the incoming message, multiply both by 3, and publish the new values via another `AckermannDriveStamped` message to a topic named `drive_relay`.

**Instructions**:
1. **Editing code inside container**:
    - We will now need to code! Since the container is mounted to your host machine, you can make changes on your host machine and they will change in your container. Thus, I recommend you use `VSCode` and open the folder `racerbot_ws` in `VSCode`. If you want to set up `VSCode` extensions for ROS 2, click [here](https://docs.ros.org/en/humble/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html). If you want to code in the terminal instead, you can use `vim` by doing `apt update` and `apt install -y vim` or you can use `nano` by doing `apt update` and `apt install -y nano`.
2. **Adding Dependencies**:
    - To send and receive our message, we use the `ackermann_msgs` dependency but later we may use others like `std_msgs` and `geometry_msgs`. The core concepts should remain the same. For this lab, we already have that dependency working when we created our package. However, we **do need** another dependency called `rclcpp`. 
    - To add a dependency in an existing package, you need to add `<depend>rclcpp</depend>` to the `package.xml` file and add `find_package(rclcpp REQUIRED)` to the `CMakeLists.txt` file. You should add them below the existing lines for your other dependencies, **do not put them at the bottom of the file** and keep it organized.
3. **Writing the Code**:
    - Inside `lab1_pkg` create a `src` directory and inside add `talker.cpp` and `relay.cpp` in your src folder. These will be the two nodes we code. The source code: [talker](talker.cpp) and [relay](relay.cpp). **Make sure you understand what each line of code does**.
4. **Update `CMakeLists.txt`**:
    - Now you just need to add the two node executables we created into our `CMakeLists.txt` file. Add them just below `# Find dependencies`. I'll do one (`talker`), and **you do the other (`relay`)**.
    ```
    add_executable(talker src/talker.cpp)
    ament_target_dependencies(talker rclcpp ackermann_msgs)
    install(TARGETS talker DESTINATION lib/${PROJECT_NAME})
    ```
    - The first line creates an executable named `talker` from the given C++ source files (in this case just `src/talker.cpp`).
    - The second line links the executable against its ROS 2 dependencies. Any non-standard libraries used in the code (for example, packages included via `#include` that are not part of the C++ standard library) must be listed here, or you will get build/linker errors.
    - The third line installs the executable into the ROS 2 install space (this is why we will be able to run using `ros2 run`).
5. Like before, (and always for the future) we install our dependencies with `rosdep` and build with `colcon`. The commands to do this are in **step 4**: `3.` & `4.`
6. To run, we will use `tmux` and make 3 new terminals, giving us 4 in total to work with. The reason we need multiple terminals is that we cannot run commands on a terminal that is actively running a node.
- In first terminal **run the talker node**, setting your velocity and direction parameters:
    ```
    cd /racerbot_ws && source install/local_setup.bash
    ros2 run lab1_pkg talker --ros-args -p v:=2.0 -p d:=0.5
    ```
- In the second terminal **run the relay node**:
    ```
    cd /racerbot_ws && source install/local_setup.bash
    ros2 run lab1_pkg relay
    ```
- In the third terminal **listen to the drive talker**:
    ```
    cd /racerbot_ws && source install/local_setup.bash
    ros2 topic echo /drive
    ```
- In the fourth terminal **listen to the drive relay**:
    ```
    cd /racerbot_ws && source install/local_setup.bash
    ros2 topic echo /drive_relay
    ```
7. From the drive talker (third terminal) you should see:
```
drive:
  steering_angle: 0.5
  steering_angle_velocity: 0.0
  speed: 2.0
  acceleration: 0.0
  jerk: 0.0
```
8. From the drive relay (fourth terminal) you should see values which are 3 times the speed and steering angle:
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
3. Now we need to add this to our `CMakeLists.txt` file (**right after where you added your executables**).
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
cd /racerbot_ws && source install/local_setup.bash
ros2 launch lab1_pkg lab1_launch.py
```
6. In another terminal (always `tmux` so we stay in the container environment) you can run these two one at a time (`ctrl+c` to stop the first and then run the second so we only use two terminals!)
```
cd /racerbot_ws && source install/local_setup.bash
ros2 topic echo /drive
ros2 topic echo /drive_relay
```

## 7: ROS 2 Commands
After you've finished all the deliverables, launch the two nodes and test out these ROS 2 commands one at a time (try and guess the output before you run the command):
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

- Q1: During this assignment, you've probably ran these two following commands at some point: `source /opt/ros/humble/setup.bash` and `source install/local_setup.bash`. Functionally what is the difference between the two?

- A1: <details>The first command sets up the core ROS 2 environment. The second command sets up your local terminal to find and run your ROS 2 packages. That's why we must do the second one for each new terminal even though it's the same container.</details>

- Q2: What does the `queue_size` argument control when creating a subscriber or a publisher? How does a different `queue_size` affect how messages are handled? (Try answering without looking back at the code)

- A2: <details>The `queue_size` controls how many messages are buffered if the subscriber or publisher can’t immediately handle them. A larger queue stores more messages, which is useful if you need to process every message. For real-time control (like driving commands), a small queue (even 1) is ideal, because you only care about the latest message and old ones can be dropped.</details>

- Q3: Do you have to call `colcon build` again after you've changed a launch file in your package? (Hint: consider two cases: calling `ros2 launch` in the directory where the launch file is, and calling it when the launch file is installed with the package.)

- A3: <details>If you run the file directly (not using `ros2 launch`) then no rebuild needed because ROS 2 will use the launch file from the source space with your changes. If you use `ros2 launch` then rebuild is needed because it will run using the ROS 2 install space. In practice, you should just rebuild and always use `ros2 launch`.</details>

## Credits
Written by Milad Abdi, assisted by AI, and adapted from [f1tenth lab 1](https://github.com/f1tenth/f1tenth_lab1_template).
