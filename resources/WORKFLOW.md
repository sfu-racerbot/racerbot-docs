# Workflow

## Initial Setup (only done once)
1. Create a workspace following the `README` [here](https://github.com/sfu-racerbot/racerbot_ws).
2. Set up the simulator in the workspace following the instructions [here](https://github.com/sfu-racerbot/racerbot-docs/blob/main/resources/F1TENTH_GYM_ROS.md).

## Starting a Session
1. Open Docker Desktop.

2. Open a terminal (Linux/macOS) or PowerShell (Windows).

3. Run your workspace container.
```bash
docker start -ai racerbot
```

4. Launch `tmux`.
```bash
tmux
```

5. Source your environment.
```bash
source /racerbot_ws/.venv/bin/activate
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

6. Launch the simulator.
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

7. Connect to your simulator using [Foxglove](https://app.foxglove.dev/).

8. Open a new `tmux` window (`Ctrl+b` then `c`) and launch the keyboard teleop to manually control the car (optional).
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

9. Open a new `tmux` window and run your package(s).

## Building and Running Packages
1. Source your environment.
```bash
cd /racerbot_ws
source install/setup.bash
```

2. Create a package.
```bash
cd /racerbot_ws/src
ros2 pkg create your_pkg_name_pkg --build-type ament_cmake --dependencies dependency1 dependency2 dependency3
```
Replace `dependency1` `dependency2` `dependency3` with the dependencies your package needs (e.g., `rclcpp` `sensor_msgs`).

3. Use `rosdep` to install the dependencies specified in your `package.xml` file.
```bash
cd /racerbot_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

4. If you need to add, remove, or change dependencies after creating your package:

    1. You must update `package.xml`. 

    To remove a dependency, delete:
    ```
    <depend>to_delete_dependency_name</depend>
    ```
    To add a dependency, add (next to the other ones):
    ```
    <depend>new_dependency_name</depend>
    ```

    2. You must update `CMakeLists.txt`.

    To remove a dependency, delete:
    ```
    find_package(to_delete_dependency_name REQUIRED)
    ```
    To add a dependency, add (next to the other ones):
    ```
    find_package(new_dependency_name REQUIRED)
    ```

    3. Repeat the earlier `rosdep` step.

5. Add your `your_node_name.cpp` file inside `/racerbot_ws/src/your_pkg_name_pkg/src`.

6. Program your `cpp` file. Examples of code can be found [here](https://github.com/sfu-racerbot/racerbot_solutions).

7. Add your node executable into `CMakeLists.txt`.
```
add_executable(your_node_name src/your_node_name.cpp)
ament_target_dependencies(your_node_name dependency1 dependency2 dependency3)
install(TARGETS your_node_name DESTINATION lib/${PROJECT_NAME})
```

8. Add a launch file (optional).
    1. Inside `/racerbot_ws/src/your_pkg_name_pkg` create a folder `launch`.

    2. Create your launch file `your_node_name_launch.py` inside your `launch` folder.

    3. Program your `your_node_name_launch.py` file. Examples of code can be found [here](https://github.com/sfu-racerbot/racerbot_solutions).
    
    4. Add your launch file installation rules to `CMakeLists.txt`.
    ```
    # add launch
    install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
    )
    ```

9. Build all your packages using `colcon`.
```bash
cd /racerbot_ws
colcon build
```
Or build a specific package.
```bash
cd /racerbot_ws
colcon build --packages-select your_pkg_name_pkg
```

10. Source your environment again.
```bash
source install/setup.bash
```

11. Run your node.
```bash
ros2 run your_pkg_name_pkg your_node_name
```
Or run your launch file.
```bash
ros2 launch your_pkg_name_pkg your_node_name_launch.py
```

12. You can put information about your nodes in your `package.xml` or in a `README.md` inside your package (optional).
