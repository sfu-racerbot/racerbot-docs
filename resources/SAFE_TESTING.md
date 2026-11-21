# Work safely with the club cars

> **Who this is for:** anyone moving from coursework to the club's actual car software or joining a track session.
> **Read first:** [How Racerbot works](HOW_RACERBOT_WORKS.md).
> **You'll be able to:** pick a no-hardware test, know when to ask for supervision and follow the club's test order.
> **Time:** about 10 minutes to read; book any physical test with a maintainer.

## Test the car software in simulation

Use a laptop or development machine with **no car attached and no access to a live car's ROS network**. The car simulator can publish stand-in sensor data and a fake gamepad signal that reports LB held. Never run it beside real drivers or disable the check that keeps it from doing so.

1. **Set up the Jazzy workspace.** Use an Ubuntu 24.04 development machine with [ROS 2 Jazzy and development tools](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) installed and `rosdep` initialized. The car repository references other repositories as Git **submodules**. Ask your maintainer for team repository access if cloning a submodule is denied.

   **Development terminal:** open Bash in a folder where you keep projects.

   ```bash
   git clone --recurse-submodules https://github.com/sfu-racerbot/Racerbot-Car-Workspace.git racerbot-ws
   cd racerbot-ws
   source /opt/ros/jazzy/setup.bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ```

   **It worked if** the build summary shows no failed packages and the last line reports no error. Don't continue past a clone or build error; check the [workspace setup](https://github.com/sfu-racerbot/Racerbot-Car-Workspace#quick-start) with your maintainer.

2. **Start simulated sensors without a driving program.** Install the simulator engine once, then launch the simulated hardware with a car profile selected. Use the same profile and ROS isolation settings in any additional simulator terminals, and source Jazzy and the workspace in each one.

   **Same development terminal**, at the workspace root:

   ```bash
   tools/f1tenth_sim/setup.sh
   export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
   export RACERBOT_CAR=rb1
   ros2 launch racerbot_sim sim_bringup_launch.py hold_deadman:=false
   ```

   **It worked if** the log says `gym_bridge_node ready` and nothing drives. If it says `REAL HARDWARE DETECTED`, stop the simulator with Ctrl+C and check where you're running it. Don't bypass the interlock.

3. **Stop with Ctrl+C or continue with the [car simulator validation guide](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/main/docs/sim-validation.md).** That guide covers starting a driving node and checking that it stays stopped with LB released. Keep all ROS tests off the car's live network. A simulation pass doesn't prove real stopping distance, grip or sensor behaviour.

## Before a supervised lab or track session

- **Ask a maintainer** to confirm the car, the software version, the test area, who has the controller, and who can cut motor power. Know where that disconnect is before powering the motor.
- **Start with manual driving under supervision**, not autonomy. Use the [car workspace operations checklist](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/main/docs/operations.md#safety-checklist-every-time-before-powering-the-drive-motor). Check the F710 is in XInput mode and verify LB release stops motion with the wheels off the ground before driving on the floor.
- **Run only one control layer.** Manual and autonomous modes are alternatives, not a backup for each other. Every physical driving node must honour LB. Never disable the deadman, publish directly onto a motor command topic to get around it, or treat the dashboard as an emergency stop.
- **Watch first.** The dashboard's shared viewing connection is read-only, but separate controls can change settings, stop a process or affect maps. Don't arm tuning, call services or change a live run without the operator and maintainer's agreement.

## Test changes in this order

1. **No motor path.** With real drivers off, use an isolated ROS network to inspect the node's messages: check steering direction, bounded speed and that it won't command motion with LB released or the gamepad signal lost. See the [driving node test procedure](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/main/docs/writing-your-own-node.md#testing-before-its-on-wheels).
2. **Wheels off the ground.** With a maintainer, repeat the checks on the supported car after any new driving code or configuration change. Keep hands clear of wheels and steering, and confirm the stop on LB release before advancing.
3. **Low speed on the floor.** Only after the earlier checks pass: use a clear area, keep bystanders out, and have someone ready to cut power. Stop on unexpected motion, missing data or a failed stop check; don't increase speed to see if the problem goes away.

**To stop:** release LB. If the car doesn't stop as expected, have the designated person cut drive power using the agreed procedure. For a normal shutdown once stopped, end the control launch first, then the hardware launch with Ctrl+C, then power down as described in [car operations](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/main/docs/operations.md#shutting-down-cleanly).

**Hand off the result:** record which car and code revision you tested, whether it was in simulation, on stands or on the floor, what changed and what happened. If you saved a run, bring its log location to the maintainer; the [run diagnostics guide](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/main/docs/run-diagnostics.md) explains where to look.
