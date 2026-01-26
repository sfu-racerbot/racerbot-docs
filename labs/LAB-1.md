# Lab 1

## Learning Goals
- Become familiar with ROS 2 workflow
- Understand how to create nodes with publishers, and subscribers
- Understand ROS 2 package structure, files, and dependencies
- Create launch files

## 1. Set Up Docker
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

## 2. Set Up TMUX


## Commands

- `ls`: lists the file and directories
- `cd <path_to_go_to>` changes directory to the path given

## Questions (outdated with foxy)

Q1: During this assignment, you've probably ran these two following commands at some point: `source /opt/ros/foxy/setup.bash` and `source install/local_setup.bash`. Functionally what is the difference between the two?

Q2: What does the `queue_size` argument control when creating a subscriber or a publisher? How does different `queue_size` affect how messages are handled?

Q3: Do you have to call `colcon build` again after you've changed a launch file in your package? (Hint: consider two cases: calling `ros2 launch` in the directory where the launch file is, and calling it when the launch file is installed with the package.)

## Credits
- Written by Milad Abdi and adapted from [f1tenth lab 1](https://github.com/f1tenth/f1tenth_lab1_template/blob/24f7320ebf1b9a325a2039d4208204da4454d1ab).