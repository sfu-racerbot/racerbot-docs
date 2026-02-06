# Lab 2

## 1. Set up the simulation
- We will be using the Humble distribution and this repository is a fork supporting it.
- There are many ways to run the simulation but we recommend using the Docker setup which requires no NVIDIA GPU (even if you have one). 
1. Make sure you have docker-compose installed (you likely have it from when you installed Docker in lab 1). To do so, open your native terminal and run `docker-compose -v`. If it gives the version, you are finished this step. If it says the command is not found, then install docker-compose [here](https://docs.docker.com/compose/install/)
2. To run the simulation go through the README until reaching the header "Docker" and continue until  reaching the header "Without an NVIDIA GPU" [here](https://github.com/f1tenth/f1tenth_gym_ros/tree/dev-humble?tab=readme-ov-file#launching-the-simulation).
3. Follow the instructions.
4. Once you have the simulation running, test it out by driving with the keyboard Teleop (all instructions should be in the README).
