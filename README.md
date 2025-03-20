# Robot Garden

**Robot Garden** is a project developed for the Mobile Robots course in the Master's in Intelligent Systems program at the University of the Balearic Islands. This project involves simulating, controlling, and modeling a robotic system capable of navigating and interacting with a structured environment—similar to an automated garden or maintenance robot.



https://github.com/user-attachments/assets/bcef8630-114c-4a92-aa83-3e7ed4e79127



## Team

- [**Ahmad Kamal Baig**](https://github.com/akbaig) - Simulation and Integration Lead
- **Irving Perez** - Simulation and Modeling Expert
- [**Joaquin Caballero**](https://github.com/Joaquinecc) - Project Manager
- [**Jose David Perez**](https://github.com/jd-perez96) - Documentation and Presentation Lead
- [**Umar Faruk Abdullahi**](https://github.com/ufakz) - Control Lead

## Installation

This project requires ROS Noetic on Ubuntu 20.04. Follow the steps below to set up your environment and workspace.

### Prerequisites

- **Operating System**: Ubuntu 20.04 (Focal Fossa)
- **ROS Version**: ROS Noetic Ninjemys

### ROS Installation

Make sure ROS Noetic is installed before proceeding. Refer to the official ROS installation guide for Ubuntu 20.04: [Ubuntu ROS Noetic Installation](https://wiki.ros.org/noetic/Installation/Ubuntu). Ensure that you install the full version (`ros-noetic-desktop-full`), which includes the necessary tools for simulation and visualization.

Additionally, download navigation package for path planning

```bash
sudo apt install ros-neotic-navigation
```

### Workspace Setup

1. **Create the catkin workspace**  
   Create and initialize a workspace for building the project:

    ```bash
    mkdir -p ~/robot_garden/src
    cd ~/robot_garden/src
    catkin_init_workspace
    ```

2. **Clone the repository**  
   Clone this repository into the `src` folder of your workspace:

    ```bash
    cd ~/robot_garden/src
    git clone https://github.com/ufakz/robot_garden
    ```

3. **Copy model files**  
   Copy models from [`p3at_gazebo/models`](p3at_gazebo/models) folder to your local gazebo folder located at `~/.gazebo/models`:

   ```bash
   cp robot_garden/p3at_gazebo/models ~/.gazebo/models
   ```

   Alternatively, you can create a symbolic link.

4. **Build the workspace**  
   Build and install the project packages from the `robot_garden` directory:

    ```bash
    cd ~/robot_garden
    catkin_make install
    ```

5. **Start the ROS Core** (Optional)

   In a terminal, start the ROS core:

    ```bash
    roscore
    ```

6. **Run Gazebo**  
   Launch Gazebo with the robot's environment configuration:

    ```bash
    cd ~/robot_garden
    source install/setup.bash
    roslaunch p3at_description garden_world.launch
    ```

8. **AMCL with move_base** (Navigation Pre-requisites)

   To run navigation algorithms using AMCL and Move Base, launch in separate terminal:

   ```bash
   cd ~/robot_garden
   source install/setup.bash
   roslaunch p3at_description navigation_main.launch
   ```

7. **Launch the control node**  
   In a separate terminal, run the (waypoint following) control node for the robot:

   ```bash
   cd ~/robot_garden
   source install/setup.bash
   rosrun p3at_plugin waypoint_following_movebase
   ```

   Note that this requires running **step 7** before. If you don't want to do that, try our completely reactive control node:

   ```
   rosrun p3at_plugin waypoint_following_custom 
   ```

   Alternatively, you can also use the **path planning** control node:

    ```
   rosrun p3at_plugin path_planning_movebase 
   ``` 

### Troubleshooting

Ensure all dependencies are correctly installed and the ROS environment is properly sourced in each terminal. Run `source devel/setup.bash` or `source install/setup.bash` in each terminal as necessary.

## Project Milestones

Below are the major milestones for the **Robot Garden** project. This checklist provides an overview of the development and integration steps:

- [x] **Define Application and Environment** - Established the scope, functionality, and the basic environmental model for the robot.
- [x] **Setup Simulation** - Configured Gazebo simulation environment and robot models.
- [x] **Design Environment** - Develop a more realistic and detailed simulation environment.
- [x] **Design Water Spray** - Implement a water spray mechanism for the robot.
- [x] **Final Robot Model** - Complete design of the robot model with final adjustments and additions.
- [x] **Waypoint Following Control** - Implement a control system for the robot to follow a series of waypoints.
- [x] **Path Following** - Develop a path-following algorithm for continuous navigation.
- [x] **Final Integration** - Integrate all components for a fully functional system.
- [x] **Documentation** - Finalize and polish the documentation to ensure ease of use and understanding.

## How to Contribute

To contribute to the project, follow these steps:

1. **Create a Branch**  
   Create a new branch for your feature or bug fix:

    ```bash
    git checkout -b your-feature-branch
    ```

2. **Make Changes**  
   Implement your changes.

3. **Push Changes and Create a Pull Request**  
   Push your branch to the remote repository:

    ```bash
    git push origin your-feature-branch
    ```

   Then, go to the GitHub repository, navigate to "Pull Requests," and submit a new pull request.
