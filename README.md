# Robot Garden

**Robot Garden** is a project developed for the Mobile Robots course in the Master's in Intelligent Systems program at the University of the Balearic Islands. This project involves simulating, controlling, and modeling a robotic system capable of navigating and interacting with a structured environment—similar to an automated garden or maintenance robot.

## Team

- **Ahmad Kamal Baig** - Simulation and Integration Lead
- **Irving Perez** - Simulation and Modeling Expert
- **Joaquin Caballero** - Project Manager
- **Jose David Perez** - Documentation and Presentation Lead
- **Umar Faruk Abdullahi** - Control Engineer

## Installation

This project requires ROS Noetic on Ubuntu 20.04. Follow the steps below to set up your environment and workspace.

### Prerequisites

- **Operating System**: Ubuntu 20.04 (Focal Fossa)
- **ROS Version**: ROS Noetic Ninjemys

### ROS Installation

Make sure ROS Noetic is installed before proceeding. Refer to the official ROS installation guide for Ubuntu 20.04: [Ubuntu ROS Noetic Installation](https://wiki.ros.org/noetic/Installation/Ubuntu). Ensure that you install the full version (`ros-noetic-desktop-full`), which includes the necessary tools for simulation and visualization.

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
   Copy models from [`p3at_gazebo/models`](p3at_gazebo/models) folder to your local gazebo folder located at `~/home/.gazebo/models`:

   ```bash
   cp robot_garden/p3at_gazebo/models ~/home/.gazebo/models
   ```

   Alternatively, you can create a symbolic link.

4. **Build the workspace**  
   Build and install the project packages from the `robot_garden` directory:

    ```bash
    cd ~/robot_garden
    catkin_make install
    ```

5. **Start the ROS Core**  
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

7. **Launch the control node**  
   In a separate terminal, run the control node for the robot:

    ```bash
    cd ~/pioneer3at_ws
    source install/setup.bash
    rosrun p3at_plugin p3at
    ```

### Troubleshooting

Ensure all dependencies are correctly installed and the ROS environment is properly sourced in each terminal. Run `source devel/setup.bash` or `source install/setup.bash` in each terminal as necessary.

## Project Milestones

Below are the major milestones for the **Robot Garden** project. This checklist provides an overview of the development and integration steps:

- [x] **Define Application and Environment** - Established the scope, functionality, and the basic environmental model for the robot.
- [x] **Setup Simulation** - Configured Gazebo simulation environment and robot models.
- [x] **Design Environment** - Develop a more realistic and detailed simulation environment.
- [ ] **Design Water Spray** - Implement a water spray mechanism for the robot.
- [ ] **Final Robot Model** - Complete design of the robot model with final adjustments and additions.
- [ ] **Waypoint Following Control** - Implement a control system for the robot to follow a series of waypoints.
- [ ] **Path Following** - Develop a path-following algorithm for continuous navigation.
- [ ] **Final Integration** - Integrate all components for a fully functional system.
- [ ] **Documentation** - Finalize and polish the documentation to ensure ease of use and understanding.

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