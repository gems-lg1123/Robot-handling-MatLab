# Robot-handling-MatLab
## More details are in the ppt file
[![Video Preview](https://github.com/gems-lg1123/Robot-handling-MatLab/assets/127874636/dd6b4f3e-86e8-44bb-a95b-8219355ec434)](https://user-images.githubusercontent.com/127874636/282528735-3139b48c-30c3-454c-a83b-5f3d2115103c.mp4)
[![Video Preview](https://github.com/gems-lg1123/Robot-handling-MatLab/assets/127874636/ac57881f-0e0a-4542-bccd-134199864a68)](https://user-images.githubusercontent.com/127874636/282528743-0de5249e-78a9-44aa-84f5-f541b2859cfd.mp4)  

## Introduction
This project involves the creation and control of a robotic arm using MATLAB. The main focus is on constructing a robot model, performing both forward and inverse kinematics, visualizing the robot, and simulating its movements within a specified environment.

## Installation
- MATLAB (with appropriate toolboxes for robotics)
- Clone this repository or download the source code.

## Usage
To use this code:
1. Open MATLAB.
2. Navigate to the directory containing the code.
3. Run the main script to see the simulation of the robot's movement and kinematics.

## Features
- **Robot Model Construction**: Building a robot model using specified Denavit-Hartenberg parameters.
- **Kinematics**: Calculating forward and inverse kinematics to understand and control the robot's motion.
- **Visualization**: Visualizing the robot's movement in a simulated 3D environment.

## Code Overview
### Robot Construction
- The robot is constructed using Denavit-Hartenberg parameters.
- The robotic arm is defined with its joint configurations and transformations.

### Joint Space and Kinematics
- The script calculates the home configuration of the robot and its position transformations.
- Forward kinematics are used to determine the position of the end effector based on given joint angles.

### Inverse Kinematics
- The code includes functions for both analytical and numerical inverse kinematics, allowing the robot to reach specified positions and orientations.

### Path Visualization
- The script visualizes the trajectory of the robot's end effector in joint space.
- It includes detailed visualization of the robot's movement from a 'home' position to a target configuration.

### Simulation Environment
- The script creates a simulated environment with objects like shelves, plates, and wheels to demonstrate the robot's interaction capabilities.

## Contributing
Contributions to this project are welcome. Please ensure to follow the existing code structure and comment your code thoroughly.

