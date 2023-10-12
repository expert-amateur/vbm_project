# Vision-Based Manipulation (VBM) Project

This repository contains the code for the Vision-Based Manipulation (VBM) project as part of WPI's Vision Based Manipulation course. The project utilizes top surface information from an RGBD camera to find grasping points.

## ROS2 Version

This project is developed and tested with ROS2 Humble.

## Table of Contents
- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Introduction

Provide a brief introduction to the project. Explain what the project does, its goals, and why it's important. Mention any specific features or components that make this project unique.

## Prerequisites

List the prerequisites that are necessary to run your project. Include hardware requirements, software dependencies, and any other setup that users need to have in place before running your project.

## Installation

Provide step-by-step instructions on how to install and set up the project on a user's machine. Include installation of ROS2 and any additional dependencies. You can also include a list of commands or scripts to automate the installation process.


## Steps to run the packages
- ros2 launch vbm_project_env simulation.launch.py
- ros2 run rgbd_to_pointcloud depth_image_capture
- ros2 run vbm_project_grasping processPointCloud
- ros2 run vbm_project_grasping grasp_synthesis_node.py

```bash
# Example installation steps
$ git clone https://github.com/your/repo.git
$ cd repo
$ colcon build
