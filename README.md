# EE 382V: Human Robot Interaction

### Author: Selma Wanna
### email: slwanna@utexas.edu

## Table of Contents
1. [About](#about)
2. [Installation](#installation)

## About
This repository is reserved for ROS programming assignments from Dr. Thomaz's
Human Robot Interaction class. There will be three small homework assignments
plus a final project. Each assignment will be contained in a ROS package.
Relevant documentation can be found in those package's README's.

## Installation
1. Navigate to your home directory and type
    ```
    git clone https://github.com/SouLeo/hri.git
    ```
2. Compile the packages
    ```
    cd ~/hri
    catkin build
    ```

3. Adjust your bashrc
    ```
    cd ~/
    vi ~/.bashrc
    ```
   Write the following at the end of the file
   ```
   source ~/hri/devel/setup.bash
   ```
