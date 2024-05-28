# FACT-Fast-and-Active-Coordinate-Initialization
Open source code for our paper FACT: Fast and Active Coordinate Initialization for Vision-based Drone Swarm.

## Introduction
This repository contains the code for our paper FACT: Fast and Active Coordinate Initialization for Vision-based Drone Swarm. We provide the code for the simulation. The code base is still under development for easier deployment and use. We will keep updating the code base and provide more detailed instructions.

The simulation and trajectory planning part of the code is based on our previous work [Ego-Planner](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2).

## Prerequisites
We developed the code with Ubuntu 20.04 and ROS Noetic. It might work with older distributions, but we recommend using Ubuntu 20.04 and ROS Noetic for simplicity. We recommend using computers with NVIDIA GPUs with CUDA installed. The code has been tested on two laptops with NVIDIA GTX 1660 Ti-MaxQ and RTX 3070.

### Mosek
We use Mosek v10.1 to solve the optimization problem. You can get a free academic license from [Mosek](https://www.mosek.com/products/academic-licenses/). More information on [license](https://docs.mosek.com/latest/licensing/index.html).

After you get the license, download Mosek v10.1 from [here](https://www.mosek.com/downloads/list/10/)(v10.1.22 and v10.1.23 have been tested) and extract the 'mosek' folder to your home directory. Then put the 'mosek.lic' file in the 'mosek' folder. To install Mosek, open a terminal and navigate to the 'mosek' folder and run the following command:
```bash
cd 10.1/tools/platform/linux64x86/src/fusion_cxx/ # the command may vary depending on the version and your platform
make install -j
```
Detailed instructions of installing Mosek can be found here [here](https://docs.mosek.com/latest/install/installation.html).

### ROS
We use ROS Noetic for the simulation. You can install ROS Noetic by following the instructions [here](http://wiki.ros.org/noetic/Installation/Ubuntu). We recommend installing the full-desktop version.

## Installation
Clone the repository to your workspace and build the code with catkin_make:
```bash
git clone https://github.com/ZJU-FAST-Lab/FACT-Coordinate-Initialization.git
cd FACT-Coordinate-Initialization
catkin_make -DMOSEK_DIR=$YOUR_HOME_DIR/mosek/10.1/tools/platform/linux64x86 # same directory as the one you installed Mosek
```
Most of the possible missing dependencies can be installed with apt-get. If you encounter any problems, please feel free to open an issue or contact us.

## Usage
We provide a launch file to run the simulation. You can run the simulation with the following command:
```bash
source $YOUR_WORKSPACE/devel/setup.bash
roslaunch ego_planner swarmtakeoff.launch
```
By default, the simulation will run with 6 drones using the SDP optimization method. If you want to test local optimization methods mentioned in the paper, you can change the launch instruction to:
```bash
roslaunch ego_planner swarmtakeoff.launch solver_type:=LM
```
for Levenberg-Marquardt method, or
```bash
roslaunch ego_planner swarmtakeoff.launch solver_type:=GaussNewton
```
for Gauss-Newton method.

The terminal will print the published goal positions and the calculated observations for all drones at each iteration. If enough observations are received, the terminal will print the solved relative rotations. The simulation will stop when all relative poses are solved.

By default, the results will be saved to the 'results' folder in the workspace directory. To evaluate the results, you can run the script 'process_data.py' in the 'results' folder:
```bash
cd $YOUR_WORKSPACE/results
python3 process_data.py
```
The script will print the MAE of relative rotations and the time used for each method.

If you want to visualize the simulation, you can run the following command in another terminal before or after running the simulation command above:
```bash
source $YOUR_WORKSPACE/devel/setup.bash
roslaunch ego_planner rviz.launch
```

## Known Issues
Certain situations may cause the planning or renderer to fail, printing huge amounts of error messages.

## Contact Us
Feel free to open an issue if you have any questions or suggestions. If you prefer to contact us directly, you can send an email to Anke Zhao at 1203302828@qq.com for deployment problems, or Yuan Li at yuanli_cse@zju.edu.cn for algorithm problems.
