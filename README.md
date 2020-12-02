# MPC_SharedControl with Joystick Teleoperation using MATLAB 
This branch includes codes for teleoperation of the robot using Joystick and controller running on MATLAB. Follow the steps below to install requirements and run the simulation with shared control or with manual teleoperation.

## Clone the repository with required branch to your root directory
Running the command below will clone the repository with the following name **"shared_control_python"**.
```
git clone --single-branch --branch Melodic_Joystick_SC_MATLAB https://github.com/teddy1496/MPC_SharedControl.git shared_control_matlab
```

## Install required packages and modules.
Navigate to the folder **"shared_control_matlab"** and run the **"install_requirements.sh"** script. This will check if required packages are installed and will install them if not already installed. You can run the script from within the folder with the following command.
```
cd shared_control_python

./install_requirements.sh
```
## Build the cloned workspace
Now that required dependencies are installed move navigate back to the directory **"shared_control_matlab"** and build the directory as a catkin workspace using the command below.
```
cd shared_control_python

catkin_make
```
## Running the controller
* We must now install the CasADi framework manually for matlab ubuntu. The download instruction and files can be found on their website
```
https://web.casadi.org/get/
```
* Downlaod and extract the files to your PC. 
* Edit the **"line 9"** of the script **"ROS_Obs_Avoid_Poles_MiR.m"** where the path should be the path of the casadi folder downloaded in your computer.

Now that all required steps are done the next step is to run the actual controller. 
* Before that make sure the Joystick is connected to the PC. If using Virtual Machine make sure the VM can see and access the Joystick.
* After the joystick has been connected you can either run the MiR gazebo simulation with direct teleoperation or with using the shared control teleoperation method.
* For direct teleoperation, run the **"launch_DirectTeleop.sh"** script on the root of the workspace.
```
./launch_DirectTeleop.sh
```
* For teleoperation with shared control, run the **"launch_SharedControl.sh"** script on the root of the workspace.
```
./launch_SharedControl.sh
```

* Once the script is run and gazebo and RVIZ are running, Run the script **"ROS_Obs_Avoid_Poles_MiR.m"** from MATLAB. This will start the MPC controller and initialize the robot position in gazebo as desired. 

## Gazebo Crashes!!!!
If gazebo crashes while using the Virtual Machine try running the following command in the terminal before running the script or launching gazebo manually.
```
export SVGA_VGPU10=0
```
