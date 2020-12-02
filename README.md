# MPC_SharedControl with Keyboard Teleoperation
This branch includes codes for teleoperation of the robot using Keyboard. Follow the steps below to install requirements and run the simulation with shared control or with manual teleoperation.

## Clone the repository with required branch to your root directory
Running the command below will clone the repository with the following name **"shared_control_python"**.
```
git clone --single-branch --branch Melodic_Keyboard_SC https://github.com/teddy1496/MPC_SharedControl.git shared_control_python_KB
```

## Install required packages and modules.
Navigate to the folder **"shared_control_pyhton_KB"** and run the **"install_requirements.sh"** script. This will check if required packages are installed and will install them if not already installed. You can run the script from within the folder with the following command.
```
cd shared_control_python_KB

./install_requirements.sh
```
## Build the cloned workspace
Now that required dependencies are installed move navigate back to the directory **"shared_control_python_KB"** and build the directory as a catkin workspace using the command below.
```
cd shared_control_python_KB

catkin_make
```
## Running the controller
Now that all required steps are done the next step is to run the actual controller. 
* You can either run the MiR gazebo simulation with direct teleoperation or with using the shared control teleoperation method.

**The terminal which launches the Keyboard node must be selected in order for the keyboard teleoperation must work.**

* For direct teleoperation, run the **"launch_DirectTeleop.sh"** script on the root of the workspace.
```
./launch_DirectTeleop.sh
```
* For teleoperation with shared control, run the **"launch_SharedControl.sh"** script on the root of the workspace.
```
./launch_SharedControl.sh
```
## Gazebo Crashes!!!!
If gazebo crashes while using the Virtual Machine try running the following command in the terminal before running the script or launching gazebo manually.
```
export SVGA_VGPU10=0
```
