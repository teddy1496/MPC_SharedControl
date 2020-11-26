cd ~/shared_control_python/
source devel/setup.bash
cd launch/

export SVGA_VGPU10=0

gnome-terminal -- roslaunch MirPoles_World_SC.launch &
sleep 8

rosservice call /gazebo/unpause_physics

sleep 3
roslaunch joystick_control python_MPC_controller.launch
