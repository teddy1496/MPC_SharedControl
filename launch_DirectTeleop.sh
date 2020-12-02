cd ~/shared_control_python_KB/
source devel/setup.bash
cd launch/

export SVGA_VGPU10=0

gnome-terminal -- roslaunch MirPoles_World_DT.launch &
sleep 8

rosservice call /gazebo/unpause_physics

rosservice call /gazebo/set_model_state '{model_state: { model_name: mir, pose: { position: { x: -4, y: -4 ,z: 0 }, orientation: {x: 0, y: 0, z: 0, w: 1 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'


gnome-terminal -- rosrun teleop_twist_keyboard teleop_twist_keyboard_DT.py
