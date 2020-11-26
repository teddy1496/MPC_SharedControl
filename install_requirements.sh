sudo apt-get update 
sudo apt-get --yes upgrade

REQUIRED_PKG="ros-melodic-desktop-full"
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG|grep "install ok installed")
echo Checking for $REQUIRED_PKG: $PKG_OK
if [ "" = "$PKG_OK" ]; then
  echo "No $REQUIRED_PKG. Setting up $REQUIRED_PKG."
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
	sudo apt-get update
  sudo apt-get --yes install $REQUIRED_PKG
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc 
	source ~/.bashrc
	sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
	sudo apt install python-rosdep
	sudo rosdep init
	rosdep update
fi

REQUIRED_PKG_1="python-catkin-tools"
PKG_OK_1=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG_1|grep "install ok installed")
echo Checking for $REQUIRED_PKG_1: $PKG_OK_1
if [ "" = "$PKG_OK_1" ]; then
  echo "No $REQUIRED_PKG_1. Setting up $REQUIRED_PKG_1."
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
	wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
  sudo apt-get --yes install $REQUIRED_PKG_1 
fi

REQUIRED_PKG_2="ros-melodic-mir-robot"
PKG_OK_2=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG_2|grep "install ok installed")
echo Checking for $REQUIRED_PKG_2: $PKG_OK_2
if [ "" = "$PKG_OK_2" ]; then
  echo "No $REQUIRED_PKG_2. Setting up $REQUIRED_PKG_2."
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
	sudo apt-get update -qq
  sudo apt-get --yes install $REQUIRED_PKG_2
fi

REQUIRED_PKG_3="python-pip"
PKG_OK_3=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG_3|grep "install ok installed")
echo Checking for $REQUIRED_PKG_3: $PKG_OK_3
if [ "" = "$PKG_OK_3" ]; then
  echo "No $REQUIRED_PKG_3. Setting up $REQUIRED_PKG_3."
  sudo apt-get --yes install $REQUIRED_PKG_3
fi

REQUIRED_PKG_4="ros-melodic-joy"
PKG_OK_4=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG_4|grep "install ok installed")
echo Checking for $REQUIRED_PKG_4: $PKG_OK_4
if [ "" = "$PKG_OK_4" ]; then
  echo "No $REQUIRED_PKG_4. Setting up $REQUIRED_PKG_4."
  sudo apt-get --yes install $REQUIRED_PKG_4
fi

REQUIRED_PKG_5="ros-melodic-navigation"
PKG_OK_5=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG_5|grep "install ok installed")
echo Checking for $REQUIRED_PKG_5: $PKG_OK_5
if [ "" = "$PKG_OK_5" ]; then
  echo "No $REQUIRED_PKG_5. Setting up $REQUIRED_PKG_5."
  sudo apt-get --yes install $REQUIRED_PKG_5
fi



pip install casadi
pip install numpy

cd ~

mkdir shared_control_python
cd shared_control_python
git clone --single-branch --branch Melodic_Joystick_SC https://github.com/teddy1496/MPC_SharedControl.git
