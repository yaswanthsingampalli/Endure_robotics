# Endure_robotics


pip install gunicorn
sudo apt install ros-humble-teleop-twist-keyboard
sudo apt install xterm
pip3 install --user flask
python3 -m pip install --user setuptools==59.6.0
sudo apt update
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-visualization-msgs
pip3 install pyproj
sudo apt updatete
sudo apt install ros-humble-airport-navigation-interfaces
sudo apt install ros-humble-rosbridge-server
sudo apt update
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo pip3 install gevent
pip install PyTurboJPEG
sudo apt update
sudo apt install python3-opencv

Delete the existing roslib.min.js file from the path and then try to install it again if incase you see any error:
cd ~/airport_navigation_ws/web_ui
wget https://raw.githubusercontent.com/RobotWebTools/roslibjs/master/build/roslib.min.js

echo "source ~/airport_navigation_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc

chmod +x start_server.sh
./start_server.sh

From any phone, tablet, or laptop on the same Wi-Fi or network, 
open a browser and go to: http://172.27.211.62:8081  --> it is jetson ip(hostname -I) and the port
