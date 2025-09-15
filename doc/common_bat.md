# vrpn_mocap
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch vrpn_mocap client.launch.yaml server:=192.168.1.7 port:=3883 multi_sensor:=true
colcon build --packages-select vrpn_mocap

source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 topic list | grep vrpn_mocap
ros2 topic echo /vrpn_mocap/MCServer/pose300
ros2 param get /vrpn_mocap/vrpn_mocap_client_node multi_sensor


# motion_robot
source ~/dev/unitree/unitree_ros2/setup.sh 
source install/setup.bash
export ROS_DOMAIN_ID=0
./install/motion_robot/bin/vrpn_test_node