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




colcon build --packages-select vrpn_urdf
ros2 launch vrpn_urdf vrpn_topic_test.launch.py
ros2 launch vrpn_urdf vrpn_g1_controller.launch.py

source ~/unitree_ros2/setup_local.sh # Use the local network card
export ROS_DOMAIN_ID=1 # Modify the domain id to match the simulation
./install/stand_go2/bin/stand_go2 # Run