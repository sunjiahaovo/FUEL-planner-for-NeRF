sudo chmod 777 /dev/ttyACM0 & sleep 2;
# roslaunch vrpn_client_ros sample.launch server:=192.168.50.150 & sleep 2;
#roslaunch realsense2_camera rs_camera.launch & sleep 12;
roslaunch realsense2_camera rs_camera.launch camera:=camera serial_no:= & sleep 5;
roslaunch realsense2_camera rs_t265.launch camera:=cam_2 serial_no:= & sleep 5;
# roslaunch mavros px4.launch & sleep 10;
# roslaunch vins fast_drone_250.launch
wait;
