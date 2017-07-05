roslaunch rtabmap_ros visodomlong.launch &
sleep 5
#rosservice call /rtabmap/rgbd_odometry/set_logger_level ros WARN
echo printing suppressed
source ~/Desktop/topicforwarding/devel/setup.bash
rosrun odomtopose forwardotp
