DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
roslaunch $DIR/visodomlong.launch &
sleep 5
#rosservice call /rtabmap/rgbd_odometry/set_logger_level ros WARN
echo printing suppressed
source $DIR/topicforwarding/devel/setup.bash
rosrun odomtopose forwardotp
