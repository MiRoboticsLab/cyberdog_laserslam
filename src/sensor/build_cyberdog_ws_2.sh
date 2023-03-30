#!/bin/bash
if [ x$1 == x ];then
	SHELL_FOLDER=$(cd "$(dirname "$0")";pwd)
elif [ $1 == . ];then
	SHELL_FOLDER=$(cd "$(dirname "$0")";pwd)
elif [ $1 == .. ];then
	SHELL_FOLDER=$(cd "$(dirname "$0")";pwd)
	SHELL_FOLDER=$(dirname $SHELL_FOLDER)
else
	SHELL_FOLDER=$1
	FINAL=${SHELL_FOLDER: -1}
	if [ $FINAL == / ];then
		SHELL_FOLDER=${SHELL_FOLDER:0:${#SHELL_FOLDER}-1}
	fi
fi


# sudo docker run --privileged=true -it --rm -v /home/quan/workspace/company/mi/sources/cyberdog/arm_docker_build:/home/quan/workspace/company/mi/sources/cyberdog/arm_install_extend micr.cloud.mioffice.cn/cyberdog/carpo_arm64:1.0

# navigation_core
#docker run --privileged=true -it --rm -v "$SHELL_FOLDER/arm_build_extend:$SHELL_FOLDER/build" -v "$SHELL_FOLDER/arm_install_extend:/opt/ros2/cyberdog" -v "$SHELL_FOLDER/src:$SHELL_FOLDER/src" --env SHELL_FOLDER=$SHELL_FOLDER micr.cloud.mioffice.cn/cyberdog/carpo_arm64:1.0 bash -c "cd $SHELL_FOLDER && source /opt/ros2/galactic/local_setup.bash && colcon build --merge-install --install-base /opt/ros2/cyberdog --parallel-workers 10 --packages-up-to cyberdog_gridmap_costmap_plugin lidar_controller map_label_server navigation_bringup navigation_interfaces odom_controller positionchecker realsense_controller velocity_adaptor"

# laser_slam cyberdog_visions_interfaces cyberdog_gridmap_costmap_plugin lidar_controller map_label_server navigation_bringup navigation_interfaces odom_controller positionchecker realsense_controller velocity_adaptor

docker run --privileged=true -it --rm -v "$SHELL_FOLDER/arm_build_extend:$SHELL_FOLDER/build" -v "$SHELL_FOLDER/arm_install_extend:/opt/ros2/cyberdog" -v "$SHELL_FOLDER/src:$SHELL_FOLDER/src" --env SHELL_FOLDER=$SHELL_FOLDER micr.cloud.mioffice.cn/cyberdog/carpo_arm64:1.0 bash -c "cd $SHELL_FOLDER && source /opt/ros2/galactic/local_setup.bash && colcon build --merge-install --install-base /opt/ros2/cyberdog --parallel-workers 10 --packages-up-to navigation2"


