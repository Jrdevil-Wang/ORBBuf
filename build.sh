#!/bin/bash

# change this path to your ros path
export TARGET_ROS_ROOT=/opt/ros/kinetic

export TARGET_INCLUDE_DIR=$TARGET_ROS_ROOT/include
export TARGET_LIB_DIR=$TARGET_ROS_ROOT/lib
export TARGET_SHARE_DIR=$TARGET_ROS_ROOT/share

# compile ros_comm
cd ros_comm_ws
catkin_make
. ./install.sh

# compile image_transport
cd ../image_transport_ws
catkin_make
. ./install.sh

# compile image_plugins
cd ../plugins_ws
catkin_make
. ./install.sh

# compile image_content
cd ../image_content
catkin_make
. ./install.sh

# compile test program
cd ../test_pub
catkin_make
cp ./devel/lib/image_pub/stereo_kitti_pub ..

echo "Build Finish. You can use 'stereo_kitti_pub' and 'rostopic echo' to do simple test."
