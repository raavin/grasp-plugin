#!/bin/sh
# source /opt/ros/electric/setup.bash
 export ROS_ROOT=/opt/ros/electric/ros
 export PATH=$ROS_ROOT/bin:$PATH
 export PYTHONPATH=$ROS_ROOT/core/roslib/src:$PYTHONPATH
 
 
 case "$0" in
/*) RRPATH=`dirname "${0}"` ;;
*) RRPATH=`dirname "$PWD/${0}"` ;;
esac

echo $RRPATH
export ROS_PACKAGE_PATH=${RRPATH}:/opt/ros/electric/stacks 

echo "$0"
echo "$1"
echo "$PWD"