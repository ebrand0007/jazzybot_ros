#jbot

Rewrite of linobot below

Linorobot is an Open Source robotic project that aims to provide students, developers, and researchers a low-cost platform in creating new exciting applications on top of ROS.

http://linorobot.org

building:

mkdir catkin_ws
cd catkin_ws
catkin_init_workspace


#!/bin/sh
SOURCE=/tmp/jbot/jazzybot_ros/
DEST=src/jazzybot/.

rsync -av $SOURCE* $DEST/.
catkin_make clean

catkin_make


rm -rf install
catkin_make install

echo "To run the code:"
echo "  . install/setup.bash"
echo "  rosrun jazzybot jbot_base2_node"

