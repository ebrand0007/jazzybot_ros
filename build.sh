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
