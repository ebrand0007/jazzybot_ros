#!/bin/sh
SOURCE=`pwd`/src
DEST=/tmp/jazzybot_ws/.

mkdir -p $DEST
rsync -av $SOURCE* $DEST/.
cd $DEST

. /opt/ros/indigo/setup.bash
catkin_make clean
catkin_make


rm -rf install
catkin_make install

echo "To run the code:"
echo "  . install/setup.bash"
echo "  rosrun jazzybot jbot_base2_node"
echo
echo "Return to source by running:"
echo "  cd $SOURCE"
