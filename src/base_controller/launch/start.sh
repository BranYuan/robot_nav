#!/bin/bash

sudo -S chmod 777 /dev/ttyS1 << EOF
1
EOF

sudo -S chmod 777 /dev/ttyS2 << EOF
1
EOF

sudo -S chmod 777 /dev/ttyS3 << EOF
1
EOF

sudo -S chmod 777 /dev/ttyS4 << EOF
1
EOF

sudo -S chmod 777 /dev/ttyS5 << EOF
1
EOF

source /opt/ros/melodic/setup.bash
source /home/sweet/github_store/robot_nav/devel/setup.bash


#echo "this is a test" >> /home/sweet/github_store/robot_nav/src/base_controller/launch/text.log

roslaunch /home/sweet/github_store/robot_nav/src/base_controller/launch/base_controller.launch


exit 0
