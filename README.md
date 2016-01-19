# rostesting
Here are the steps for running the ROS PSoC nodes for drive and arm control

[1] Drive Control
note the IP of the Beaglebone Black (BBB)
do the following on the BBB
run: roscore
run: cd /git/rostesting/rover_ws
run: source devel/setup.bash
run: rosrun psoc_driver psoc_driver_node
run: rosrun drive_teleop drive_teleop_node

On the slave Linux machine
run: export ROS_MASTER_URI=http://XXX.XXX.XXX.XXX:11311 (XXX are IP)
(make sure that the /etc/hosts file has a hostname called ubuntu with 
the aforementioned IP, otherwise you will not be able to see the BBB,
also, if trouble still is happening, make sure that the /etc/hosts 
file on the BBB has the name of the slave Linux machine with its
corresponding ip)
run: rosparam set joy_node/dev "/dev/input/jsX" (X is the port,
default is 0, google "ROS joy" and find configuring a joystick for help)
run: rosrun joy joy_node

[2]Arm Control
All the steps are the same, except for the rosrun commands on BBB

run: rosrun psoc_arm psoc_arm_node
run: rosrun arm_teleop arm_teleop_node
