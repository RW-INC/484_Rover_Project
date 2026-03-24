Run and drive simulation (edited for gazebo fortress)

1. open 3 terminals (tmux recommended)
2. in one terminal run 

	ros2 launch rover_sim sim.launch.py

3. in the other run

	ros2 run ros_gz_bridge parameter_bridge \
	/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist

4. in the last run (change linear and angular as needed)

	ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
	linear:
	x: 0.5
	angular:
	z: 0.3
	"

5. to key the robot around run instead of step 4:

	ros2 run teleop_twist_keyboard teleop_twist_keyboard

follow the instructions to key it around

Image Editing for Heightmaps
- Using Imagemagick: https://imagemagick.org/script/command-line-processing.php#gsc.tab=0 
- HEIGHTMAPS RENDERED MUST TAKE THE SIZE OF 2^n+1

Debug
potentially useful cmds:

source /opt/ros/humble/setup.bash	# source the setup file for ros2 if cmds not found

ros2 topic list					# check active topics
ros2 topic echo /cmd_vel		# check velocity commands published
ros2 topic echo /odom			# echos odometry
ros2 control list_controllers	# lists active ros2 controllers

dont't run this on fortress
killall -9 gazebo & killall -9 gzserver & killall -9 gzclient	# kills all gazebo processes

if classic gazebo starts throwing hands this clears the keyrings: 
(don't run on fortress)
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg # fixes gpg key errors

if spawn msg not sending
manually spawn the rover in:

	ros2 run ros_gz_sim create \
  	-file rover.urdf \
  	-name rover