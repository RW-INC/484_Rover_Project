<h2>Run and drive simulation (edited for gazebo fortress)</h2>
BEFORE RUNNING ANYTHING FOLLOW DIRECTIONS <br>
inside simulation_ws <br>

If you pulled from git, DELETE build, install, and log folders before continuing

	rm -rf build install log

Now you may build and source your package again

	colcon build rover_sim
	source install/setup.bash

Open 3 terminals (tmux recommended) <br>
in one terminal run:

	ros2 launch rover_sim sim.launch.py

in the other run:

	ros2 run ros_gz_bridge parameter_bridge \
	/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist

in the last run (change linear and angular as needed)

	ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
	linear:
	x: 0.5
	angular:
	z: 0.3
	"

to key the robot around run instead:

	ros2 run teleop_twist_keyboard teleop_twist_keyboard

<h2>Visualizing Point Cloud Using Lidar Plugin</h2>

You can see the LiDAR data in a few ways <br>
In a seperate, sourced terminal run: <br> <br>
Option 1: Echo topic

	ros2 topic echo /lidar

Option 2: Use Rviz

	rviz2

<h2>Image Editing for Heightmaps</h2>

1. Using Imagemagick: https://imagemagick.org/script/command-line-processing.php#gsc.tab=0 
2. HEIGHTMAPS RENDERED MUST TAKE THE SIZE OF 2^n+1 x 2^n+1

<br>
reformat images to be viable sizes:

	# This forces it to 513x513 and Grayscale
	convert heightmap_fixed.png -resize 513x513! -colorspace gray heightmap_final.png

and smooth gradients so it's not blocky:

	convert heightmap_fixed.png -blur 0x8 heightmap_smooth.png

<h2>Debugging</h2>

source the setup files for ros if run/launch commands not working

	source /opt/ros/humble/setup.bash	# source the setup file for ros2 if cmds not found

checking topics and controllers: 

	ros2 topic list					# check active topics
	ros2 topic echo /cmd_vel		# check velocity commands published
	ros2 topic echo /odom			# echos odometry
	ros2 control list_controllers	# lists active ros2 controllers

if spawn msg not sending, you can manually spawn the rover in:

	ros2 run ros_gz_sim create \
  	-file rover.urdf \
  	-name rover

<h4>DO NOT RUN FOLLOWING ON GAZEBO FORTRESS </h4>
for classic gazebo debugging only
<br> <br>
if for some reason gazebo doesn't fully shut down, the client won't relaunch until it does. Run:

	killall -9 gazebo & killall -9 gzserver & killall -9 gzclient	# kills all gazebo processes

if classic gazebo starts throwing hands this clears the keyrings: 

	sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg # fixes gpg key errors


