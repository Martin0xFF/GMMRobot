# GMMRobot
The Galbraith Memorial Mail Robot is a project containing several components for a simple delievery bot simulated in ROS Gazebo.

# Files
- monke.py is a ros node which provides the basic linefollowing and control algorithm
- probablistic_neuron_activation provides the bayesian localization algorithm
- linedetect.py is provides filtering from camera information for the bot
- lab01_odomotor.py provides some basic interfacing logic for controlling the robot
- our_final_project.py combines the various files above into one node which executes the commands

Note that the files require a ros gazebo environment, as well as a simulator for the turtlebot waffle pi with odometry and camera sensing enabled. This code was developed for a simulated version of this bot.
