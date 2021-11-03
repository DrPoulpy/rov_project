# rov_subseatech package
##installation
 go to  the src folder in your catkin workspace 
 on a terminal, import the package with the command :
 `git clone `
 get back to your catkin workspace
 on a terminal, compile the ros package with the command :
 `catkin_make`
 enter the following command :
 `source devel/setup.bash`
 This command allow ROS to see the package
 ##use
 in a terminal, run the package with the following command :
 `roslaunch rov_subseatech rov_simulation.launch `
 this will launch a rosmaster node, a rov node,a station node and a terminal linked to the station node.
 the terminal display the joystick, the gain configuration and the 3 motors speed computed by the rov
 by default, * use **z** and **s** to control the vertical axis * use **q** and **d** to control the longitudinal axis * use **a** and **e** to control the rotational axis * use **r** to reset joystick position * use **p** and **m** to control the gain
 these parameters can be changed in rov_subseatech/src/station/cmd_param.h
  you can run the simulation in debug with a second monitor that display what happen in the rov node by launching the simulation with the following command: 
  `roslaunch rov_subseatech rov_simulation.launch `