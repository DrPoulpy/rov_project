# rov_subseatech package

## installation
   Go to  the src folder in your catkin workspace  
   On a terminal, import the package with the command :\
   `git clone https://github.com/DrPoulpy/rov_project.git `\
   Get back to your catkin workspace\
   On a terminal, compile the ros package with the command :\
   `catkin_make`\
   Enter the following command :\
   `source devel/setup.bash`\
   This command allow ROS to see the package\
   ## use
 In a terminal, run the package with the following command :\
 `roslaunch rov_subseatech rov_simulation.launch `\
 This will launch a rosmaster node, a rov node,a station node and a terminal linked to the station node\
 The terminal display the joystick, the gain configuration and the 3 motors speed computed by the rov\
 by default, 
 * use **z** and **s** to control the vertical axis 
 * use **q** and **d** to control the longitudinal axis 
 * use **a** and **e** to control the rotational axis 
 * use **r** to reset joystick position 
 * use **p** and **m** to control the gain 
 
 These parameters can be changed in ***rov_subseatech/src/station/cmd_param.h*** \
 You can also change parameters of the rov from ***rov_subseatech/src/rov/rov_param.h*** \
 You can run the simulation in debug with a second monitor that display what happen in the rov node by launching the simulation with the following command: \
  `roslaunch rov_subseatech debug_rov_simulation.launch `
