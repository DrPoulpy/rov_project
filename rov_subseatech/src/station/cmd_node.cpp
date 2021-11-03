#include "ros/ros.h"
#include "std_msgs/String.h"
#include <termios.h>
#include "rov_subseatech/robot_data.h"
#include "rov_subseatech/robot_cmd.h"
#include "cmd_param.h"

rov_subseatech::robot_data rov_data; //a global variable to store the data received by the subscriber

/** A code I took online to get key inputs without waiting needing to press ENTER
https://github.com/sdipendra/ros-projects/blob/master/src/keyboard_non_blocking_input/src/keyboard_non_blocking_input_node.cpp
**/
char getch()
{
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if(rv == -1)
        ROS_ERROR("select");
    else if(rv == 0)
        ROS_INFO("no_key_pressed");
    else
        read(filedesc, &buff, len );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR ("tcsetattr ~ICANON");
    return (buff);
}
/** a method that get user input key and update joystick and gain value accordingly **/
void updateJoyState( rov_subseatech::robot_cmd &msg)
{
    int c = 0;
    c=getch();

    switch(c)
    {
    case JOY_ROTATE_LEFT :
    {
        msg.rotational_value += JOY_INCREMENT;
        if( msg.rotational_value > 100 )
            msg.rotational_value = 100;
        break;
    }
    case JOY_ROTATE_RIGHT :
    {
        msg.rotational_value -= JOY_INCREMENT;
        if( msg.rotational_value < -100 )
            msg.rotational_value = -100;
        break;
    }
    case JOY_LEFT :
    {
        msg.longitudinal_value += JOY_INCREMENT;
        if( msg.longitudinal_value > 100 )
            msg.longitudinal_value = 100;
        break;
    }
    case JOY_RIGHT :
        msg.longitudinal_value -= JOY_INCREMENT;
        if( msg.longitudinal_value < -100 )
            msg.longitudinal_value = -100;
        break;
    case JOY_UP :
    {
        msg.vertical_value += JOY_INCREMENT;
        if( msg.vertical_value > 100 )
            msg.vertical_value = 100;
        break;
    }
    case JOY_DOWN :
    {
        msg.vertical_value -= JOY_INCREMENT;
        if( msg.vertical_value < -100 )
            msg.vertical_value = -100;
        break;
    }
    case JOY_RESET :
    {
        msg.longitudinal_value = 0;
        msg.vertical_value = 0;
        msg.rotational_value = 0;
        break;
    }
    case GAIN_UP :
    {
        msg.gain_value += GAIN_INCREMENT;
        if( msg.gain_value > 100 )
            msg.gain_value = 100;
        break;
    }
    case GAIN_DOWN :
    {
        msg.gain_value -= GAIN_INCREMENT;
        if( msg.gain_value < 0 )
            msg.gain_value = 0;
        break;
    }
    }
}

void displayStationState(const rov_subseatech::robot_data motor_msg,const rov_subseatech::robot_cmd joy_cmd)
{
    /** display input **/
    ROS_INFO("joystick input from station : ");
    ROS_INFO("--- longitudinal axis : %d ", joy_cmd.longitudinal_value);
    ROS_INFO("--- vertical axis : %d ", joy_cmd.vertical_value);
    ROS_INFO("--- rotational axis : %d ", joy_cmd.rotational_value);
    ROS_INFO("--- gain : %d ", joy_cmd.gain_value);

    /** display motor state **/
    ROS_INFO("percentage motor speed:");
    ROS_INFO("--- left motor : %d percent", (signed int)motor_msg.left_motor);
    ROS_INFO("--- right motor : %d percent", (signed int)motor_msg.right_motor);
    ROS_INFO("--- vertical motor : %d percent", (signed int)motor_msg.vertical_motor);

}

void chatterCallback(const rov_subseatech::robot_data::ConstPtr& msg)
{
    rov_data.left_motor = msg->left_motor;
    rov_data.right_motor = msg->right_motor;
    rov_data.vertical_motor = msg->vertical_motor;
}


int main(int argc, char **argv)
{
  
	ros::init(argc, argv, "station");


	ros::NodeHandle n;

        /** Node communication **/
        ros::Subscriber rov_feedback = n.subscribe("rov_feedback", 1000, chatterCallback);
        ros::Publisher cmd_order = n.advertise<rov_subseatech::robot_cmd>("station_order", 1000);
  
        ros::Rate loop_rate(FRAMERATE);

        /** initialisation **/
        rov_subseatech::robot_cmd joy_order;

        joy_order.longitudinal_value = 0;
        joy_order.vertical_value = 0;
        joy_order.rotational_value = 0;
        joy_order.gain_value = 50;

	while (ros::ok())
	{

            ros::spinOnce();

            /** update joystick and gain with user input **/
            updateJoyState(joy_order);

            /** display  station order and rov feedback **/
            displayStationState(rov_data, joy_order);

            /** send station order to rov **/
            cmd_order.publish(joy_order);

            loop_rate.sleep();

  	}

	return 0;
}
