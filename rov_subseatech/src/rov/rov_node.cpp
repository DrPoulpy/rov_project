#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "rov_subseatech/robot_data.h"
#include "rov_subseatech/robot_cmd.h"

#include "rov_param.h"


/** a structure to store the vertical speed and the horizontal linear and angular speed of the rov **/
struct speed_vector{
    float vertical;
    float horizontal;
    float angular;
};

/** a structure to store the raw speed of the  different rov motors **/
struct motor_vector{
    float vertical_motor;
    float left_motor;
    float right_motor;
};

rov_subseatech::robot_cmd joy_cmd; //a global variable to store the data received by the subscriber

/** the callback method to get inputs from station **/
void chatterCallback(const rov_subseatech::robot_cmd::ConstPtr& msg)
{
    joy_cmd.longitudinal_value = msg->longitudinal_value;
    joy_cmd.vertical_value = msg->vertical_value;
    joy_cmd.rotational_value = msg->rotational_value;
    joy_cmd.gain_value = msg->gain_value;

}
/** a method to get the vertical linear speed and the horizontal linear and angular speed command from joystick input **/
struct speed_vector joyToSpeed(const rov_subseatech::robot_cmd joy)
{
    struct speed_vector temp;

    float gain = (float)joy.gain_value/100.0;

    temp.angular = gain*MAX_ANGULAR_SPEED*((float)joy.longitudinal_value/100.0);
    temp.vertical = gain*MAX_LINEAR_SPEED*((float)joy.rotational_value/100.0);
    temp.horizontal = gain*MAX_LINEAR_SPEED*((float)joy.vertical_value/100.0);

    return(temp);
}

/** a method that give motors speed from desired angular and linear speed for a rov with a diferential system **/
struct motor_vector kinematicModel(const struct speed_vector cmd)
{
    struct motor_vector temp_motor;

    //vertical motor speed
    temp_motor.vertical_motor = cmd.vertical*R_VERTICAL_MOTOR;

    //Horizontal motors speeds
    temp_motor.left_motor = R_HORIZONTAL_LEFT_MOTOR*(2*cmd.horizontal + cmd.angular*LEFT_MOTOR_DISTANCE)/2;
    temp_motor.right_motor = R_HORIZONTAL_RIGHT_MOTOR*(2*cmd.horizontal - cmd.angular*RIGHT_MOTOR_DISTANCE)/2;

    return(temp_motor);
}

/** a method that check if the raw motor speed is compatible to specs and convert it in percent **/
rov_subseatech::robot_data rawSpeedToPercent(struct motor_vector raw)
{
    rov_subseatech::robot_data temp;

    /** safety check of motor speed **/
    //left motor
    if(raw.left_motor > HORIZONTAL_LEFT_MOTOR_MAX_SPEED)
    {
        raw.left_motor = HORIZONTAL_LEFT_MOTOR_MAX_SPEED;
        ROS_INFO("left motor speed limited for safety");
    }
    else if(raw.left_motor < -HORIZONTAL_LEFT_MOTOR_MAX_SPEED)
    {
        raw.left_motor = -HORIZONTAL_LEFT_MOTOR_MAX_SPEED;
        ROS_INFO("left motor speed limited for safety");
    }
    //right motor
    if(raw.right_motor > HORIZONTAL_RIGHT_MOTOR_MAX_SPEED)
    {
        raw.right_motor = HORIZONTAL_RIGHT_MOTOR_MAX_SPEED;
        ROS_INFO("right motor speed limited for safety");
    }
    else if(raw.right_motor < -HORIZONTAL_RIGHT_MOTOR_MAX_SPEED)
    {
        raw.right_motor = -HORIZONTAL_RIGHT_MOTOR_MAX_SPEED;
        ROS_INFO("right motor speed limited for safety");
    }
    //vertical motor
    if(raw.vertical_motor > VERTICAL_MOTOR_MAX_SPEED)
    {
        raw.vertical_motor = VERTICAL_MOTOR_MAX_SPEED;
        ROS_INFO("vertical motor speed limited for safety");
    }
    else if(raw.vertical_motor < -VERTICAL_MOTOR_MAX_SPEED)
    {
        raw.vertical_motor = -VERTICAL_MOTOR_MAX_SPEED;
        ROS_INFO("vertical motor speed limited for safety");
    }
    /** conversion to percent **/
    temp.left_motor = 100*raw.left_motor/HORIZONTAL_LEFT_MOTOR_MAX_SPEED;
    temp.right_motor = 100*raw.right_motor/HORIZONTAL_RIGHT_MOTOR_MAX_SPEED;
    temp.vertical_motor = 100*raw.vertical_motor/VERTICAL_MOTOR_MAX_SPEED;

    return(temp);
}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "rov");

    ros::NodeHandle n;

    /** Node communication **/
    ros::Publisher motor_value = n.advertise<rov_subseatech::robot_data>("rov_feedback", 1000);
    ros::Subscriber speed_input = n.subscribe("station_order", 1000, chatterCallback);


    /** node initialisation **/
    ros::Rate loop_rate(FRAMERATE);

    struct speed_vector cmd;
    struct motor_vector  raw_motor_speed;
    rov_subseatech::robot_data percent_motor_speed;

    while (ros::ok())
    {
        ros::spinOnce();

        /** display input from station **/
        ROS_INFO("joystick input from station : ");
        ROS_INFO("--- longitudinal axis : %d ", joy_cmd.longitudinal_value);
        ROS_INFO("--- vertical axis : %d ", joy_cmd.vertical_value);
        ROS_INFO("--- rotational axis : %d ", joy_cmd.rotational_value);
        ROS_INFO("--- gain : %d ", joy_cmd.gain_value);

        //compute speed order from station input **/
        cmd = joyToSpeed(joy_cmd);
        ROS_INFO("speed order:") ;
        ROS_INFO("--- angular speed : %f rad/s", cmd.angular);
        ROS_INFO("--- horizontal speed : %f m/s", cmd.horizontal);
        ROS_INFO("--- vertical motor : %f m/s", cmd.vertical);

        /** compute motor speed from speed order **/
        raw_motor_speed = kinematicModel(cmd);
        ROS_INFO("raw motor speed:");
        ROS_INFO("--- left motor : %f RPM", raw_motor_speed.left_motor);
        ROS_INFO("--- right motor : %f RPM", raw_motor_speed.right_motor);
        ROS_INFO("--- vertical motor : %f RPM", raw_motor_speed.vertical_motor);

        /** convert motor speed in percent **/
        percent_motor_speed = rawSpeedToPercent(raw_motor_speed);
        ROS_INFO("percentage motor speed:");
        ROS_INFO("--- left motor : %d percent", (signed int)percent_motor_speed.left_motor);
        ROS_INFO("--- right motor : %d percent", (signed int)percent_motor_speed.right_motor);
        ROS_INFO("--- vertical motor : %d percent", (signed int)percent_motor_speed.vertical_motor);

        /** send motor speed to station **/
        motor_value.publish(percent_motor_speed);

        loop_rate.sleep();

    }

    return 0;
}


