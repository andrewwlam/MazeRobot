#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

#include <stdio.h>
#include <cmath>
#include <chrono>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <unistd.h>
int usleep(useconds_t usec);
int currentTime;
int exitTime;

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad)*180. / M_PI)
#define DEG2RAD(deg) ((deg)*M_PI / 180.)

float posX = 0.0, posY = 0.0, yaw = 0.0;
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

uint8_t leftState = 0, rightState = 0, centerState = 0;

float minLaserDist = std::numeric_limits<float>::infinity();
// Change angle to change field of view
//Currently 115 degree FOW
int32_t nLasers = 0, desiredNLasers = 0, desiredAngle = 120;

int state = 1;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg)
{
    //fill with your code
    // Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
    leftState = bumper[kobuki_msgs::BumperEvent::LEFT];
    rightState = bumper[kobuki_msgs::BumperEvent::RIGHT];
    centerState = bumper[kobuki_msgs::BumperEvent::CENTER];
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    //nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    //desiredNLasers = DEG2RAD(desiredAngle)/msg->angle_increment;

    minLaserDist = std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = desiredAngle * M_PI / (180 * msg->angle_increment);
    //ROS_INFO("Angle of laser scan array: %i and size of offset: %i", (msg->angle_max - msg->angle_min), desiredNLasers);

    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min)
    {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx)
        {
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }
    else
    {
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx)
        {
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("Position:(%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Subscriber odom = nh.subscribe("odom", 1, odomCallback);
    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    float angular = 0.0;
    float linear = 0.0;
    float desiredYaw = 90;
    while (ros::ok() && secondsElapsed <= 480)
    {
        ros::spinOnce();
        //ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, yaw*180/pi, maxLaserRange);
        //
        // Check if any of the bumpers were pressed.
        bool any_bumper_pressed = false;
        
        //If left, centre or right bumper is pressed combine into any_bumper_pressed
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx)
        {
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
            
        }
        
        // Control logic after bumpers are being pressed.
        ROS_INFO("Position: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);

        //If there are no obstacles go straight
        if (!any_bumper_pressed && minLaserDist > 0.7 && state < 80)
        {
            
            angular = -M_PI/40;
            linear = 0.175;
            ROS_INFO("MOVING FWD");
        }
        if (!any_bumper_pressed && minLaserDist > 0.7 && state >= 80)
        {
            
            angular = M_PI/40;
            linear = 0.175;
            ROS_INFO("MOVING FWD");
        }
        if (!any_bumper_pressed && minLaserDist > 0.5 && minLaserDist < 0.7)
        {
            angular = 0.0;
            linear = 0.10;
            ROS_INFO("MOVING FWD SLOWLY");
        }

        //Turn left at an obstacle if the state is 1
        if (!any_bumper_pressed && minLaserDist < 0.5 && state < 80)
        {
            ros::Time start_time1 = ros::Time::now();
            ros::Duration timeout1(0.2); // Timeout of 3 seconds
            while (ros::Time::now() - start_time1 < timeout1)
            {
                angular = M_PI/3;
                linear = 0;

                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);

                ROS_INFO("TURN LEFT");
                ROS_INFO ("STATE IS %i", state);
            }
            state ++;
        }
        //Turn right at an obstacle when the state is 2
        else if (!any_bumper_pressed &&minLaserDist < 0.5 && state >= 80)
        {
            ros::Time start_time2 = ros::Time::now();
            ros::Duration timeout2(0.2); // Timeout of 2 seconds
            while (ros::Time::now() - start_time2 < timeout2)
            {
                angular = -M_PI/3;
                linear = 0;

                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);

                ROS_INFO("TURN RIGHT");
                 ROS_INFO ("STATE IS %i", state);
            }
           
            state++;
        }
        
        //Spin the robot periodically
        if (state == 20 || state == 60 || state == 90 || state == 120)
        { 
            ros::Time start_time3 = ros::Time::now();
            ros::Duration timeout3(10.10); 
            while (ros::Time::now() - start_time3 < timeout3)
            {
                angular = M_PI/4;
                linear = 0;
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
        
            }
           state++;      
        ROS_INFO("WhhEEEEEEE");
        }
        if (state == 140)
        { 
            state = 1;
        }

        // //If any bumper is pressed, go backwards 0.3 metres and turn 90 degrees left
        // if (any_bumper_pressed)
        // {
        //     //going backwards 0.5 metres
        //     ros::Time start_time3 = ros::Time::now();
        //     ros::Duration timeout3(3.5); // Timeout of 1.5 seconds
        //     while (ros::Time::now() - start_time3 < timeout3)
        //     {
        //         angular = -M_PI/6;
        //         linear = -0.10;
        //         vel.angular.z = angular;
        //         vel.linear.x = linear;
        //         vel_pub.publish(vel);
        //     }
        //     //turning 90 degrees
        //     // ros::Time start_time = ros::Time::now();
        //     // ros::Duration timeout(2.2); // Timeout of 2 seconds
        //     // while (ros::Time::now() - start_time < timeout)
        //     // {
        //     //     angular = M_PI / 4;
        //     //     linear = 0;
        //     //     vel.angular.z = angular;
        //     //     vel.linear.x = linear;
        //     //     vel_pub.publish(vel);
        //     // }
        //     ROS_INFO("WE'RE HIT");
        // }

        //If the left bumper or centre bumper is pressed, go backwards 0.3 and turn 90 degrees left
        if (leftState || centerState )
        {
            ros::Time start_time3 = ros::Time::now();
            ros::Duration timeout3(3.5); // Timeout of 3.5 seconds
            while (ros::Time::now() - start_time3 < timeout3)
            {
                angular = M_PI/6;
                linear = -0.10;
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
            }
        }

        //If the right bumper is pressed, go backwards 0.3 and turn 90 degrees right
        if (rightState)
        {
            ros::Time start_time3 = ros::Time::now();
            ros::Duration timeout3(3.5); // Timeout of 3.5 seconds
            while (ros::Time::now() - start_time3 < timeout3)
            {
                angular = -M_PI/6;
                linear = -0.10;
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
            }
        }

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
        loop_rate.sleep();
    }

    return 0;
}
