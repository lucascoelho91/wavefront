
#ifndef ROBOT_H_
#define ROBOT_H_

#include <iostream>

#include <stdio.h>
#include <math.h>

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>

#include <wavefront/rgb.h>
#include <wavefront/Vector2.h>

#include <boost/lexical_cast.hpp>



class robot
{
    public:
        int id;       //agent ID
        float weight;     //agent weight
        rgb color;        //agent color in visualization
        std::string rname; //name of the agent on the topic
        char status;	  //status

        geometry_msgs::Twist speed; //speed ROS structure
		nav_msgs::Odometry poseOdom;    //position of the agent
		Vector2 pose;       //simpler way to represent the position

		ros::Publisher speedPub;    //ROS speed publisher
		ros::Subscriber poseSub; //ROS position subscriber

		void publishSpeed();
		void setSpeedPublisher(ros::NodeHandle& nh);
		void setPoseSubscriber(ros::NodeHandle* nh);
		void setSpeed(double v, double w);
		void setWeight(double p);
		double getPower();
		std::string getName();
		robot(int id, double weight, rgb color, std::string name);

		void setSpeedHolo(double dx, double dy);

		void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);


};

#endif /* ROBOT_H_ */