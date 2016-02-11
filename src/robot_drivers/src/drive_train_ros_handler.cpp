#include "drive_train_ros_handler.h"

#include <iostream>

/* define the global const variables to be used by the drive train manager */
std::string odom_frame_id;
std::string base_frame_id;

/* Create an instance of the {@DriveTrainManager drive_train_manager} */
DriveTrainManager drive_train_manager;

/**
 *  This funciton recieves twist messages and
 *  sends it to the {@DriveTrainManager drive_train_manager}
 */
void twist_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
	double linear_vel = msg->linear.x;
	double angular_vel = msg->angular.z;

	drive_train_manager.set_motion(linear_vel, angular_vel);
}


/**
 *  This funciton publishes {@Quaternion odom_quaternion} messages on the topic {@nh.advertise<nav_msgs::Odometry>(odom_frame_id 50) odometry_publisher}
 *  using the {@TransformBroadcaster odometry_broadcaster}
 */
bool publish_odometry(ros::Publisher &odometry_publisher)
{
	ros::Time current_time = ros::Time::now();
	tf::TransformBroadcaster odometry_broadcaster;
	
	// since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quaternion = tf::createQuaternionMsgFromYaw(drive_train_manager.current_theta);

	// publish the transform over tf
	geometry_msgs::TransformStamped odom_transform;
	odom_transform.header.stamp = current_time;
	odom_transform.header.frame_id = odom_frame_id;
	odom_transform.child_frame_id = base_frame_id;

	odom_transform.transform.translation.x = drive_train_manager.current_x;
	odom_transform.transform.translation.y = drive_train_manager.current_y;
	odom_transform.transform.translation.z = 0.0;
	odom_transform.transform.rotation = odom_quaternion;

	// send the transform
	odometry_broadcaster.sendTransform(odom_transform);

	// publish the odom message over ros
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = odom_frame_id;

	// set the position
	odom.pose.pose.position.x = drive_train_manager.current_x;
	odom.pose.pose.position.y = drive_train_manager.current_y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quaternion;

	// set the velocity
	odom.child_frame_id = base_frame_id;
	odom.twist.twist.linear.x = drive_train_manager.current_vx;
	odom.twist.twist.linear.y = drive_train_manager.current_vy;
	odom.twist.twist.angular.z = drive_train_manager.current_vtheta;

	// publish message
	odometry_publisher.publish(odom);

	int left_position; int right_position;
	drive_train_manager.left_wheel.get_position(left_position);
	drive_train_manager.right_wheel.get_position(right_position);

	// std::cout << "lt counts: " << left_position << std::endl;
	// std::cout << "rt counts: " << right_position << std::endl;
	std::cout << "vx: " << drive_train_manager.current_vx << std::endl;
	std::cout << "vy: " << drive_train_manager.current_vy << std::endl;
	std::cout << "x: " << drive_train_manager.current_x << std::endl;
	std::cout << "y: " << drive_train_manager.current_y << std::endl;
	std::cout << "theta: " << (drive_train_manager.current_theta / M_PI) * 180.0 << std::endl << std::endl;

	return true;
}

void odometry_loop()
{
	ros::Rate loop_rate(15);

	while (drive_train_manager.should_run)
	{
		drive_train_manager.update_odometry();
		loop_rate.sleep();
	}
}

int main(int argc, char *argv[])
{
	ROS_INFO("drive_train_handler started");
	
	ros::init(argc, argv, "drive_train_handler");
	ros::NodeHandle nh;
	bool status = true;

	int publish_rate;
	nh.param<int>("publish_rate", publish_rate, 5);
	nh.param<std::string>("odom_frame_id", odom_frame_id, "odom");
	nh.param<std::string>("base_frame_id", base_frame_id, "base_link");


	if ( !drive_train_manager.initialize("USB0", "USB1") )
	{
		ROS_INFO("Setup failed");
		status = false;
	}

	if (status)
	{
		ROS_INFO("Setup successful");
		ros::Subscriber twist_subscriber = nh.subscribe("/cmd_vel", 10, twist_callback);
		ros::Subscriber teleop_subscriber = nh.subscribe("/cmd_vel_mux/input/teleop", 10, twist_callback);
		ros::Publisher odometry_publisher = nh.advertise<nav_msgs::Odometry>(odom_frame_id, publish_rate);

		std::thread odometry_update_loop(odometry_loop);

		ros::Rate loop_rate(publish_rate);
		while (ros::ok)
		{
			if ( !publish_odometry(odometry_publisher) )
			{
				ROS_INFO("could not publish odometry");
			}

			ros::spinOnce();
			loop_rate.sleep();
		}

		odometry_update_loop.join();
	}

	if ( !drive_train_manager.terminate() )
	{
		ROS_INFO("Shutdown failed");
	}
	else
	{
		ROS_INFO("Shutdown successful");
	}

	return 0;
}