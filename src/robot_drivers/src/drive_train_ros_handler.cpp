#include "drive_train_ros_handler.h"

#include <iostream>

/* define the global const variables to be used by the drive train manager */
std::string odom_frame_id;
std::string base_frame_id;
std::string global_frame_id;

double odom_offset_x = 0.0;
double odom_offset_y = 0.0;
double odom_offset_yaw = 0.0;

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

void reset_drive_train_callback(const std_msgs::Bool::ConstPtr& msg)
{
	drive_train_manager.reset_encoders();	
	drive_train_manager.reset();

	odom_offset_x = drive_train_manager.current_x;
	odom_offset_y = drive_train_manager.current_y;
	odom_offset_yaw = drive_train_manager.current_theta;
}


void update_drive_train_position_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	double roll, pitch, yaw;

	// reset the drive train
	drive_train_manager.reset();

	geometry_msgs::Pose new_pose = msg->pose.pose;
	tf::Quaternion q(new_pose.orientation.x, new_pose.orientation.y, new_pose.orientation.z, new_pose.orientation.w);
	tf::Matrix3x3 m(q);

	m.getRPY(roll, pitch, yaw);

	// std::cout << "q: " << new_pose.orientation.x << ", " << new_pose.orientation.y << ", " << new_pose.orientation.z << ", " << new_pose.orientation.w << std::endl;
	// std::cout << "RPY: " << roll << ", " << pitch << ", " << yaw << std::endl;

	odom_offset_x = new_pose.position.x;
	odom_offset_y = new_pose.position.y;
	odom_offset_yaw = yaw - M_PI_2;
}


/**
 *  This funciton publishes {@Quaternion odom_quaternion} messages on the topic {@nh.advertise<nav_msgs::Odometry>(odom_frame_id 50) odometry_publisher}
 *  using the {@TransformBroadcaster odometry_broadcaster}
 */
bool publish_odometry(ros::Publisher &odometry_publisher)
{
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion q;

	ros::Time _time = ros::Time::now() + ros::Duration(10);

	// broadcast the transform between the map and the odom frame
	transform.setOrigin( tf::Vector3(odom_offset_x, odom_offset_y, 0.0) );
	q.setRPY(0,0,odom_offset_yaw);
	q.normalize();
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), global_frame_id, odom_frame_id));

	// broadcast the transform between the base and the odom frame
	// see: wiki.ros.org/amcl  towards the bottom of the page there's a really nice page on {map_frame, odom_frame, base_frame}
	transform.setOrigin( tf::Vector3(drive_train_manager.current_x, drive_train_manager.current_y, 0.0) );
	q.setRPY(0,0, drive_train_manager.current_theta);
	q.normalize();
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odom_frame_id, base_frame_id));

	// publish the odom message over ros
	nav_msgs::Odometry odom;

	// set the position
	odom.pose.pose.position.x = drive_train_manager.current_x;
	odom.pose.pose.position.y = drive_train_manager.current_y;
	odom.pose.pose.position.z = 0.0;

	// set the orientation
	geometry_msgs::Quaternion odom_quaternion = tf::createQuaternionMsgFromYaw(drive_train_manager.current_theta);
	odom.pose.pose.orientation = odom_quaternion;

	// set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = drive_train_manager.current_vx;
	odom.twist.twist.linear.y = drive_train_manager.current_vy;
	odom.twist.twist.angular.z = drive_train_manager.current_vtheta;

	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = odom_frame_id;

	// publish message
	odometry_publisher.publish(odom);

	// int left_position; int right_position;
	// drive_train_manager.left_wheel.get_position(left_position);
	// drive_train_manager.right_wheel.get_position(right_position);

	// std::cout << "lt counts: " << left_position << std::endl;
	// std::cout << "rt counts: " << right_position << std::endl;
	// std::cout << "vx: " << drive_train_manager.current_vx << std::endl;
	// std::cout << "vy: " << drive_train_manager.current_vy << std::endl;
	// std::cout << "x: " << drive_train_manager.current_x << std::endl;
	// std::cout << "y: " << drive_train_manager.current_y << std::endl;
	// std::cout << "theta: " << (drive_train_manager.current_theta / M_PI) * 180.0 << std::endl << std::endl;

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
	nh.param<int>("publish_rate", publish_rate, 10);
	nh.param<std::string>("odom_frame_id", odom_frame_id, "odom");
	nh.param<std::string>("base_frame_id", base_frame_id, "base_link");
	nh.param<std::string>("global_frame_id", global_frame_id, "map");


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
		ros::Subscriber reset_drive_train_subscriber = nh.subscribe("/reset_drive_train", 10, reset_drive_train_callback);
		ros::Subscriber initialpose_subscriber = nh.subscribe("initialpose", 10, update_drive_train_position_callback);
		ros::Subscriber update_drive_train_position_subscriber = nh.subscribe("update_drive_train_position", 10, update_drive_train_position_callback);
		ros::Publisher odometry_publisher = nh.advertise<nav_msgs::Odometry>("odom", publish_rate);

		std::thread odometry_update_loop(odometry_loop);

		// std::cout << "publish_rate: " << publish_rate << std::endl;
		// std::cout << "global_frame_id: " << global_frame_id << std::endl;
		// std::cout << "odom_frame_id: " << odom_frame_id << std::endl;
		// std::cout << "base_frame_id: " << base_frame_id << std::endl;

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

		drive_train_manager.should_run = false;
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