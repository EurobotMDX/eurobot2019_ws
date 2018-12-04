#include "drive_train_ros_handler.h"

/* define the global const variables to be used by the drive train manager */
int encoder_counts;
double gear_ratio;
double base_width_in_meters;
double wheel_diameter_in_meters;
std::string odom_frame_id;
std::string base_frame_id;

/* Create an instance of the {@DriveTrainManager drive_train_manager} */
DriveTrainManager drive_train_manager = {base_width_in_meters, wheel_diameter_in_meters, (unsigned int) encoder_counts, gear_ratio};

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
	static double x = 0;
	static double y = 0;
	static double th = 0;

	static double vx  = 0;
	static double vth = 0;

	ros::Time current_time;
	static ros::Time last_update_time = ros::Time::now();

	tf::TransformBroadcaster odometry_broadcaster;

	current_time = ros::Time::now();

	double right_position_in_meters = 0; double left_position_in_meters = 0;
	if (!drive_train_manager.get_displacements(left_position_in_meters, right_position_in_meters))
	{
		return false;
	}

	double average_displacement = (right_position_in_meters + left_position_in_meters) / 2.0;
	th = (right_position_in_meters - left_position_in_meters) / base_width_in_meters;

	// compute the odometry using the velocities of the robot
	double dt = (current_time - last_update_time).toSec();
	vx = average_displacement / dt;
	vth = th / dt;

	if ( average_displacement != 0)
	{
		double _x = std::cos(th) * average_displacement;
		double _y = std::sin(th) * average_displacement;

		x += ( std::cos( th ) * _x - std::sin( th ) * _y );
		y += ( std::sin( th ) * _x + std::cos( th ) * _y );
	}

	if (th != 0)
	{
		th += th;
	}

	// since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quaternion = tf::createQuaternionMsgFromYaw(th);

	// publish the transform over tf
	geometry_msgs::TransformStamped odom_transform;
	odom_transform.header.stamp = current_time;
	odom_transform.header.frame_id = odom_frame_id;
	odom_transform.child_frame_id = base_frame_id;

	odom_transform.transform.translation.x = x;
	odom_transform.transform.translation.y = y;
	odom_transform.transform.translation.z = 0.0;
	odom_transform.transform.rotation = odom_quaternion;

	// send the transform
	odometry_broadcaster.sendTransform(odom_transform);

	// publish the odom message over ros
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = odom_frame_id;

	// set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quaternion;

	// set the velocity
	odom.child_frame_id = base_frame_id;
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = 0;
	odom.twist.twist.angular.z = vth;

	// publish message
	odometry_publisher.publish(odom);

	// update time
	last_update_time = current_time;

	return true;
}

int main(int argc, char *argv[])
{
	ROS_INFO("drive_train_handler started");
	
	ros::init(argc, argv, "drive_train_handler");
	ros::NodeHandle nh;
	bool status = true;

	int publish_rate;
	nh.param<int>("publish_rate", publish_rate, 5);
	nh.param<int>("encoder_counts", encoder_counts, 4096);
	nh.param<double>("gear_ratio", gear_ratio, 4554.0 / 130.0);
	nh.param<double>("base_width", base_width_in_meters, 200 / 1000.0);
	nh.param<double>("wheel_diameter", wheel_diameter_in_meters, 70 / 1000.0);
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

		ros::Rate loop_rate(25);
		while (ros::ok)
		{
			if ( !publish_odometry(odometry_publisher) )
			{
				ROS_INFO("could not publish odometry");
			}

			ros::spinOnce();
			loop_rate.sleep();
		}
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


// bool publish_odometry(ros::Publisher &odometry_publisher)
// {
// 	static double x = 0;
// 	static double y = 0;
// 	static double th = 0;

// 	static double vx  = 0;
// 	static double vy  = 0;
// 	static double vth = 0;

// 	static ros::Time current_time;
// 	static ros::Time last_update_time = ros::Time::now();

// 	static tf::TransformBroadcaster odometry_broadcaster;

// 	current_time = ros::Time::now();

// 	double right_position_in_meters = 0; double left_position_in_meters = 0;
// 	if (!drive_train_manager.get_displacements(left_position_in_meters, right_position_in_meters))
// 	{
// 		return false;
// 	}

// 	double diff_right_left = right_position_in_meters - left_position_in_meters;
// 	double amplitude = (right_position_in_meters + left_position_in_meters) * 0.5;
// 	double fraction = diff_right_left / wheel_diameter_in_meters;

// 	// update the robot's velocities
// 	vx = amplitude * std::cos(th + (fraction / 2.0));
// 	vy = amplitude * std::sin(th + (fraction / 2.0));
// 	vth = fraction;

// 	// compute the odometry using the velocities of the robot
// 	double dt = (current_time - last_update_time).toSec();
// 	double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
// 	double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
// 	double delta_th = vth * dt;

// 	x += delta_x;
// 	y += delta_y;
// 	th += delta_th;

// 	// since all odometry is 6DOF we'll need a quaternion created from yaw
// 	geometry_msgs::Quaternion odom_quaternion = tf::createQuaternionMsgFromYaw(th);

// 	// publish the transform over tf
// 	geometry_msgs::TransformStamped odom_transform;
// 	odom_transform.header.stamp = current_time;
// 	odom_transform.header.frame_id = odom_frame_id;
// 	odom_transform.child_frame_id = base_frame_id;

// 	odom_transform.transform.translation.x = x;
// 	odom_transform.transform.translation.y = y;
// 	odom_transform.transform.translation.z = 0.0;
// 	odom_transform.transform.rotation = odom_quaternion;

// 	// send the transform
// 	odometry_broadcaster.sendTransform(odom_transform);

// 	// publish the odom message over ros
// 	nav_msgs::Odometry odom;
// 	odom.header.stamp = current_time;
// 	odom.header.frame_id = odom_frame_id;

// 	// set the position
// 	odom.pose.pose.position.x = x;
// 	odom.pose.pose.position.y = y;
// 	odom.pose.pose.position.z = 0.0;
// 	odom.pose.pose.orientation = odom_quaternion;

// 	// set the velocity
// 	odom.child_frame_id = base_frame_id;
// 	odom.twist.twist.linear.x = vx;
// 	odom.twist.twist.linear.y = vy;
// 	odom.twist.twist.angular.z = vth;

// 	// publish message
// 	odometry_publisher.publish(odom);

// 	// update time
// 	last_update_time = current_time;

// 	return true;
// }