#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <cmath>
#include "sensor_msgs/Imu.h"
#include "nord_messages/IMUValues.h"
#include "nord_messages/Vector2.h"

class IMUReader
{

	public:
	ros::Subscriber imu_sub;
	ros::Publisher values_pub;	
	ros::Publisher bump_pub;	
	ros::NodeHandle n;
	nord_messages::IMUValues robot;

	IMUReader()
	{
		imu_sub = n.subscribe("/imu/data", 1, &IMUReader::IMUOrientationCallback, this);
		values_pub = n.advertise<nord_messages::IMUValues>("/imu/calc_data", 1);
		bump_pub = n.advertise<nord_messages::Vector2>("/imu/bump", 1);
		
		//setting all values in the arrays to 0
		for(int i = 0; i<4; i++){
			q[i] = 0;
			if(i<3){
				gravity[i] = 0;
				a[i] = 0;
				old_a[i] = 0;
			}
		}

		w_pitch = 0; w_slope = 0; w_rotation = 0;
		bump_array[0] = 0; bump_array[1] = 0;
		
	}


	void IMUOrientationCallback(const sensor_msgs::Imu values)
	{
		//Gettin the quaternions
		q[0] = values.orientation.w; q[1] = values.orientation.x;
		q[2] = values.orientation.y; q[3] = values.orientation.z;
		
		//Getting the angular velocities from imu
		w_pitch = values.angular_velocity.x;
		w_slope = values.angular_velocity.y;
		w_rotation = values.angular_velocity.z;

		//Getting the raw acc data from IMU
		a[0] =  values.linear_acceleration.x;
		a[1] =  values.linear_acceleration.y;
		a[2] =  values.linear_acceleration.z;
		
		removeGravity();
		transformIntoRad();
		detectBump();
		rosPrint(); //Turn off when code has been validated
					//comments slow down the processing

		//assigning all the correct values to the publisher
		robot.direction   = rad_yaw;
		robot.acc_forward = a[1];
		robot.acc_right   = a[0];
		robot.acc_up      = a[2];

		values_pub.publish(robot);
	}

	void removeGravity(){
		gravity[0]  = 2*(q[1]*q[3]-q[0]*q[2]);
		gravity[1]  = 2*(q[0]*q[1]+q[2]*q[3]);
		gravity[2]  = pow(q[0],2.0) - pow(q[1],2.0) - pow(q[2],2.0) + pow(q[3],2.0);
		
		a[0] -= gravity[0]*9.81; //removes gravity from the long side
		a[1] -= gravity[1]*9.81; //removes gravity from the short side
		a[2] -= gravity[2]*9.81; //removes gravity from the flat side

	}

	void transformIntoRad(){
		
		// rad_roll  = atan2(2*(q[2]*q[0]-q[1]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2]));
		// rad_pitch =	asin(2*(q[1]*q[2]+q[3]*q[0]));
		//Returns the angle the robot will be looking in
		//WARNING value shifts from 3.14 to minus -3.14 as soon as yo go above 180 deg
		rad_yaw   = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));

	}

	//This is Beta, we have not decided what we should send, how we should handle it
	//or calibrated the function yet. NEEDS FIXING!
	void detectBump(){
		float dx = a[1] - old_a[1];
		float dy = a[2] - old_a[2];
		if(std::sqrt(dx * dx + dy * dy) > 6)
		{
			bump_array[0] = a[1]-old_a[1];
			bump_array[1] = a[2]-old_a[2];
			ROS_INFO("___________BUMP__________");
			ROS_INFO("bump_array[ %f, %f]", bump_array[0],bump_array[1]);
			nord_messages::Vector2 msg;
			msg.x = bump_array[0];
			msg.y = bump_array[1];
			bump_pub.publish(msg);
		}

		for(int i = 0; i < 3; i++){
			old_a[i] = a[i];
		}	
	}


	void rosPrint(){
		// ROS_INFO("w_pitch    = %f ", w_pitch);
		// ROS_INFO("w_slope    = %f ", w_slope);
		// ROS_INFO("w_rotation = %f ", w_rotation);
		// ROS_INFO("gravity x   = %f ", gravity[0]);
		// ROS_INFO("gravity y   = %f ", gravity[1]);
		// ROS_INFO("gravity z   = %f ", gravity[2]);
		//ROS_INFO("acc_right   = %f ", a[0]);
		//ROS_INFO("acc_forward = %f ", a[1]);
		//ROS_INFO("acc_upp     = %f ", a[2]);
		// ROS_INFO("rad_roll  = %f ", rad_roll);
		// ROS_INFO("rad_pitch = %f ", rad_pitch);
		//ROS_INFO("rad_yaw   = %f ", rad_yaw);

	}


	// forward is defined as if the cord was the tail
	// the top of the PCB is defiened as up
	// right is defined whe the top is up and back is cord

	// pitch =  /  down = +
	// slope = turn around the cord  right = +
	// yaw = around the pcb normal  right = + 

	private:
		double q[4]; double gravity[3]; double a[3]; //acceleration
		double w_pitch; double w_slope; double w_rotation;
		double rad_roll; double rad_pitch; double rad_yaw; 
		double old_a[3]; double bump_array[2];



};

int main(int argc,char** argv){
	ros::init(argc, argv, "nord_IMU");
	IMUReader object;

	ros::Rate loop_rate(20); //atm it updates in 20 hz

	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep(); // go to sleep
	}
	return 0;
}
