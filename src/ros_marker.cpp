#include <sstream>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include "tf/transform_listener.h"
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <ar_pose/ARMarkers.h>
// #include <ar_pose/ARMarker.h>
#include <ros_project/ctrlparamsConfig.h>

double velx=0;
double vely=0;
double velz=0;
double roll, pitch, yaw;
bool Takeoff_config = false;
bool Land_config = false;
bool only_once = true;
std_msgs::Empty empty;
std_srvs::Empty tog;
geometry_msgs::Twist vel;
tf::Quaternion qt;
float Xm = 0; float Ym = 0; float Zm = 0;
float Kp = 0.1; float Ki = 0.0; float Kd = 0.0;
ros::Publisher lnd, toff, velocity;
// ros::Subscriber image_feed;
ros::ServiceClient toggle_camera;
bool camera_state = false; //assume false is bottom camera and true is front camera
bool is_marker_ok = true; // by default the marker is set to ok, later we evaluate in the code.
int drone_state = 0;
ros::Time begin;

void marker_position(const ar_pose::ARMarkers::ConstPtr& msg)
{
	ar_pose::ARMarker ar_marker;
	if (!msg->markers.empty())
	{
		ar_marker = msg->markers[0];
		Xm = ar_marker.pose.pose.position.x;
		Ym = ar_marker.pose.pose.position.y;
		Zm = ar_marker.pose.pose.position.z;
		tf::quaternionMsgToTF(ar_marker.pose.pose.orientation, qt);
		tf::Matrix3x3(qt).getRPY(roll, pitch, yaw);
		is_marker_ok=true;
		ROS_INFO("Marker Found");
		ROS_INFO("X : [%f] Y : [%f] Z : [%f] ", ar_marker.pose.pose.position.x, ar_marker.pose.pose.position.y, ar_marker.pose.pose.position.z);
		if (std::isnan(roll) || !std::isfinite(roll) || (ar_marker.pose.pose.position.z<0))
		{
			is_marker_ok=false;
			ROS_INFO("ROLL IS INVALID");
			ROS_INFO("Inside condition");
		}
	}
	else
		is_marker_ok=false;
}

void togglecam(void)
{
	camera_state = !camera_state; // will change the camera state when togglecam is called 
	toggle_camera.call(tog);
}

void image_feed(const sensor_msgs::ImageConstPtr &msg)
{ 
	cv_bridge::CvImagePtr img;
	try
	{
		img = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	}
	catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
	cv::imshow("ardrone display",img->image);
	cv::waitKey(2);
}

void defvel(void)
{
	vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = 0;
	vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;
}

void takeoff(void)
{
	toff.publish(empty);
	Takeoff_config = true;
	Land_config = false;
}

void land(void)
{
	lnd.publish(empty);
	Land_config = true;
	Takeoff_config = false;
}

void velocity_pub(const geometry_msgs::Twist& vety)
{
	velocity.publish(vety);
}

void parameter(ros_project::ctrlparamsConfig &config, uint32_t level) 
{
  Kp = config.Kp;
  Ki = config.Ki;
  Kd = config.Kd;
  if (!Takeoff_config && config.takeoff)// && !Land_config)
  {
  	takeoff();
  	ROS_INFO("Takeoff Operation in-action");
  	// toggle_camera.call(tog);
  }
  if (!Land_config && config.land && Takeoff_config)
  {
  	land();
  	ROS_INFO("Landing Operation in-action (Action from dynamic_reconfigure)");
  }
  ROS_INFO("Reconfigure Request: Kp - %f Ki - %f Kd - %f ",config.Kp, config.Ki, config.Kd);
}

double pidx(void)
{
	double x;
    double error = 0 - Xm;
	double integral = 0;//integral + error*dt;
	double derivative = 0;//(error - previous_error)/dt;
	x = Kp*error + Ki*integral + Kd*derivative;
 	// x = x > 1.0 ? 1.0 : x;
 	// x = x < -1.0 ? -1.0 : x;
	return x;
}

double pidy(void)
{
	double y;
    double error = 0 - Ym;
	double integral = 0;//integral + error*dt;
	double derivative = 0;//(error - previous_error)/dt;
	y = Kp*error + Ki*integral + Kd*derivative;
	return y;
}

double pidz(void)
{
	double z;
    double error = 0 - Zm;
	double integral = 0;//integral + error*dt;
	double derivative = 0;//(error - previous_error)/dt;
	z = Kp*error + Ki*integral + Kd*derivative;
	return z;
}


void controlaction(void)
{
	int a,b;
	if(drone_state==0)
	{
		togglecam();
		drone_state = 1;
	}
	if (drone_state==1 && is_marker_ok)
	{
		defvel();
		if (Xm>0.2 & Xm<-0.2)
		{
			vel.linear.x = pidx();
		}
		else
		{
			if (Ym>0.2 & Ym<-0.2)
			{
				vel.linear.y = pidy();
			}
			else
			{
				if (yaw<-3.12 || yaw>3.12)
					vel.angular.z=0;
				else
					vel.angular.z = 0.5*yaw;
			}
		}
		if (Zm>1.5)
		{
			vel.linear.z = pidz();
		}
		// ROS_INFO("yaw : %4.2f", yaw);
		velocity_pub(vel);
		defvel();
		if (((Xm<0.2 & Xm>-0.2) && (Ym<0.2 & Ym>-0.2)) && (yaw<-3.10 || yaw>3.10) )
		{
			defvel();
			velocity_pub(vel);
			ROS_INFO("Will Hover for 5 seconds");
			// while((ros::Time::now()-begin).toSec()>=5)
			// {
			// 	defvel();
			// 	velocity_pub(vel);
			// 	drone_state=2;	
			// }
			drone_state=2;
			// ros::Duration(5.0).sleep(); // sleep for half a second
			// ROS_INFO("Hovering at spot for 5 seconds is successful");
			togglecam();
			is_marker_ok=false;
			begin = ros::Time::now();

		}
	}
	// if (drone_state==1 && !is_marker_ok)
	// {
	// 	defvel();
	// 	vel.linear.z = 0.5;
	// 	velocity_pub(vel);	
	// }
	if (drone_state==2)
	{
		defvel();
		if ((ros::Time::now()-begin).toSec() >=5)
		{
			drone_state=3;
		}	
		else
		{
			velocity_pub(vel);
			ROS_INFO("Hovering at spot for 5 seconds is successful");
		}
	}
	if (drone_state==3)
	{
		// ROS_INFO("IN drone_state 2");
		if(!is_marker_ok)
		{
			defvel();
			vel.angular.z = -0.9;
			velocity_pub(vel);
			defvel();	
		}
		else
		{
			ROS_INFO("IN ELSE");
			defvel();
			velocity_pub(vel);
			drone_state=4;
			// land();
		}
	}
	if (drone_state==4)
	{
		defvel();
		if (Xm>0.2 || Xm<-0.2)
		{
			vel.angular.z = pidx();
		}
		if (Ym>0.2 || Ym<-0.2)
		{
			vel.linear.z = pidy();
		}
		if ((Zm-1)>0.1 || (Zm-1)<-0.1)
		{
			vel.linear.x = 0.2*(Zm-1);
		}
		// else
		// {
		// 	if (Ym>0.2 || Ym<-0.2)
		// 	{
		// 		vel.linear.z = pidy();
		// 	}
		// 	else
		// 	{
		// 		if ((Zm-1)>0.1 || (Zm-1)<-0.1)
		// 		{
		// 			vel.linear.x = 0.2*(Zm-1);
		// 		}
		// 	}
		// }
		velocity_pub(vel);
		defvel();
		if (((Zm-1)<0.1 && (Zm-1)>-0.1) & (Xm<0.2 && Xm>-0.2) & (Ym<0.2 && Ym>-0.2))
		{
			defvel();
			velocity_pub(vel);
			drone_state=5;
		}
	}
	if (drone_state==5)
	{
		if (is_marker_ok)
		{
			if(pitch>0.02 || pitch<-0.02)
			{
				defvel();
				vel.angular.z=-0.5*pitch;
				velocity_pub(vel);
			}
			else
			{
				drone_state=6;
				begin = ros::Time::now();
			}
			if (Ym>0.1 || Ym<-0.1)
			{
				vel.linear.z = pidy();
			}
		}
		else
		{
			drone_state==3;
			defvel();
			velocity_pub(vel);
		}
	}
	if (drone_state==6)
	{
		defvel();
		if ((ros::Time::now()-begin).toSec() >=5)
		{
			drone_state=7;
		}	
		else
			velocity_pub(vel);
	}
	if (drone_state==7)
	{
		ROS_INFO("GOAL SUCCESSFULL!!!!");
		land();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "node_marker");
  	ros::NodeHandle n;
  	toff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); //publisher of takeoff operation
  	lnd = n.advertise<std_msgs::Empty>("/ardrone/land", 1); // publisher of land operation
  	velocity = n.advertise<geometry_msgs::Twist>("/cmd_vel",1); //publisher of velocities
	ros::Subscriber sub = n.subscribe("/ar_pose_marker", 1, marker_position); //subscriber
	ros::Subscriber im_feed = n.subscribe("/ardrone/image_raw", 1, image_feed); //subscriber
	// ros::Subscriber sub = n.subscribe("/ar_pose_marker", 1, marker_position); //subscriber
	toggle_camera = n.serviceClient<std_srvs::Empty>("/ardrone/togglecam"); //service

	dynamic_reconfigure::Server<ros_project::ctrlparamsConfig> server;
	dynamic_reconfigure::Server<ros_project::ctrlparamsConfig>::CallbackType func;
	func = boost::bind(&parameter, _1, _2);
	server.setCallback(func);
	ros::Rate loop_rate(10);
	takeoff();
	// if(toggle_camera.call(tog))
	// 	ROS_INFO("Camera Toggled to Bottom Camera for Initial Marker Detection");
	// else
	// 	ROS_INFO("Camera is not toggled - Front Camera in-operation");\
	// ros::Duration(3.0).sleep(); // sleep for half a second
	// defvel();
	// vel.linear.y = 1.0;
	// velocity_pub(vel);
	// ros::Duration(4.0).sleep(); // sleep for half a second
	// defvel();
	// toggle_camera.call(tog);
	begin = ros::Time::now();
	while (ros::ok())
	{
		ros::spinOnce();
		if(Takeoff_config)
			takeoff();
		if (Land_config)
			land();
		controlaction();
	}
	return 0;
}