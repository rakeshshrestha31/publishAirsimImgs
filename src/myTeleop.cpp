#define TELEOP_DURATION 2.0

#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "common/Common.hpp"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <iostream>

//using namespace std;
//string localization_method;
msr::airlib::CarRpcLibClient * client;

int main(int argc, char **argv)
{
  
  //Start ROS ----------------------------------------------------------------
  ros::init(argc, argv, "airsim_teleop");
  ros::NodeHandle n("~");
  ros::Rate loop_rate(60);

  
  //Parameters for communicating with Airsim
  std::string ip_addr;
  int portParam;
  ros::param::param<std::string>("~Airsim_ip",ip_addr, "localhost");   //"192.168.0.107");
  ros::param::param<int>("~Airsim_port", portParam, 0); //41451);
  uint16_t port = portParam;

  //int is_startup_takeoff=0;
  //ros::param::param<int>("~Airsim_startup_takeoff", is_startup_takeoff, 0);

  //Verbose
  ROS_INFO("Image publisher connecting to:");
  ROS_INFO("IP: %s", ip_addr.c_str());
  ROS_INFO("Port: %d", port);
  //ROS_INFO("is_startup_takeoff: %d", is_startup_takeoff);
  //this connects us to the drone 
  if (!port)
  {
    client = new msr::airlib::CarRpcLibClient(ip_addr);  
  }
  else
  {
    client = new msr::airlib::CarRpcLibClient(ip_addr, port);
  }
  //client->enableApiControl(false);
  client->confirmConnection();
  client->enableApiControl(true);

/*  if (is_startup_takeoff)
  {
    ROS_INFO("Waiting to take off");
    client->takeoff(2);
    ROS_INFO("took off");
  }
*/

  //auto drive_train = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
  //msr::airlib::YawMode yaw_mode(true, 0);

  ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 100,
        (boost::function<void (const geometry_msgs::TwistConstPtr& )>)
        [&](const geometry_msgs::TwistConstPtr& twist_msg) -> void {
        using namespace msr::airlib;
        
        CarApiBase::CarControls controls;
        controls.steering = -twist_msg->angular.z;
        controls.throttle = twist_msg->linear.x;
        
        if(twist_msg->linear.x > 0)
        {
            controls.is_manual_gear = false;
        }
        else
        {
            controls.is_manual_gear = true;
            controls.manual_gear = -1;
        }
        
        client->setCarControls(controls);
        
        }
  );

  ros::spin();

  return 0;
}

