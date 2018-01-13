#define TELEOP_DURATION 2.0

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/Common.hpp"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

using namespace std;
string localization_method;
msr::airlib::MultirotorRpcLibClient * client;

int main(int argc, char **argv)
{
  
    
  //Start ROS ----------------------------------------------------------------
  ros::init(argc, argv, "airsim_teleop");
  ros::NodeHandle n("~");
  ros::Rate loop_rate(60);

  //Parameters for communicating with Airsim
  string ip_addr;
  int portParam;
  ros::param::param<std::string>("~Airsim_ip",ip_addr,"localhost");
  ros::param::param<int>("~Airsim_port", portParam, 0);
  uint16_t port = portParam;

  int is_startup_takeoff=0;
  ros::param::param<int>("~Airsim_startup_takeoff", is_startup_takeoff, 0);

  //Verbose
  ROS_INFO("Image publisher connecting to:");
  ROS_INFO("IP: %s", ip_addr.c_str());
  ROS_INFO("Port: %d", port);
  ROS_INFO("is_startup_takeoff: %d", is_startup_takeoff);
  //this connects us to the drone 
  if (!port)
  {
    client = new msr::airlib::MultirotorRpcLibClient(ip_addr);  
  }
  else
  {
    client = new msr::airlib::MultirotorRpcLibClient(ip_addr, port);
  }
  //client->enableApiControl(false);
  client->confirmConnection();
  client->enableApiControl(true);

  if (is_startup_takeoff)
  {
    ROS_INFO("Waiting to take off");
    client->takeoff(2);
    ROS_INFO("took off");
  }

  auto drive_train = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
  msr::airlib::YawMode yaw_mode(true, 0);
  
  ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist>(
    "cmd_vel", 100,
    [&](const geometry_msgs::TwistConstPtr &twist_msg) {
      if (twist_msg->angular.z)
      {
        client->rotateByYawRate(-twist_msg->angular.z, TELEOP_DURATION);
      }
      else
      {
        using namespace msr::airlib;
        auto global_velocity = VectorMathT<Vector3r, Quaternionr, real_T>::transformToWorldFrame(
          (Vector3r() << twist_msg->linear.x, twist_msg->linear.y, twist_msg->linear.z).finished(), 
          client->getOrientation()
        );
        client->moveByVelocity(
          global_velocity(0), global_velocity(1), global_velocity(2),
          TELEOP_DURATION, drive_train, yaw_mode
        );
      }
    }
  );

  ros::spin();

  //poll_frame_thread.join();
  return 0;
}

