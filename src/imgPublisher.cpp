#define TELEOP_DURATION 2.0

// #define PUBLISH_DISPARITY
#undef PUBLISH_DISPARITY

#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <iostream>
#include <math.h>
#include <iterator>
#include "common/Common.hpp"
#include <fstream>
#include "input_sampler.h"
#include "Callbacks/callbacks.h"
#include <signal.h>
#include "stereo_msgs/DisparityImage.h"
#include <thread>
#include <mutex>

using namespace std;
string localization_method;
msr::airlib::MultirotorRpcLibClient * client;
extern std::mutex client_mutex;
void sigIntHandler(int sig)
{
    //my_thread.join(); 
    client_mutex.lock(); 
    ros::shutdown();
    //exit(0);
    //client_mutex.unlock();
}



sensor_msgs::CameraInfo getCameraParams(){
    double Tx, Fx, Fy, cx, cy, width, height;
    sensor_msgs::CameraInfo CameraParam;

    // Read camera parameters from launch file
    ros::param::get("Tx",Tx);
    ros::param::get("Fx",Fx);
    ros::param::get("Fy",Fy);
    ros::param::get("cx",cx);
    ros::param::get("cy",cy);
    ros::param::get("scale_x",width);
    ros::param::get("scale_y",height);

    //CameraParam.header.frame_id = "camera";
    CameraParam.header.frame_id = localization_method;

    CameraParam.height = height;
    CameraParam.width = width;

    CameraParam.distortion_model = "plumb_bob";
    CameraParam.D = {0.0, 0.0, 0.0, 0.0, 0.0};

    CameraParam.K = {Fx,  0.0, cx, 
                     0.0, Fy,  cy, 
                     0.0, 0.0, 1};
    CameraParam.R = {1.0, 0.0, 0.0, 
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0};
    CameraParam.P = {Fx,  0.0, cx,  Tx, 
                     0.0, Fy,  cy,  0.0, 
                     0.0, 0.0, 1.0, 0.0};

    CameraParam.binning_x = 0;
    CameraParam.binning_y = 0;

    return CameraParam;
}

void CameraPosePublisher(geometry_msgs::Pose CamPose, geometry_msgs::Pose CamPose_gt)
{
    static tf::TransformBroadcaster br;
    tf::Transform transformQuad, transformCamera;
    const double sqrt_2 = 1.41421356237;
    transformCamera.setOrigin(tf::Vector3(CamPose.position.y,
                                        CamPose.position.x,
                                        -CamPose.position.z));

    geometry_msgs::Vector3 rpy =  quat2rpy(CamPose.orientation);
    rpy.y = -rpy.y;
    rpy.z = -rpy.z + M_PI/2.0;

    geometry_msgs::Quaternion q_body2cam = setQuat(0.5, -0.5, 0.5, -0.5);

    geometry_msgs::Quaternion q_cam = rpy2quat(rpy);
    q_cam = quatProd(q_body2cam, q_cam);
    transformCamera.setRotation(tf::Quaternion(q_cam.x,
                                             q_cam.y,
                                             q_cam.z, 
                                             q_cam.w));

    if (localization_method != "ground_truth" && localization_method !="orb_slam2_rgbd"){ //note that slam itself posts this transform
        br.sendTransform(tf::StampedTransform(transformCamera, ros::Time::now(), "world", localization_method));
    }  
    
    
    //ground truth values
    static tf::TransformBroadcaster br_gt;
    tf::Transform transformQuad_gt, transformCamera_gt;
    transformCamera_gt.setOrigin(tf::Vector3(CamPose_gt.position.y,
                                        CamPose_gt.position.x,
                                        -CamPose_gt.position.z));

    geometry_msgs::Vector3 rpy_gt =  quat2rpy(CamPose_gt.orientation);
    rpy_gt.y = -rpy_gt.y;
    rpy_gt.z = -rpy_gt.z + M_PI/2.0;

    geometry_msgs::Quaternion q_body2cam_gt = setQuat(0.5, -0.5, 0.5, -0.5);

    geometry_msgs::Quaternion q_cam_gt = rpy2quat(rpy_gt);
    q_cam_gt = quatProd(q_body2cam_gt, q_cam_gt);
    transformCamera_gt.setRotation(tf::Quaternion(q_cam_gt.x,
                                             q_cam_gt.y,
                                             q_cam_gt.z, 
                                             q_cam_gt.w));
    br_gt.sendTransform(tf::StampedTransform(transformCamera_gt, ros::Time::now(), "world", "ground_truth"));
}

void do_nothing(){
    return;
}
//std::thread poll_frame_thread(do_nothing);


int main(int argc, char **argv)
{
  
    
  //Start ROS ----------------------------------------------------------------
  ros::init(argc, argv, "airsim_imgPublisher");
  ros::NodeHandle n("~");
  ros::Rate loop_rate(60);

    
  //Publishers ---------------------------------------------------------------
  image_transport::ImageTransport it(n);

  // image_transport::Publisher imgL_pub = it.advertise("/Airsim/left/image_raw", 1);
  image_transport::Publisher imgR_pub = it.advertise("right/image_raw", 1);
  image_transport::Publisher depth_pub = it.advertise("depth", 1);

   ros::Publisher imgParamL_pub = n.advertise<sensor_msgs::CameraInfo> ("left/camera_info", 1);
  ros::Publisher imgParamR_pub = n.advertise<sensor_msgs::CameraInfo> ("right/camera_info", 1);
  ros::Publisher imgParamDepth_pub = n.advertise<sensor_msgs::CameraInfo> ("camera_info", 1);
  ros::Publisher disparity_pub = n.advertise<stereo_msgs::DisparityImage> ("disparity", 1);
  //ROS Messages
  sensor_msgs::ImagePtr msgImgL, msgImgR, msgDepth;
  sensor_msgs::CameraInfo msgCameraInfo;

  //Parameters for communicating with Airsim
  string ip_addr;
  int portParam;
  ros::param::param<std::string>("~Airsim_ip",ip_addr,"localhost");
  ros::param::param<int>("~Airsim_port", portParam, 0);
  uint16_t port = portParam;

  // Parameter for localizing camera
  if(!ros::param::get("~localization_method", localization_method)){
    ROS_FATAL_STREAM("you have not set the localization method");
    return -1;
  }

  int is_startup_takeoff=0;
  ros::param::param<int>("~Airsim_startup_takeoff", is_startup_takeoff, 0);

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

  //Verbose
  ROS_INFO("Image publisher started! Connecting to:");
  ROS_INFO("IP: %s", ip_addr.c_str());
  ROS_INFO("Port: %d", port);
  
  //Local variables
  input_sampler input_sample__obj(ip_addr.c_str(), port, localization_method);
  msgCameraInfo = getCameraParams();

  // std::thread poll_frame_thread(&input_sampler::poll_frame, &input_sample__obj);
  signal(SIGINT, sigIntHandler);

  auto drive_train = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
  msr::airlib::YawMode yaw_mode(true, 0);
  ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist>(
    "cmd_vel", 100,
    [&](const geometry_msgs::TwistConstPtr &twist_msg) {
      if (twist_msg->angular.z)
      {
        client->rotateByYawRate(-twist_msg->angular.z*180/M_PI, TELEOP_DURATION);
      }
      else
      {
        using namespace msr::airlib;
        auto global_velocity = VectorMathT<Vector3r, Quaternionr, real_T>::transformToWorldFrame(
          (Vector3r() << twist_msg->linear.x, twist_msg->linear.y, -twist_msg->linear.z).finished(), 
          client->getOrientation()
        );
        client->moveByVelocity(
          global_velocity(0), global_velocity(1), global_velocity(2),
          TELEOP_DURATION, drive_train, yaw_mode
        );
      }
    }
  );

  // *** F:DN end of communication with simulator (Airsim)
  while (ros::ok())
  {
#ifdef PUBLISH_DISPARITY
    auto imgs = input_sample__obj.image_decode();
    //auto imgs = input_sample__obj.poll_frame_and_decode();
    if (!imgs.valid_data) {
        continue;
    }

    
    cv::Mat disparityImageMat;
    imgs.depth.convertTo(disparityImageMat, CV_8UC1);
    stereo_msgs::DisparityImage disparityImg;
    disparityImg.header.stamp = ros::Time::now();
    
    disparityImg.header.frame_id= localization_method;
    //disparityImg.header.frame_id= "camera";
    
    disparityImg.f = 128; //focal length, half of the image width
    disparityImg.T = .14; //baseline, half of the distance between the two cameras
    disparityImg.min_disparity = .44; // f.t/z(depth max)
    disparityImg.max_disparity = 179; // f.t/z(depth min)
    disparityImg.delta_d = .018; //possibly change
    disparityImg.image = *(cv_bridge::CvImage(std_msgs::Header(), "8UC1", disparityImageMat).toImageMsg());
    disparityImg.valid_window.x_offset = 0;
    disparityImg.valid_window.y_offset = 0;
    disparityImg.valid_window.height =  144;
    disparityImg.valid_window.width =  256;
    disparityImg.valid_window.do_rectify =  false; //possibly change
    


    // *** F:DN conversion of opencv images to ros images
    // msgImgL = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgs.left).toImageMsg();
    msgImgR = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgs.right).toImageMsg();
    msgDepth = cv_bridge::CvImage(std_msgs::Header(), "32FC1", imgs.depth).toImageMsg();

    //Stamp messages
    msgCameraInfo.header.stamp = ros::Time::now();
    // msgImgL->header.stamp = msgCameraInfo.header.stamp;
    msgImgR->header.stamp = msgCameraInfo.header.stamp;
    msgDepth->header.stamp =  msgCameraInfo.header.stamp;

    // Set the frame ids
    msgDepth->header.frame_id = localization_method;
    //msgDepth->header.frame_id = "camera";

    //Publish transforms into tf tree
    CameraPosePublisher(imgs.pose, imgs.pose_gt);

    //Publish images
    // imgL_pub.publish(msgImgL);
    imgR_pub.publish(msgImgR);
    depth_pub.publish(msgDepth);
    imgParamL_pub.publish(msgCameraInfo);
    imgParamR_pub.publish(msgCameraInfo);
    imgParamDepth_pub.publish(msgCameraInfo);
    disparity_pub.publish(disparityImg);
#else
    std::vector<uint8_t> rgb_img_data = client->simGetImage(
      0, msr::airlib::ImageCaptureBase::ImageType::Scene
    );
    #if CV_MAJOR_VERSION==3
        cv::Mat rgb_img_mat = cv::imdecode(rgb_img_data, cv::IMREAD_COLOR);
    #else
        cv::Mat rgb_img_mat = cv::imdecode(rgb_img_data, CV_LOAD_IMAGE_COLOR);
    #endif
    msgImgR = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_img_mat).toImageMsg();
    imgR_pub.publish(msgImgR);
    imgParamR_pub.publish(msgCameraInfo);

    // loop_rate.sleep();
#endif    

    ros::spinOnce();
    
    //loop_rate.sleep();
  }
  //poll_frame_thread.join();
  return 0;
}

