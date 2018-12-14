#define TELEOP_DURATION 2.0

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>

// opencv
#include <opencv2/highgui/highgui.hpp>

// package related headers
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "common/Common.hpp"

#include <iostream>
#include <math.h>

//std::string localization_method;
msr::airlib::CarRpcLibClient* client;

const std::string camera_front_center = "front_center";
//const std::string camera_front_left = "front_left";
//const std::string camera_front_right = "front_right";
//const std::string camera_fpv = "fpv";
//const std::string camera_back_center = "back_center";

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

    CameraParam.header.frame_id = "camera";
//    CameraParam.header.frame_id = localization_method;

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


int main(int argc, char** argv) {

    // Start ROS
    ros::init(argc, argv, "airsim_car_driver");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(60);

    // Publishers
    image_transport::ImageTransport it(nh);

    image_transport::Publisher imgR_pub = it.advertise("front_center/image_raw", 1);

    ros::Publisher imgParamR_pub = nh.advertise<sensor_msgs::CameraInfo>("front_center/camera_info", 1);

    // ROS messages
    sensor_msgs::CameraInfo msgCameraInfo;
    sensor_msgs::ImagePtr msgImgR;

    // Parameters for communicating with Airsim
    std::string ip_addr;
    int portParam;
    ros::param::param<std::string>("~Airsim_ip", ip_addr,"localhost");
    ros::param::param<int>("~Airsim_port", portParam, 0);
    uint16_t port = portParam;

    // this connects us to the car
    if(!port) {
        client = new msr::airlib::CarRpcLibClient(ip_addr);
    }
    else {
        client = new msr::airlib::CarRpcLibClient(ip_addr, port);
    }

    client->confirmConnection();
    client->enableApiControl(true); // disables manual control

    // Verbose
    ROS_INFO("Image publisher started! Connecting to:");
    ROS_INFO("IP: %s", ip_addr.c_str());
    ROS_INFO("Port: %d", port);

    msgCameraInfo = getCameraParams();
    
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 100,
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

    // *** F:DN end of communication with simulator (Airsim)

    while(ros::ok()) {

    std::vector<uint8_t> rgb_img_data = client->simGetImage(
            camera_front_center, msr::airlib::ImageCaptureBase::ImageType::Scene);

    #if CV_MAJOR_VERSION==3
        cv::Mat rgb_img_mat = cv::imdecode(rgb_img_data, cv::IMREAD_COLOR);
    #else
        cv::Mat rgb_img_mat = cv::imdecode(rgb_img_data, CV_LOAD_IMAGE_COLOR);
    #endif
    msgImgR = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_img_mat).toImageMsg();
    imgR_pub.publish(msgImgR);
    imgParamR_pub.publish(msgCameraInfo);

    ros::spinOnce();

    //loop_rate.sleep();
    }
    return 0;
}

