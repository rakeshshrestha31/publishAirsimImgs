#ifndef DRONE_H
#define DRONE_H

#include <stdint.h>
#include <string>
#include <opencv2/opencv.hpp>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
using ImageReq = msr::airlib::VehicleCameraBase::ImageRequest;
using ImageRes = msr::airlib::VehicleCameraBase::ImageResponse;
using ImageTyp = msr::airlib::VehicleCameraBase::ImageType;



//#include "configs.h"
// Control functions

struct image_response {
	cv::Mat left;
	cv::Mat right;
	
	cv::Mat depth;
	cv::Mat planar_depth;
	cv::Mat disparity;
	
	geometry_msgs::Pose pose;
	geometry_msgs::Pose pose_gt; //ground truth
    geometry_msgs::Twist twist;	
    bool valid_data = true;

};


using ImageResponse = msr::airlib::VehicleCameraBase::ImageResponse;

class input_sampler {
public:
	input_sampler();
	input_sampler(const std::string& ip_addr, uint16_t port);
	input_sampler(const std::string& ip_addr, uint16_t port, std::string localization_method);
	~input_sampler();

	// *** F:DN Control functions
	void connect();
	void connect(const std::string& ip_addr, uint16_t port);

	// *** F:DN Odometry functions
    geometry_msgs::Twist twist();

	// *** F:DN Camera functions
	// cv::Mat poll_frame();
	// cv::Mat poll_frame_depth();
	
    void do_nothing();
    void poll_frame();
    struct image_response poll_frame_and_decode();
    struct image_response image_decode();
private:
     std::string localization_method;	
    msr::airlib::MultirotorRpcLibClient * client;
};

#endif

