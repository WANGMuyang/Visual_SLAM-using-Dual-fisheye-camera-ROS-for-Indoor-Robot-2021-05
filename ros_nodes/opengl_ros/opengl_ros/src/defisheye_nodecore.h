#ifndef DEFISHEYE_NODECORE_H
#define DEFISHEYE_NODECORE_H

#include <memory>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include "simple_renderer.h"

namespace opengl_ros {

class DefisheyeNode
{
    //Handles
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    //Publishers and Subscribers
    image_transport::Publisher imagePublisher_;
    image_transport::Subscriber imageSubscriber_;

    //Other members 
    std::unique_ptr<cgs::SimpleRenderer> renderer_;

    cv::Mat resized_image_;

    cv::Mat output_;

    cv::Mat resized_output_;

    cv::Mat left_roi;
    cv::Mat right_roi;

    cv::Mat left_roi2;
    cv::Mat right_roi2;

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

public:
    DefisheyeNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    void run();
};

} //opengl_ros

#endif