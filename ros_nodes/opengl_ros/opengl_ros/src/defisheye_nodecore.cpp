#include "defisheye_nodecore.h"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace opengl_ros;
using namespace std;


DefisheyeNode::DefisheyeNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh_private), it_(nh)
{
    std::string image_in, image_out;
    nh_.param<std::string>("image_in"  , image_in  , "/video/image_raw");
    nh_.param<std::string>("image_out" , image_out  , "/camera/image_raw");

    imagePublisher_  = it_.advertise(image_out, 1);
    imageSubscriber_ = it_.subscribe(image_in , 1, &DefisheyeNode::imageCallback, this);
    // imageSubscriber_ = it_.subscribe("/theta_s_uvc/image_raw" , 1, &DefisheyeNode::imageCallback, this);

    int width,height;
    nh_.param<int>("width" , width , 1280);
    nh_.param<int>("height", height, 720);

    std::string vertexShader, fragmentShader;
    nh_.param<std::string>("vertex_shader"  , vertexShader  , "/home/nvidia/openvslam/ros/src/opengl_ros/opengl_ros/shader/defisheye_vert.glsl");
    nh_.param<std::string>("fragment_shader", fragmentShader, "/home/nvidia/openvslam/ros/src/opengl_ros/opengl_ros/shader/defisheye_frag.glsl");

    renderer_ = std::make_unique<cgs::SimpleRenderer>(
        width, height, 
        vertexShader, fragmentShader
    );

    output_.create(height, width, CV_8UC3);

    resized_image_.create(height, width, CV_8UC3);

    resized_output_.create(int(height*(8/9)), width, CV_8UC3);

    left_roi.create(resized_output_.rows, resized_output_.cols*0.25, CV_8UC3);
    right_roi.create(resized_output_.rows, resized_output_.cols*0.75, CV_8UC3);

    left_roi2.create(resized_output_.rows, resized_output_.cols*0.5, CV_8UC3);
    right_roi2.create(resized_output_.rows, resized_output_.cols*0.5, CV_8UC3);
}

void DefisheyeNode::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
        return;
    }

    auto start = std::chrono::system_clock::now();

    const auto& image = cv_ptr->image;

    cv::resize(image,resized_image_,cv::Size(int(output_.cols),int(output_.rows))); // 1280,720 -> 1920,1080
    renderer_->render(output_, resized_image_); 

    // renderer_->render(output_, image); //renderer에는 (1920,1080으로 넣어준다)

    // cout<<output_.cols<<endl;
    // cout<<output_.rows<<endl;
    // cout<<int(output_.rows*8/9)<<endl;

    cv::resize(output_,resized_output_,cv::Size(int(output_.cols),int(output_.rows*8/9))); // 1920,1080 -> 1920,960

    // cout<<resized_output_.size()<<endl;

    cv::Rect roi_1(0, 0, resized_output_.cols*0.25, resized_output_.rows);
    cv::Rect roi_2(resized_output_.cols*0.25, 0, resized_output_.cols*0.75, resized_output_.rows);

    // cout<<roi_1.size()<<endl;
    // cout<<roi_2.size()<<endl;
    //cout<<"a"<<endl;

    resized_output_(roi_1).copyTo(left_roi);
    resized_output_(roi_2).copyTo(right_roi);

    cv::hconcat(right_roi,left_roi,resized_output_);

    // cout<<"a"<<endl;

    cv::Rect roi_3(0, 0, resized_output_.cols*0.5, resized_output_.rows);
    cv::Rect roi_4(resized_output_.cols*0.5, 0, resized_output_.cols*0.5, resized_output_.rows);

    // cout<<left_roi2.size()<<endl;
    // cout<<right_roi2.size()<<endl;

    // cout<<roi_3.size()<<endl;
    // cout<<roi_4.size()<<endl;

    // cout<<"a"<<endl;

    resized_output_(roi_3).copyTo(left_roi2);
    resized_output_(roi_4).copyTo(right_roi2);

    // cout<<"a"<<endl;

    cv::hconcat(right_roi2,left_roi2,resized_output_);

    // cout<<"a"<<endl;

    resized_output_ = resized_output_+50;

    // cout<<"a"<<endl;

    ROS_DEBUG_STREAM(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count() << "ns";
    );

    //Publish
    cv_bridge::CvImage outImage;
    outImage.header = cv_ptr->header;
    outImage.encoding = cv_ptr->encoding;
    outImage.image = resized_output_;
    imagePublisher_.publish(outImage.toImageMsg());
}

void DefisheyeNode::run()
{
    ros::spin();
}
