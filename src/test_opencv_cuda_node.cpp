#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/cuda.hpp>
//#include <opencv2/core/opengl.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

#include <cmath>
#include <thread>

#include <iostream>

cv::Mat frame;
cv::Mat detected_edges;
cv::Mat blurred_frame;
cv::cuda::GpuMat frame_cuda, frame_cuda_color, blurred_frame_cuda,
    detected_edges_cuda;

char* camera_topic;

cv::Ptr<cv::cuda::CannyEdgeDetector> canny;

int lowThreshold = 30;
int ratio_pos = 100;
double ratio = 2.0;
int blurring = 5;
const int kernel_size = 3;


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    frame_cuda_color.upload(cv_bridge::toCvShare(msg, "rgb8")->image);
    cv::cuda::cvtColor(frame_cuda_color, frame_cuda, cv::COLOR_RGB2GRAY);
    frame_cuda.download(frame);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
  }
  cv::cuda::bilateralFilter(
      frame_cuda, blurred_frame_cuda, blurring, 75, 75);
  //blurred_frame_cuda.download(blurred_frame);

            
  canny->detect(blurred_frame_cuda,
      detected_edges_cuda);

  detected_edges_cuda.download(detected_edges);

  cv::imshow("Edge map", detected_edges);
  cv::imshow("Original image", frame);
  cv::waitKey(10);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_opencv_cuda");
  ros::NodeHandle nh;

  if(argc < 2) {
    std::cout << "Argument missing...please specify the image topic name..." << std::endl;
    return 0;
  } 
  
  camera_topic = argv[1];

  cv::namedWindow("Edge Map", cv::WINDOW_NORMAL);
  cv::namedWindow("Original image", cv::WINDOW_NORMAL); 

  canny = cv::cuda::createCannyEdgeDetector(lowThreshold, lowThreshold * ratio,
                                            kernel_size, false);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub =
      it.subscribe(camera_topic, 1, imageCallback);

  sleep(1);
  ros::Rate r(100); // 10 hz
  while(ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
