#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/opengl.hpp>
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

//cv::ogl::Texture2D tex, tex_edges;

cv::Ptr<cv::cuda::CannyEdgeDetector> canny;

int lowThreshold = 30;
int ratio_pos = 100;
double ratio = 2.0;
int blurring = 5;
const int kernel_size = 3;


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    frame_cuda_color.upload(cv_ptr->image);
    cv::cuda::cvtColor(frame_cuda_color, frame_cuda, cv::COLOR_RGB2GRAY);
    frame_cuda_color.upload(cv_bridge::toCvShare(msg, "rgb8")->image);
    //tex.copyFrom(frame_cuda);
    //tex.bind();
    //frame_cuda.download(frame);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  cv::cuda::bilateralFilter(
      frame_cuda, blurred_frame_cuda, blurring, 75, 75);
  //blurred_frame_cuda.download(blurred_frame);

            
  canny->detect(blurred_frame_cuda,
      detected_edges_cuda);

  // detected_edges_cuda.download(detected_edges);
  //tex_edges.copyFrom(detected_edges_cuda);
  //tex_edges.bind();

  double ret = cv::getWindowProperty("Original image", cv::WND_PROP_OPENGL);
  std::cout << "Opengl original: " << ret << std::endl;

  cv::imshow("Edge map", detected_edges_cuda);
  cv::imshow("Original image", frame_cuda);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_opencv_cuda");
  ros::NodeHandle nh;

  cv::namedWindow("Edge Map", cv::WINDOW_OPENGL);
  cv::namedWindow("Original image", cv::WINDOW_OPENGL);
  //cv::setWindowProperty("Edge Map", cv::WND_PROP_OPENGL, cv::WINDOW_OPENGL);
  //cv::setWindowProperty("Original image", cv::WND_PROP_OPENGL,
  //                      cv::WINDOW_OPENGL);
  double ret = cv::getWindowProperty("Edge Map", cv::WND_PROP_OPENGL);
  std::cout << "Opengl: " << ret << std::endl;

  canny = cv::cuda::createCannyEdgeDetector(lowThreshold, lowThreshold * ratio,
                                            kernel_size, false);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub =
      it.subscribe("/csi_cam_0/image_raw", 1, imageCallback);

  sleep(2);
  ros::Rate r(100); // 10 hz
  while(ros::ok()) {
    ros::spinOnce();
    //cv::imshow("Edge map", detected_edges_cuda);
    //cv::imshow("Original image", frame_cuda);
    // cv::imshow("Edge map", tex_edges);
    // cv::imshow("Original image", tex);
    cv::waitKey(30);
    r.sleep();
  }
  return 0;
}
