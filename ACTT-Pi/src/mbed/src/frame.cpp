#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <signal.h>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <unistd.h>

cv::VideoCapture cap(0);

void signal_handler(int signum) {
   cap.release();
   ros::shutdown();
}

int main(int argc, char** argv) {
   ros::init(argc, argv, "frame_pub");
   ros::NodeHandle nh;

   image_transport::ImageTransport it(nh);
   image_transport::Publisher img_pub = it.advertise("image_raw", 10);

   int width  = 640;
   int height = 480;

   cv::Mat frame;
   cap.set(cv::CAP_PROP_FRAME_WIDTH,  width);
   cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

   if(!cap.isOpened()) ROS_ERROR("Cannot Open.");

   cv::Mat intrinsic, distortion;

   if(width == 320 && height == 240) {
      intrinsic = (cv::Mat_<double>(3,3)<<263.025899, 0., 162.62139, 0., 261.364368, 112.415473, 0., 0., 1.);
      distortion = (cv::Mat_<double>(1,5)<<-0.68401, 0.317224, -0.00449, 0.005869, 0.);
   }
   if(width == 640 && height == 480) {
      intrinsic = (cv::Mat_<double>(3,3)<<361.885249, 0., 333.527005, 0., 363.0478, 227.509681, 0., 0., 1.);
      distortion = (cv::Mat_<double>(1,5)<<-0.306587, 0.063345, -0.000768, 0.000574, 0.);
   }

   cv::Mat MapX, MapY;
   cv::Mat mapR = cv::Mat::eye(3, 3, CV_64F);
   cv::Mat new_intrinsic = cv::getOptimalNewCameraMatrix(intrinsic, distortion, cv::Size(width, height), 0);

   cv::initUndistortRectifyMap(intrinsic, distortion, mapR, new_intrinsic, cv::Size(width, height), CV_32FC1, MapX, MapY);

   ros::Rate looprate(15);

   while(ros::ok) {
      cap >> frame;

      cv::remap(frame, frame, MapX, MapY, cv::INTER_LINEAR);

      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      img_pub.publish(msg);

      ros::spinOnce();
      looprate.sleep();

      signal(SIGINT, signal_handler);
   }
   return 0;
}
