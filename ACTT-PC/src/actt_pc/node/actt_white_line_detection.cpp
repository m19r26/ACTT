#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <math.h>

#define TYPE "AWLD"   //actt_joyノード用(モード切り替え)

cv::Mat frame, frame_c;

ros::Publisher servo_pub, msg_pub;
geometry_msgs::Vector3 servo;

unsigned short int white_flag = 0;

const char         point_num = 16;
unsigned char      pre_left_point = 0, pre_right_point = 0, i = 0;
unsigned short int left_line = 0, right_line = 0;
unsigned short int left_start_point,  left_final_point,  left_mid_point;
unsigned short int right_start_point, right_final_point, right_mid_point;
unsigned short int mid_point[point_num];

std::vector<std::vector<float> > date(point_num, std::vector<float>(2));  //date[i][x,y]

float         radian, degree, f_degree, b_degree;
float         dis;
unsigned char mid_length[15] = {}, line_num = 0, pre_line_num = 0;

std_msgs::String msg_type;
std::stringstream ss;

int count = 0, n = 0;
int gain_flag, gain_cnt = 0;

int nnn=0, mmm=0;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

   if(n == 0){     //一度のみ
      ss << TYPE << count;

      msg_type.data = ss.str();
      msg_pub.publish(msg_type);
   }
   n = 1;

   try {
      frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
   }
   catch(cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
   }

   frame_c = frame;
   
   cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
   cv::threshold(frame, frame, 150, 255, cv::THRESH_BINARY);

   //白線検知
   for(int y=frame.rows*2/3; y<frame.rows; y+=10) {

      for(int x=frame.cols/2; x>=0; x--) {
         //白ドットをカウント. 2以上で白線とみなす.
         if(frame.at<unsigned char>(y, x) == 255) {    //白の場合
            if(left_line == 0) left_start_point = x;
            left_line++;

            if(x == 0 && white_flag == 0) {
               left_mid_point = (left_start_point + 0) / 2;   //最端まで白線のとき例外処理
               cv::circle(frame, cv::Point(left_mid_point, y), 2, cv::Scalar(0,0,0), -1, cv::LINE_AA);
               white_flag++;
            }
         } else if(pre_left_point == 0) {              //黒かつ１つ前も黒の場合
            if(left_line > 2 && white_flag == 0) {
               left_final_point = x - 1;
               left_mid_point   = (left_start_point + left_final_point) / 2;

               cv::circle(frame, cv::Point(left_mid_point, y), 2, cv::Scalar(0,0,0), -1, cv::LINE_AA);
               white_flag++;
            } else left_line = 0;
         }
         pre_left_point = frame.at<unsigned char>(y, x);
         if(white_flag == 0) {
            left_mid_point = 0;
            cv::circle(frame, cv::Point(left_mid_point, y), 2, cv::Scalar(0,0,0), -1, cv::LINE_AA);
         }
      }
      left_line  = 0;
      white_flag = 0;

      //同様に逆サイドの検出を行う.
      for(int _x=frame.cols/2; _x<frame.cols; _x++) {
         //白ドットをカウント. 2以上で白線とみなす.
         if(frame.at<unsigned char>(y, _x) == 255) {    //白の場合
            if(right_line == 0) right_start_point = _x;
            right_line++;

            if(_x == frame.cols-1 && white_flag == 0) {
               right_mid_point = (right_start_point + frame.cols - 1) / 2; //最端まで白線のときの例外処理
               cv::circle(frame, cv::Point(right_mid_point, y), 2, cv::Scalar(0,0,0), -1, cv::LINE_AA);
               white_flag++;
            }
         } else if(pre_right_point == 0 || _x == frame.cols-1) {             //黒かつ１つ前も黒の場合
            if(right_line > 2 && white_flag == 0) {
               right_final_point = _x - 1;
               right_mid_point   = (right_start_point + right_final_point) / 2;

               cv::circle(frame, cv::Point(right_mid_point, y), 2, cv::Scalar(0,0,0), -1, cv::LINE_AA);
               white_flag++;
            } else right_line = 0;
         }
         pre_right_point = frame.at<unsigned char>(y, _x);
         if(white_flag == 0) right_mid_point = frame.cols;
      }

      right_line = 0;
      white_flag = 0;

      //両サイドの中心を算出.
      mid_point[i] = (left_mid_point + right_mid_point) / 2;

      date[i][0] = (float)mid_point[i];
      date[i][1] = (float)y;

      if(i > 0) cv::line(frame, cv::Point(mid_point[i-1], y-10), cv::Point(mid_point[i], y), cv::Scalar(255,255,255), 2, 4);

      if(0 < i && i < 4) {
         if(left_mid_point == 0 || right_mid_point == frame.cols) gain_cnt++;
         nnn++;
         radian = std::atan2(((double)date[i][1] - date[i-1][1]), (double)(date[i][0] - date[i-1][0]));
         f_degree = (radian * 180) / 3.141592;
         if(50  > f_degree) f_degree = 50;
         if(130 < f_degree) f_degree = 130;
         degree += f_degree / 3.0 * 0.3;
      }

      if(i > 8) {  //i > 8
         //白線を半分見失った時, ゲインを上げる.
         if(left_mid_point == 0 || right_mid_point == frame.cols) gain_cnt++;

         mmm++;
         radian = std::atan2(((double)date[i][1] - date[i-1][1]), (double)(date[i][0] - date[i-1][0]));
         b_degree = (radian * 180) / 3.141592;
         if(50  > b_degree) b_degree = 50;
         if(130 < b_degree) b_degree = 130;
         degree += b_degree / 7.0 * 0.7;
      }
      
      i++;
   }

   ROS_INFO("deg : %f", degree);

   if(gain_cnt > 4) gain_flag = 1;

   servo.x = degree;
   servo.z = gain_flag;
   servo_pub.publish(servo);

   i = 0;
   degree = 0;
   f_degree = 0;
   b_degree = 0;
   gain_flag = 0;
   gain_cnt  = 0;

   nnn=0;
   mmm=0;

   cv::imshow("image", frame);
   //cv::imshow("image1", frame_c);
   cv::waitKey(1);
}

int main(int argc, char** argv) {
   ros::init(argc, argv, "actt_white_line_detection");
   ros::NodeHandle nh;

   image_transport::ImageTransport it(nh);
   image_transport::Subscriber img_sub = it.subscribe("/image_exp", 10, imageCallback);
   servo_pub = nh.advertise<geometry_msgs::Vector3>("cmd_servo", 1);
   msg_pub   = nh.advertise<std_msgs::String>("chatter", 1000);

   ros::spin();
   return 0;
}
