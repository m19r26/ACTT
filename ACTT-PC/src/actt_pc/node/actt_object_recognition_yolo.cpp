#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <sensor_msgs/LaserScan.h>

int car_cnt = 0, RtoL = 32;          //画角両端の角度分
int scan_deg = 640 / RtoL;                          //frameのwidthとLiDARのdegをマッチさせる.

std::vector<float> scan_dis(RtoL);
std::vector<int>   car_pos(10);

void YoloCallback(const darknet_ros_msgs::BoundingBoxes& yolo) {
   int size = yolo.bounding_boxes.size();

   for(int i=0; i<size; i++) {
      if(yolo.bounding_boxes[i].Class == "car") {         //画像上部で検知した場合, 例外処理をいれる(y座標算出)
         car_pos[car_cnt] = yolo.bounding_boxes[i].xmax - yolo.bounding_boxes[i].xmin;    //carの中心x座標算出
         car_cnt++;
         ROS_INFO("class : %d", car_pos[car_cnt]);
      }
   }
//   ROS_INFO("class : %d", yolo->bounding_boxes[0].Class);
//   ROS_INFO("class : %d", size);
}

void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
/*
   if(car_cnt > 0) {
      for(int j=0; j<car_cnt; j++) {
         int car_lidar_pos = car_pos[car_cnt] / scan_deg;     //carの中心座標に対するLiDARレーザの角度.
         scan_dis[j] = msg->ranges[car_lidar_pos];
      }
   }

   car_cnt = 0;
*/

   int num=0;
   double old_msg = 2.0;

   for(int i=0; i<90; i++) {
      if(old_msg > msg->ranges[i]){
         old_msg = msg->ranges[i];
         num = i;
      }
   }

   for(int j=270; j<360; j++) {
      if(old_msg > msg->ranges[j]){
         old_msg = msg->ranges[j];
         num = j;
      }
   }
   ROS_INFO("num : rear-ultrasonic,  %d : %0.3f", num, old_msg);
}

int main(int argc, char** argv) {
   ros::init(argc, argv, "actt_object_recognition_yolo");
   ros::NodeHandle n;

   ROS_INFO("###--------------------------------------------###");
   ROS_INFO("###--------------------------------------------###");
   ROS_INFO("###--------------------------------------------###");
   ROS_INFO("###--------------------------------------------###");
   ROS_INFO("###--------------------------------------------###");
   ros::Subscriber yolo_sub = n.subscribe("/darknet_ros/bounding_boxes", 10, YoloCallback);
   ros::Subscriber scan_sub = n.subscribe("/scan", 10, ScanCallback);
   //yolo_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

   ros::spin();
   return 0;
}
