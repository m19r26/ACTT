#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#define TYPE "AUTO"
//#define TYPE "AUTO-NAV"

int main(int argc, char** argv) {
   ros::init(argc, argv, "actt_type_changer");
   ros::NodeHandle n;

   ros::Publisher msg_pub = n.advertise<std_msgs::String>("chatter", 1000);

   ros::Rate loop_rate(0.5);

   //std::string type;
//   std::string TYPE;
//   int type;
//   n.getParam("Type", type);
   //type = "AUTO";

   int count = 0, m = 1, i = 0;
   std_msgs::String msg;
   std::stringstream ss;

   while(m) {

      loop_rate.sleep();
      ss << TYPE << count;
      msg.data = ss.str();
      ROS_INFO("msg :: %s", msg.data.c_str());

      msg_pub.publish(msg);

      i++;
      if (i == 1) m = 0;
   }
   ros::spin();
   return 0;
}
