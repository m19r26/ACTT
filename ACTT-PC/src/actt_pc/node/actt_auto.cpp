#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

#include "actt_pc/pose_2d.hpp"
#include "actt_pc/ackermann_kinematics.hpp"
#include "actt_pc/scan_simulator_2d.hpp"   // RPLiDAR使用時不要

using namespace actt_auto;

class Actt_auto {
   private:
      // tfフレーム名の宣言
      std::string map_frame, base_frame, scan_frame, odom_frame, scanmatcher_frame;

      // 車両状態とパラメータの宣言
      Pose2D pose;                         // pose = {x, y, theta}
      double wheelbase;
      double speed;
      double steering_angle;
      double previous_seconds;
      double scan_distance_to_base_link;
      double max_speed, max_steering_angle;

      // LiDARシミュレータの宣言(RPLiDAR使用時不要)
      ScanSimulator2D scan_simulator;
      double map_free_threshold;

      // ジョイスティックの宣言
      int joy_speed_axis, joy_angle_axis;
      double joy_max_speed;

      // ROSノード宣言
      ros::NodeHandle n;

      // tfをパブリッシュするための宣言
      tf2_ros::TransformBroadcaster br;

      // 車両状態を更新するための宣言
      ros::Timer update_pose_timer;

      // ドライブコマンド, ジョイスティックコマンド, 速度コマンドとLiDARデータをサブスクライブするための宣言
      ros::Subscriber drive_sub;
      ros::Subscriber joy_sub;
      ros::Subscriber cmd_sub;
      ros::Subscriber scan_sub;

      // マップをサブスクライブするための宣言
      ros::Subscriber map_sub;
      bool map_exists = false;

      // 車両状態の更新をサブスクライブするための宣言
      ros::Subscriber pose_sub;
      ros::Subscriber pose_rviz_sub;

      // スキャンデータとオドメトリデータをパブリッシュするための宣言
      bool broadcast_transform;
      ros::Publisher scan_pub;
      ros::Publisher odom_pub;

      ros::Subscriber scan_sub_sample;

      // actt_joyノードモード切り替え用
      std::string TYPE;
      int count = 0, m = 0;
      std_msgs::String msg_type;
      std::stringstream ss;

   public:

      Actt_auto() {
         // ノードハンドルの初期化
         n = ros::NodeHandle("~");

         // 車両状態とドライブコマンドの初期化
         pose = {.x=0, .y=0, .theta=0};
         speed = 0;
         steering_angle = 0;
         previous_seconds = ros::Time::now().toSec();

         // param.yamlに定義したトピック名の取得
         std::string joy_topic, drive_topic, map_topic, scan_topic,
                     pose_topic, pose_rviz_topic, odom_topic, cmd_vel;
         n.getParam("joy_topic",       joy_topic);
         n.getParam("cmd_vel",         cmd_vel);
         n.getParam("drive_topic",     drive_topic);
         n.getParam("map_topic",       map_topic);
         n.getParam("scan_topic",      scan_topic);
         n.getParam("pose_topic",      pose_topic);
         n.getParam("odom_topic",      odom_topic);
         n.getParam("pose_rviz_topic", pose_rviz_topic);

         // param.yamlに定義したtfフレーム名の取得
         n.getParam("map_frame",  map_frame);
         n.getParam("base_frame", base_frame);
         n.getParam("scan_frame", scan_frame);
         n.getParam("odom_frame", odom_frame);
         n.getParam("scanmatcher_frame", scanmatcher_frame);

         // param.yamlに定義した車両パラメータの取得
         int    scan_beams;
         double update_pose_rate, scan_field_of_view, scan_std_dev;
         n.getParam("wheelbase",                  wheelbase);
         n.getParam("update_pose_rate",           update_pose_rate);
         n.getParam("scan_beams",                 scan_beams);
         n.getParam("scan_field_of_view",         scan_field_of_view);
         n.getParam("scan_std_dev",               scan_std_dev);
         n.getParam("map_free_threshold",         map_free_threshold);
         n.getParam("scan_distance_to_base_link", scan_distance_to_base_link);
         n.getParam("max_speed",                  max_speed);
         n.getParam("max_steering_angle",          max_steering_angle);

         // parem.yamlに定義したジョイスティックパラメータの取得
         bool joy;
         n.getParam("joy",            joy);
         n.getParam("joy_speed_axis", joy_speed_axis);
         n.getParam("joy_angle_axis", joy_angle_axis);
         n.getParam("joy_max_speed",  joy_max_speed);

         // param.yamlに定義したtfをパブリッシュするか否かの判断(true)
         n.getParam("broadcast_transform", broadcast_transform);

         // LiDARシミュレータの初期化(RPLiDAR使用時不要)
         scan_simulator = ScanSimulator2D(scan_beams,
                                          scan_field_of_view,
                                          scan_std_dev        );

         // オドメトリメッセージ用のパブリッシャを作成
         //odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic, 1);

         // 車両状態出力用のタイマの開始
         update_pose_timer = n.createTimer(ros::Duration(update_pose_rate), 
                                           &Actt_auto::update_pose, this   );

         // ジョイスティックを使用する場合
         if(joy) joy_sub = n.subscribe(joy_topic, 1, &Actt_auto::joy_callback, this);

         // 速度コマンドのサブスクライブを開始(3Dモデルと同期している)
         cmd_sub = n.subscribe(cmd_vel, 1, &Actt_auto::vel_callback, this);

         // 車両状態のメッセージのサブスクライブを開始
         pose_sub      = n.subscribe(pose_topic, 1, &Actt_auto::pose_callback, this);
         pose_rviz_sub = n.subscribe(pose_rviz_topic, 1, &Actt_auto::pose_rviz_callback, this);

         // 超音波センサAPI用sample
	 //scan_sub_sample = n.subscribe(scan_topic, 1, &Actt_auto::scan_callback, this);
      }

      void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg) {
         ROS_INFO("rear-ultrasonic: %0.3f", msg->ranges[0]);
      }

      // 車両状態更新のコールバック関数
      void update_pose(const ros::TimerEvent&) {
         // 車両状態の更新
         ros::Time timestamp = ros::Time::now();
         double current_seconds = timestamp.toSec();
         // 返り値->速度に基づいて算出されたpose = {x, y, theta}
         pose = AckermannKinematics::update(pose,
                                            speed,
                                            steering_angle,
                                            wheelbase,
                                            current_seconds - previous_seconds);
         previous_seconds = current_seconds;

         // tfを用いた車両状態の変換
         geometry_msgs::Transform t;
         t.translation.x = pose.x;
         t.translation.y = pose.y;
         // オイラー角を用いたクオータニオンの設定
         tf2::Quaternion quat;
         quat.setEuler(0., 0., pose.theta);
         t.rotation.x = quat.x();
         t.rotation.y = quat.y();
         t.rotation.z = quat.z();
         t.rotation.w = quat.w();

         // tfの先頭フレームを追加
/*         geometry_msgs::TransformStamped ts;
         ts.transform       = t;
         ts.header.stamp    = timestamp;
         ts.header.frame_id = odom_frame;
         //ts.header.frame_id = scanmatcher_frame;
         ts.child_frame_id  = base_frame;

         // オドメトリメッセージの作成
         nav_msgs::Odometry odom;
         odom.header.stamp            = timestamp;
         odom.header.frame_id         = odom_frame;
         //odom.header.frame_id         = scanmatcher_frame;
         odom.child_frame_id          = base_frame;
         odom.pose.pose.position.x    = pose.x;
         odom.pose.pose.position.y    = pose.y;
         odom.pose.pose.orientation.x = quat.x();
         odom.pose.pose.orientation.y = quat.y();
         odom.pose.pose.orientation.z = quat.z();
         odom.pose.pose.orientation.w = quat.w();
         odom.twist.twist.linear.x    = speed;
         odom.twist.twist.angular.z   = 
                 AckermannKinematics::angular_velocity(speed, steering_angle, wheelbase); // 角速度

         // 上記オドメトリ情報をパブリッシュ
         if(broadcast_transform) br.sendTransform(ts);
         odom_pub.publish(odom);
*/         // タイヤを動作させるためのステア角の設定
         set_steering_angle(steering_angle, timestamp);
      }

      // ジョイスティック入力による速度制御用のコールバック関数
      void joy_callback(const sensor_msgs::Joy & msg) {
         //set_speed(joy_max_speed * msg.axes[joy_speed_axis]);
         //set_steering_angle(max_steering_angle * msg.axes[joy_angle_axis], ros::Time::now());
      }
      void set_speed(double speed_) {
         speed = std::min(std::max(speed_, -max_speed), max_speed);
      }

      // ステア角をパブリッシュする関数
      void set_steering_angle(double steering_angle_, ros::Time timestamp) {
         steering_angle = std::min(std::max(steering_angle_, -max_steering_angle),
                                   max_steering_angle);

         tf2::Quaternion quat;
         quat.setEuler(0., 0., steering_angle);
         geometry_msgs::TransformStamped ts;
         ts.transform.rotation.x = quat.x();
         ts.transform.rotation.y = quat.y();
         ts.transform.rotation.z = quat.z();
         ts.transform.rotation.w = quat.w();
         ts.header.stamp         = timestamp;
         ts.header.frame_id      = "front_left_hinge";
         ts.child_frame_id       = "front_left_wheel";
         br.sendTransform(ts);
         ts.header.frame_id      = "front_right_hinge";
         ts.child_frame_id       = "front_right_wheel";
         br.sendTransform(ts);
      }

      // rviz上の車両状態更新用のコールバック関数
      void pose_rviz_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg) {
         pose_callback(msg -> pose.pose);
      }

      // 姿勢更新用のコールバック関数
      void pose_callback(const geometry_msgs::Pose & msg) {
         pose.x = msg.position.x;
         pose.y = msg.position.y;
         geometry_msgs::Quaternion q = msg.orientation;
         tf2::Quaternion quat(q.x, q.y, q.z, q.w);
         pose.theta = tf2::impl::getYaw(quat);
      }

      // 速度コマンド入力による速度制御用のコールバック関数
      void vel_callback(const geometry_msgs::Twist & vel) {
         //set_speed(joy_max_speed * vel.linear.x);
         //set_steering_angle(max_steering_angle * vel.angular.z, ros::Time::now());
         // 
      }
};

int main(int args, char ** argv) {
    ros::init(args, argv, "actt_auto");
    Actt_auto a;
    ros::spin();
    return 0;
}
