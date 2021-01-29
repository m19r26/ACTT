#include <cmath>

#include "actt_pc/pose_2d.hpp"
#include "actt_pc/ackermann_kinematics.hpp"

using namespace actt_auto;

double AckermannKinematics::angular_velocity(
   double velocity,
   double steering_angle,
   double wheelbase) {

   // 返り値(角速度) = 速度*tan(ステア角) / 車体長
   return velocity * std::tan(steering_angle) / wheelbase;
}

Pose2D AckermannKinematics::update(
   const  Pose2D start, 
   double velocity, 
   double steering_angle, 
   double wheelbase, 
   double dt) {

   Pose2D end;  // end = {x, y, theta}

   double dthetadt = angular_velocity(velocity, steering_angle, wheelbase);  // 角速度
   end.theta = start.theta + dthetadt * dt;                                  // 角速度積分->角度

   // x, yそれぞれについて積分
   // dxdt = v * cos(theta)
   // dydt = v * cos(theta)
   if (dthetadt == 0) {
      end.x = start.x + dt * velocity * std::cos(end.theta);
      end.y = start.y + dt * velocity * std::sin(end.theta);
   } else {
      end.x = start.x + (velocity/dthetadt) * (std::sin(end.theta) - std::sin(start.theta));
      end.y = start.y + (velocity/dthetadt) * (std::cos(start.theta) - std::cos(end.theta));
   }

   return end;
}
