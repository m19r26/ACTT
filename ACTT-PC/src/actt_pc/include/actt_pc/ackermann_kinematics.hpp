#pragma once

#include "actt_pc/pose_2d.hpp"

namespace actt_auto {

class AckermannKinematics {

  public:

    static double angular_velocity(
        double velocity,
        double steering_angle,
        double wheelbase);

    static Pose2D update(
        const Pose2D start, 
        double velocity, 
        double steering_angle, 
        double wheelbase, 
        double dt);

};

}
