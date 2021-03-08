//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file MecanumDrive.cpp
 * @author Evocortex GmbH (MMA)
 *
 * @brief Class for the mecanum drive
 *
 * @version 0.2
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2020 Evocortex GmbH
 *
 */

#include "evo_robot_base_interface/MecanumDrive.h"

namespace evo {

MecanumDrive::MecanumDrive() : Drive2d()
{
   evo::log::init("");
   _logger_prefix = std::string("MecanumDrive: ");
}


void MecanumDrive::wheelData2OdomVel(const WheelData& wd, Vel2d& vel)
{
   vel._x_ms = _rpm2ms * (-wd.front_left + wd.front_right - wd.back_left + wd.back_right) / 4.0;
   vel._y_ms = _rpm2ms * ( wd.front_left + wd.front_right - wd.back_left - wd.back_right) / 4.0;
   vel._yaw_rads = _rpm2ms * (wd.front_left + wd.front_right + wd.back_left + wd.back_right) / (4.0 * _wheel_separation_sum_in_m);
}

void MecanumDrive::wheelData2OdomPose(const WheelData& wd, Pose2d& pose)
{
   pose._x_m = _rot2m * (-wd.front_left + wd.front_right - wd.back_left + wd.back_right) / 4.0;
   pose._y_m = _rot2m * ( wd.front_left + wd.front_right - wd.back_left - wd.back_right) / 4.0;
   pose._yaw_rad = _rot2m * (wd.front_left + wd.front_right + wd.back_left + wd.back_right) / (4.0 * _wheel_separation_sum_in_m);
}

void MecanumDrive::wheelData2OdomPoseInc(const WheelData& wd, WheelData& lwd, Pose2d& poin)
{
   WheelData inc_wd = wd;
   inc_wd.back_left -= lwd.back_left;
   inc_wd.back_right -= lwd.back_right;
   inc_wd.front_left -= lwd.front_left;
   inc_wd.front_right -= lwd.front_right;

   wheelData2OdomPose(inc_wd, poin);

   lwd = wd;   
}


void MecanumDrive::cmdVel2wheelData(const Vel2d& cmd_vel, WheelData& cmd_wd)
{
   cmd_wd.front_left = _ms2rpm * (-cmd_vel._x_ms + cmd_vel._y_ms + (_wheel_separation_sum_in_m * cmd_vel._yaw_rads));
   cmd_wd.back_left  = _ms2rpm * (-cmd_vel._x_ms - cmd_vel._y_ms + (_wheel_separation_sum_in_m * cmd_vel._yaw_rads));

   cmd_wd.front_right =_ms2rpm * ( cmd_vel._x_ms + cmd_vel._y_ms + (_wheel_separation_sum_in_m * cmd_vel._yaw_rads));
   cmd_wd.back_right = _ms2rpm * ( cmd_vel._x_ms - cmd_vel._y_ms + (_wheel_separation_sum_in_m * cmd_vel._yaw_rads));
}

} // namespace evo
