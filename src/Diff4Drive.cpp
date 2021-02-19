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

#include "evo_robot_base_interface/Diff4Drive.h"

namespace evo {
// TODO: Check maths
Diff4Drive::Diff4Drive() : Drive2d()
{
   evo::log::init("");
   _logger_prefix = std::string("Diff4Drive: ");
}


void Diff4Drive::wheelData2OdomVel(const WheelData& wd, Vel2d& mv)
{
   mv._x_ms = _rpm2ms * (-wd.front_left + wd.front_right - wd.back_left + wd.back_right) / 4.0;
   mv._y_ms = 0;
   mv._yaw_rads = _rpm2ms * (wd.front_left + wd.front_right + wd.back_left + wd.back_right) / (4.0 * _wheel_separation_sum_in_m);
}

void Diff4Drive::wheelData2OdomPose(const WheelData& wd, Pose2d& mp)
{
   mp._x_m = _rot2m * (-wd.front_left + wd.front_right - wd.back_left + wd.back_right) / 4.0;
   mp._y_m = 0;
   mp._yaw_rad = _rot2m * (wd.front_left + wd.front_right + wd.back_left + wd.back_right) / (4.0 * _wheel_separation_sum_in_m);
}

void Diff4Drive::wheelData2OdomPoseInc(const WheelData& wd, WheelData& lwd, Pose2d& mpi)
{
   WheelData inc_wd = wd;
   inc_wd.back_left -= lwd.back_left;
   inc_wd.back_right -= lwd.back_right;
   inc_wd.front_left -= lwd.front_left;
   inc_wd.front_right -= lwd.front_right;

   wheelData2OdomPose(inc_wd, mpi);

   lwd = wd;   
}


void Diff4Drive::cmdVel2wheelData(const Vel2d& cmd_vel, WheelData& cmd_wd)
{
   // TODO: Check if this works on the robots
   cmd_wd.front_left = _ms2rpm * (-cmd_vel._x_ms - (_wheel_separation_sum_in_m * cmd_vel._yaw_rads));
   cmd_wd.back_left  = _ms2rpm * (-cmd_vel._x_ms - (_wheel_separation_sum_in_m * cmd_vel._yaw_rads));

   cmd_wd.front_right =_ms2rpm * (-cmd_vel._x_ms + (_wheel_separation_sum_in_m * cmd_vel._yaw_rads));
   cmd_wd.back_right = _ms2rpm * (-cmd_vel._x_ms + (_wheel_separation_sum_in_m * cmd_vel._yaw_rads));
}

} // namespace evo
