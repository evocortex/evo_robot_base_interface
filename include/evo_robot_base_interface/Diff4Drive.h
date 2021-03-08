//###############################################################
//# Copyright (C) 2021, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file MecanumDrive.h
 * @author Evocortex GmbH (MMA)
 *
 * @brief Class for the mecanum drive
 *
 * @version 0.1
 * @date 2021-02-19
 *
 * @copyright Copyright (c) 2020 Evocortex GmbH
 *
 */

#ifndef DIFF4DRIVE_H
#define DIFF4DRIVE_H

#include "evo_robot_base_interface/2dKinematics.h"

namespace evo {

class Diff4Drive : public Drive2d
{
   //-------------
 public:
      Diff4Drive();

   virtual void wheelData2OdomVel(const WheelData& wd, Vel2d& mv) override;
   virtual void wheelData2OdomPose(const WheelData& wd, Pose2d& mp) override;
   virtual void wheelData2OdomPoseInc(const WheelData& wd, WheelData& lwd, Pose2d& mpi) override;

   // sending data
   virtual void cmdVel2wheelData(const Vel2d& cmd_vel, WheelData& cmd_wd) override;

};
} // namespace evo

#endif