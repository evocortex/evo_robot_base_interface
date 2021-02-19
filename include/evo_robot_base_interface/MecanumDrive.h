//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file MecanumDrive.h
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

#ifndef MECANUMDRIVE_H
#define MECANUMDRIVE_H

#include "evo_motor_shield_interface/Motor.h"

#include "evo_logger/log/Logger.h"

#include "evo_robot_base_interface/2dKinematics.h"

namespace evo {
class MecanumDrive : public Drive2d
{
 public:
      MecanumDrive();

   virtual void wheelData2OdomVel(const WheelData& wd, Vel2d& vel) override;
   virtual void wheelData2OdomPose(const WheelData& wd, Pose2d& pose) override;
   virtual void wheelData2OdomPoseInc(const WheelData& wd, WheelData& lwd, Pose2d& poin) override;

   // sending data
   virtual void cmdVel2wheelData(const Vel2d& cmd_vel, WheelData& cmd_wd) override;

};
} // namespace evo
#endif // MECANUMDRIVE_H
