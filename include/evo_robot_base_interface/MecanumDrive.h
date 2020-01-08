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
 * @version 0.1
 * @date 2019-08-09
 *
 * @copyright Copyright (c) 2019 Evocortex GmbH
 *
 */

#ifndef MECANUMDRIVE_H
#define MECANUMDRIVE_H

#include "evo_motor_shield_interface/Motor.h"

#include "evo_logger/log/Logger.h"

namespace evo {

struct MecanumVel
{
   double _x_ms     = 0.0;
   double _y_ms     = 0.0;
   double _yaw_rads = 0.0;
};

struct MecanumPose
{
   double _x_m     = 0.0;
   double _y_m     = 0.0;
   double _yaw_rad = 0.0;

   void reset()
   {
      // todo: test this line
      *this = MecanumPose();
   }

   void updatePoseFromVel(const MecanumVel& vel, const double cycle_time)
   {
      this->_x_m += (std::cos(this->_yaw_rad) * vel._x_ms -
                     std::sin(this->_yaw_rad) * vel._y_ms) *
                    cycle_time;
      this->_y_m += (std::sin(this->_yaw_rad) * vel._x_ms +
                     std::cos(this->_yaw_rad) * vel._y_ms) *
                    cycle_time;
      this->_yaw_rad += vel._yaw_rads * cycle_time;
   }

   void updatePoseFromIncrement(const MecanumPose& inc)
   {
      this->_x_m +=  std::cos(this->_yaw_rad) * inc._x_m - 
                     std::sin(this->_yaw_rad) * inc._y_m;
      this->_y_m +=  std::sin(this->_yaw_rad) * inc._x_m + 
                     std::cos(this->_yaw_rad) * inc._y_m;
      this->_yaw_rad += inc._yaw_rad;
   }
};

struct MecanumCovariance
{
   double cov_pos_x   = 0.0;
   double cov_pos_y   = 0.0;
   double cov_pos_yaw = 0.0;
   double cov_vel_x   = 0.0;
   double cov_vel_y   = 0.0;
   double cov_vel_yaw = 0.0;
};

enum MOTOR_MAPPING_MECANUM
{
   NO_POSITION = 0,
   FRONT_LEFT,
   FRONT_RIGHT,
   BACK_RIGHT,
   BACK_LEFT
};

class MecanumDrive
{
 private:
   // references to motors of motor handler
   std::shared_ptr<evo_mbed::Motor> _motor_front_left;
   std::shared_ptr<evo_mbed::Motor> _motor_front_right;
   std::shared_ptr<evo_mbed::Motor> _motor_back_left;
   std::shared_ptr<evo_mbed::Motor> _motor_back_right;

   double _wheel_radius_in_m;
   double _wheel_separation_length_in_m;
   double _wheel_separation_width_in_m;
   double _wheel_separation_sum_in_m;

   double _ms2rpm;
   double _rpm2ms;
   double _rot2m;

   std::string _logger_prefix;

   bool _is_initialized;
   bool _verbose;

   double _last_rotation_front_left;
   double _last_rotation_front_right;
   double _last_rotation_back_left;
   double _last_rotation_back_right;
   //-------------
 public:
   MecanumDrive();

   bool checkInitState();
   void setMotorRef(std::shared_ptr<evo_mbed::Motor> motor,
                    const uint8_t motor_mapping);

   void setWheelRadiusInM(const double wheel_radius_in_m);
   void setWheelSeparationLengthInM(const double wheel_separation_length_in_m);
   void setWheelSeparationWidthInM(const double wheel_separation_Width_in_m);
   void setWheelDistanceFrontBackInM(const double wheel_distance_front_back_in_m);
   void setWheelDistanceLeftRightInM(const double wheel_distance_left_right_in_m);

   void setTargetSpeed(const MecanumVel& cmd_vel);
   MecanumVel getOdom();
   MecanumPose getPoseIncrement();

   void debugMotorMapping();
};
} // namespace evo
#endif // MECANUMDRIVE_H
