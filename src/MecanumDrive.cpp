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
 * @version 0.1
 * @date 2019-08-09
 *
 * @copyright Copyright (c) 2019 Evocortex GmbH
 *
 */

#include "evo_robot_base_interface/MecanumDrive.h"

namespace evo {

MecanumDrive::MecanumDrive() :
    _logger_prefix("MecanumDrive: "), _is_initialized(false),
    _wheel_radius_in_m(0.0), _wheel_separation_length_in_m(0.0),
    _wheel_separation_width_in_m(0.0), _wheel_separation_sum_in_m(0.0),
    _last_rotation_back_left(0.0), _last_rotation_back_right(0.0),
    _last_rotation_front_left(0.0), _last_rotation_front_right(0.0)
{
   evo::log::init("");
}

void MecanumDrive::setMotorRef(std::shared_ptr<evo_mbed::Motor> motor,
                               const uint8_t motor_mapping)
{
   switch(motor_mapping)
   {
   case MOTOR_MAPPING_MECANUM::FRONT_LEFT:
   {
      evo::log::get() << _logger_prefix << "motor front left mapped!" << evo::info;
      _motor_front_left = motor;
      break;
   }
   case MOTOR_MAPPING_MECANUM::FRONT_RIGHT:
   {
      evo::log::get() << _logger_prefix << "motor front right mapped!" << evo::info;
      _motor_front_right = motor;
      break;
   }
   case MOTOR_MAPPING_MECANUM::BACK_RIGHT:
   {
      evo::log::get() << _logger_prefix << "motor back right mapped!" << evo::info;
      _motor_back_right = motor;
      break;
   }
   case MOTOR_MAPPING_MECANUM::BACK_LEFT:
   {
      evo::log::get() << _logger_prefix << "motor back left mapped!" << evo::info;
      _motor_back_left = motor;
      break;
   }
   default:
      evo::log::get() << _logger_prefix
                      << "setMotorRef received invalid mapping value"
                      << "(" << motor_mapping << ")"
                      << "! Shutdown!" << evo::error;
      exit(1);
      break;
   }
}

bool MecanumDrive::checkInitState()
{
   bool motors_connected = true;
   if(_motor_front_left == nullptr)
   {
      motors_connected &= false;
      evo::log::get() << _logger_prefix << "Motor front left not connected!"
                      << evo::warn;
   }
   if(_motor_front_right == nullptr)
   {
      motors_connected &= false;
      evo::log::get() << _logger_prefix << "Motor front right not connected!"
                      << evo::warn;
   }
   if(_motor_back_right == nullptr)
   {
      motors_connected &= false;
      evo::log::get() << _logger_prefix << "Motor back right not connected!"
                      << evo::warn;
   }
   if(_motor_back_left == nullptr)
   {
      motors_connected &= false;
      evo::log::get() << _logger_prefix << "Motor back left not connected!"
                      << evo::warn;
   }

   bool parameters_set = true;
   if(_wheel_radius_in_m <= 0.0)
   {
      parameters_set &= false;
      evo::log::get() << _logger_prefix << "Wheel Radius not set!" << evo::warn;
   }
   if(_wheel_separation_length_in_m <= 0.0)
   {
      parameters_set &= false;
      evo::log::get() << _logger_prefix << "Wheel separation length not set!"
                      << evo::warn;
   }
   if(_wheel_separation_width_in_m <= 0.0)
   {
      parameters_set &= false;
      evo::log::get() << _logger_prefix << "Wheel separation width not set!"
                      << evo::warn;
   }

   _is_initialized = motors_connected && parameters_set;
   evo::log::get() << _logger_prefix << "init state: " << _is_initialized
                   << evo::info;
   return _is_initialized;
}

void MecanumDrive::setWheelRadiusInM(const double wheel_radius_in_m)
{
   _wheel_radius_in_m = wheel_radius_in_m;
   _ms2rpm            = 60.0 / (2.0 * _wheel_radius_in_m * M_PI);
   _rpm2ms            = 1.0 / _ms2rpm;
   _rot2m             = (2.0 * _wheel_radius_in_m * M_PI);
}

void MecanumDrive::setWheelSeparationLengthInM(
    const double wheel_separation_length_in_m)
{
   _wheel_separation_length_in_m = wheel_separation_length_in_m;
   _wheel_separation_sum_in_m =
       _wheel_separation_length_in_m + _wheel_separation_width_in_m;
}

void MecanumDrive::setWheelSeparationWidthInM(
    const double wheel_separation_Width_in_m)
{
   _wheel_separation_width_in_m = wheel_separation_Width_in_m;
   _wheel_separation_sum_in_m =
       _wheel_separation_length_in_m + _wheel_separation_width_in_m;
}

void MecanumDrive::setWheelDistanceFrontBackInM(
    const double wheel_distance_front_back_in_m)
{
   setWheelSeparationLengthInM(wheel_distance_front_back_in_m / 2.0);
}

void MecanumDrive::setWheelDistanceLeftRightInM(
    const double wheel_distance_left_right_in_m)
{
   setWheelSeparationWidthInM(wheel_distance_left_right_in_m / 2.0);
}

void MecanumDrive::setTargetSpeed(const MecanumVel& cmd_vel)
{
   if(_is_initialized)
   {
      float rpm_front_left =
          _ms2rpm * (cmd_vel._x_ms - cmd_vel._y_ms -
                     (_wheel_separation_sum_in_m * cmd_vel._yaw_rads));
      float rpm_back_left =
          _ms2rpm * (cmd_vel._x_ms + cmd_vel._y_ms -
                     (_wheel_separation_sum_in_m * cmd_vel._yaw_rads));

      float rpm_front_right =
          _ms2rpm * (cmd_vel._x_ms + cmd_vel._y_ms +
                     (_wheel_separation_sum_in_m * cmd_vel._yaw_rads));
      float rpm_back_right =
          _ms2rpm * (cmd_vel._x_ms - cmd_vel._y_ms +
                     (_wheel_separation_sum_in_m * cmd_vel._yaw_rads));

      // invert left side
      rpm_front_left *= (-1.0);
      rpm_back_left *= (-1.0);

      if(_verbose)
      {
         // log speeds
      }

      bool error_flag = false;
      if(!_motor_front_left->setTargetSpeed(rpm_front_left))
         error_flag |= true;
      if(!_motor_front_right->setTargetSpeed(rpm_front_right))
         error_flag |= true;

      if(!_motor_back_left->setTargetSpeed(rpm_back_left))
         error_flag |= true;
      if(!_motor_back_right->setTargetSpeed(rpm_back_right))
         error_flag |= true;

      if(error_flag)
      {
         evo::log::get() << _logger_prefix << "Couldn't set target speed!"
                         << evo::error;
      }
   }
   else
   {
      evo::log::get() << _logger_prefix << "not initialized yet! -> check"
                      << evo::error;
      checkInitState();
   }
}

MecanumVel MecanumDrive::getOdom()
{
   if(_is_initialized)
   {
      double rpm_front_left  = _motor_front_left->getSpeedRPM();
      double rpm_front_right = _motor_front_right->getSpeedRPM();

      double rpm_back_left  = _motor_back_left->getSpeedRPM();
      double rpm_back_right = _motor_back_right->getSpeedRPM();
      //-------------

      // invert left side
      rpm_front_left *= (-1.0);
      rpm_back_left *= (-1.0);

      MecanumVel odom;
      odom._x_ms =
          _rpm2ms *
          (rpm_front_left + rpm_front_right + rpm_back_left + rpm_back_right) / 4.0;
      odom._y_ms =
          _rpm2ms *
          (-rpm_front_left + rpm_front_right + rpm_back_left - rpm_back_right) / 4.0;
      odom._yaw_rads =
          _rpm2ms *
          (-rpm_front_left + rpm_front_right - rpm_back_left + rpm_back_right) /
          (4.0 * _wheel_separation_sum_in_m);
      return odom;
   }
   else
   {
      evo::log::get() << _logger_prefix << "not initialized yet! -> check"
                      << evo::error;
      checkInitState();
   }
}

MecanumPose MecanumDrive::getPoseIncrement()
{
   if(_is_initialized)
   {
      // get current wheel rotation
      double rotation_front_left  = _motor_front_left->getRevolutions();
      double rotation_front_right = _motor_front_right->getRevolutions();

      double rotation_back_left  = _motor_back_left->getRevolutions();
      double rotation_back_right = _motor_back_right->getRevolutions();

      // dif last known rotation
      double rot_dif_FL = rotation_front_left - _last_rotation_front_left;
      double rot_dif_FR = rotation_front_right - _last_rotation_front_right;
      double rot_dif_BL = rotation_back_left - _last_rotation_back_left;
      double rot_dif_BR = rotation_back_right - _last_rotation_back_right;

      // save current rotation
      _last_rotation_front_left  = rotation_front_left;
      _last_rotation_front_right = rotation_front_right;
      _last_rotation_back_left   = rotation_back_left;
      _last_rotation_back_right  = rotation_back_right;

      // invert left side
      rot_dif_FL *= (-1.0);
      rot_dif_BL *= (-1.0);

      // calc pose increment
      MecanumPose odom;
      odom._x_m = _rot2m * (rot_dif_FL + rot_dif_FR + rot_dif_BL + rot_dif_BR) / 4.0;
      odom._y_m =
          _rot2m * (-rot_dif_FL + rot_dif_FR + rot_dif_BL - rot_dif_BR) / 4.0;
      odom._yaw_rad = _rot2m * (-rot_dif_FL + rot_dif_FR - rot_dif_BL + rot_dif_BR) /
                      (4.0 * _wheel_separation_sum_in_m);
      return odom;
   }
   else
   {
      evo::log::get() << _logger_prefix << "not initialized yet! -> check"
                      << evo::error;
      checkInitState();
   }
}

/// this function should not exist in this class. rather a function that discovers
/// all motors in motor handler
void MecanumDrive::debugMotorMapping()
{
   if(_is_initialized)
   {
      bool error_flag                 = false;
      const double test_rpm           = 50;
      const double test_sleep_time_us = 1000;

      evo::log::get() << _logger_prefix << "Set positive speed for front left"
                      << evo::warn;
      for(int i = 0; i < test_sleep_time_us; i++)
      {
         if(_motor_front_left->setTargetSpeed(test_rpm) != evo_mbed::RES_OK)
            error_flag |= true;
      }

      // _motor_front_right->setOperationStatus()
      evo::log::get() << _logger_prefix << "Set positive speed for front right"
                      << evo::info;
      for(int i = 0; i < test_sleep_time_us; i++)
      {
         if(_motor_front_right->setTargetSpeed(test_rpm) != evo_mbed::RES_OK)
            error_flag |= true;
      }

      evo::log::get() << _logger_prefix << "Set positive speed for back right"
                      << evo::info;
      for(int i = 0; i < test_sleep_time_us; i++)
      {
         if(_motor_back_right->setTargetSpeed(test_rpm) != evo_mbed::RES_OK)
            error_flag |= true;
      }

      evo::log::get() << _logger_prefix << "Set positive speed for back left"
                      << evo::info;
      for(int i = 0; i < test_sleep_time_us; i++)
      {
         if(_motor_back_left->setTargetSpeed(test_rpm) != evo_mbed::RES_OK)
            error_flag |= true;
      }

      if(error_flag)
      {
         evo::log::get() << _logger_prefix << "Couldn't set target speed!"
                         << evo::error;
      }
   }
   else
   {
      evo::log::get() << _logger_prefix << "not initialized yet! -> check"
                      << evo::error;
      checkInitState();
   }
}

} // namespace evo
