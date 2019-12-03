//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file LiftController.cpp
 * @author Evocortex GmbH (MMA)
 *
 * @brief Class for controlling the lift mechanism of the evo robot
 *
 * @version 0.1
 * @date 2019-08-12
 *
 * @copyright Copyright (c) 2019 Evocortex GmbH
 *
 */

#include "evo_robot_base_interface/LiftController.h"

namespace evo {

LiftController::LiftController() :
    _logger_prefix("LiftController: "), _is_initialized(false),
    _max_moving_speed(65.0)
{
   evo::log::init("");
}

bool LiftController::setAllMotors(const double pwm_value)
{
   bool success = true;
   for(auto& lift_motor : _vec_lift_motors)
   {
      success &= (evo_mbed::RES_OK == lift_motor->setOperationStatus(
                                          evo_mbed::MotorStatus::MOTOR_STS_ENABLED));
      success &= (evo_mbed::RES_OK == lift_motor->setTargetPWM(pwm_value));
   }
   return success;
}

void LiftController::setMotorRef(std::shared_ptr<evo_mbed::Motor> motor,
                                 const uint8_t motor_mapping)
{
   evo::log::get() << _logger_prefix << "added lift motor" << evo::info;
   _vec_lift_motors.push_back(motor);
}

bool LiftController::setMovingDirection(const int8_t direction)
{
   if(direction < 0)
   {
      return setAllMotors(-_max_moving_speed);
   }
   else if(direction > 0)
   {
      return setAllMotors(_max_moving_speed);
   }
   else
   {
      return setAllMotors(0);
   }
}

void LiftController::setMaxMovingSpeed(const double max_pwm_value)
{
   // current robot setup (36V system - 24V lift motor) doesn't allow more
   if(std::abs(max_pwm_value) <= 0.65)
   {
      _max_moving_speed = max_pwm_value;
   }
   else
   {
      evo::log::get() << _logger_prefix << "can't set max moving speed to"
                      << max_pwm_value << "! limit is 0.65 for the current setup!"
                      << evo::error;
   }
}

const std::vector<float> LiftController::getPositionVec(void)
{
   std::vector<float> position_vec;
   for(auto& lift_motor : _vec_lift_motors)
   {
      position_vec.push_back(lift_motor->getPositionMM());
   }

   return position_vec;
}

} // namespace evo
