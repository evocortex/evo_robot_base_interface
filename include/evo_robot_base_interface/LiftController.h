//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

#ifndef LIFTCONTROLLER_H
#define LIFTCONTROLLER_H

#include "evo_motor_shield_interface/Motor.h"

#include "evo_logger/log/Logger.h"

namespace evo {

enum MOTOR_MAPPING_LIFT
{
   NO_POSITION_LIFT = 0,
   FRONT_LEFT_LIFT  = 11,
   FRONT_RIGHT_LIFT,
   BACK_RIGHT_LIFT,
   BACK_LEFT_LIFT
};

class LiftController
{
 private:
   std::vector<std::shared_ptr<evo_mbed::Motor>> _vec_lift_motors;
   std::string _logger_prefix;
   bool _is_initialized;

   double _max_moving_speed;

   bool setAllMotors(const double pwm_value);

 public:
   LiftController();

   void setMaxMovingSpeed(const double max_pwm_value);

   bool setMovingDirection(const int8_t direction);

 /**
     * @brief Set the Motor Ref object
     * 
     * @param motor - evo mbed motor object
     * @param motor_mapping - use ENUM MOTOR_MAPPING_LIFT to virtually place motors at correct position
     */
   void setMotorRef(std::shared_ptr<evo_mbed::Motor> motor,
                    const uint8_t motor_mapping);

  // marco: this is still not the correct way to return const functions. (was added by MBS)
   const std::vector<float> getPositionVec(void);
};

} // namespace evo

#endif // LIFTCONTROLLER_H
