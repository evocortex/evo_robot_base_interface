//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file MotorManager.h
 * @author Evocortex GmbH (MMA)
 *
 * @brief Class for holding all mcs and motors (also creation from params)
 *
 * @version 0.1
 * @date 2019-08-12
 *
 * @copyright Copyright (c) 2019 Evocortex GmbH
 *
 */

#ifndef MOTORMANAGER_H
#define MOTORMANAGER_H

// hardware interface classes for can communication via usb
#include "evo_motor_shield_interface/MotorShield.h"
#include "evo_motor_shield_interface/Motor.h"
#include "evo_mbed/tools/com/ComServer.h"

// needed for motor mapping - room for other plugins
#include "evo_robot_base_interface/MecanumDrive.h"
#include "evo_robot_base_interface/LiftController.h"

// custom logger
#include "evo_logger/log/Logger.h"

namespace evo {

// config struct for one motor
struct MotorConfig
{
   evo_mbed::MotorType type        = evo_mbed::MOTOR_TYPE_NONE;
   evo_mbed::MotorControlMode mode = evo_mbed::MOTOR_CTRL_NONE;
   uint8_t motor_mapping           = 0;
   double pwm_limit                = 0.0;
   double kp                       = 0.0;
   double ki                       = 0.0;
   double kd                       = 0.0;
   double gear_ratio               = 0.0;
   double encoder_res              = 0.0;
   double adc_conv_mm_per_tick     = 0.0;
   double adc_offs_mm              = 0.0;
};

// config struct for one motor controller
struct MotorShieldConfig
{
   uint8_t id;
   uint32_t timeout_ms;
   uint8_t n_motors;
   std::vector<MotorConfig> motor_configs;
};

// Motor Manager Class
class MotorManager
{
 private:
   // vector of all connected motor shields
   std::vector<std::shared_ptr<evo_mbed::MotorShield>> _vec_motor_shields;
   // vector of configuration sets for multiple mcs
   std::vector<MotorShieldConfig> _vec_ms_configs;
   // can interface
   std::shared_ptr<evo_mbed::ComServer> _can_interface;

   std::string _logger_prefix;

 public:
   MotorManager();
   ~MotorManager();

   /**
    * @brief setConfig
    * @param mc_config - vector of configurations for single motorcontrollers
    * @param reset_manager - default: true - resets the motor manager (deletes the
    * motor connections)
    */
   void setConfig(const std::vector<MotorShieldConfig> mc_config,
                  const bool reset_manager = true);

   /**
    * @brief initCanInterface
    * @param can_interface_name - name of the interface given in setup script
    * @return true if successful
    */
   const bool initCanInterface(const std::string& can_interface_name);

   /**
    * @brief initMotor - will be called in initFromConfig
    * @param motor - shared ptr to motor that should be initialized
    * @param logger_prefix
    * @param motor_config - parameters for the motor
    * @return true if successful
    */
   const bool initMotor(std::shared_ptr<evo_mbed::Motor> motor,
                        const std::string logger_prefix,
                        const MotorConfig& motor_config);

   /**
    * @brief initFromConfig - use config that has to be set via setConfig before to
    * initialize Motorhandler
    * @return true if successful
    */
   const bool initFromConfig();

   /**
    * @brief initMotorMapping - sets motor references from config in given
    * mecanumdrive and liftcontroller object
    * @param md
    * @return true if successful
    */
   const bool initMotorMapping(MecanumDrive& md, LiftController& lc);

   /**
    * @brief setOperationStatus - function to enable or disable all connected motors
    * @param enable
    * @return true if successful
    */
   const bool setOperationStatus(const bool enable);

   /**
    * @brief enableAllMotors - calls setOperationStatus with param TRUE
    * @return true if successful
    */
   const bool enableAllMotors();

   /**
    * @brief disableAllMotors - calls setOperationStatus with param FALSE
    * @return true if successful
    */
   const bool disableAllMotors();

   /**
    * @brief Checks if all drives are in "ok" state
    *
    * @return true All drives in no error state
    * @return false Drive timeout, resync etc. detected
    */
   const bool checkStatus();
};
} // namespace evo
#endif // MOTORMANAGER_H
