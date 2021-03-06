//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file MotorManager.cpp
 * @author Evocortex GmbH (MMA)
 *
 * @brief Class for holding all mcs and motors (also creation from params)
 *
 * @version 0.2
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2020 Evocortex GmbH
 *
 */

#include "evo_robot_base_interface/MotorManager.h"
#include <csignal>

namespace evo {

MotorManager::MotorManager() : _logger_prefix("MotorManager: ")
{
   _can_interface = std::make_shared<evo_mbed::ComServer>();
   evo::log::init("");
}

MotorManager::~MotorManager()
{
   disableAllMotors();
   _can_interface->release();
}

const bool MotorManager::initMotorMapping(Drive2d& drive, LiftController& lc)
{
   evo::log::get() << _logger_prefix << "init motor mapping!" << evo::info;
   uint8_t mapped_motors = 0;
   for(auto& mc_cfg : _vec_ms_configs)
   {
      uint8_t motor_counter = 0;
      for(auto& motor_cfg : mc_cfg.motor_configs)
      {
         // only allow drive motors to be mapped to the mecanum drive
         if((motor_cfg.type == evo_mbed::MOTOR_TYPE_DRIVE) &&
            (motor_cfg.motor_mapping > MOTOR_MAPPING::NO_POSITION))
         {
            // search in actual connected motors
            for(auto& ms : _vec_motor_shields)
            {
               if(ms->getComID() == mc_cfg.id)
               {
                  std::shared_ptr<evo_mbed::Motor> motor = ms->getMotor(motor_counter);
                  if(motor != nullptr)
                  {
                     drive.setMotorRef(motor, motor_cfg.motor_mapping);
                     ++mapped_motors;
                  }
               }
            }
         }
         else if(motor_cfg.type == evo_mbed::MOTOR_TYPE_LIFT)
         {
            // search in actual connected motors
            for(auto& ms : _vec_motor_shields)
            {
               if(ms->getComID() == mc_cfg.id)
               {
                  std::shared_ptr<evo_mbed::Motor> motor = ms->getMotor(motor_counter);
                  if(motor != nullptr)
                  {
                     lc.setMotorRef(motor, motor_cfg.motor_mapping);
                     ++mapped_motors;
                  }
               }
            }
         }
         ++motor_counter;
      }
   }
   evo::log::get() << _logger_prefix << "mapped " 
                   << +mapped_motors << " motors"
   << evo::info;

   return true;
}

const bool MotorManager::initCanInterface(const std::string& can_interface_name)
{
   if(_can_interface == nullptr)
   {
      evo::log::get() << _logger_prefix << "cannot init CAN interface! nullptr!"
      << evo::error;
      
      return false;
   }
   return (_can_interface->init(can_interface_name) == evo_mbed::RES_OK);
}

const bool MotorManager::initMotor(std::shared_ptr<evo_mbed::Motor> motor,
                                   const std::string logger_prefix,
                                   const MotorConfig& motor_config)
{
   evo::log::get() << logger_prefix << "start init process" << evo::info;

   if(motor == nullptr)
   {
      evo::log::get() << logger_prefix << "motor pointer is empty!" << evo::error;
      return false;
   }

   bool error_occured = false;

   // try to set params and track if any error occured
   if(!motor->setType(motor_config.type))
   {
      evo::log::get() << logger_prefix << "failed to set type!" << evo::error;
      error_occured |= true;
   }

   if(!motor->setControlMode(motor_config.mode))
   {
      evo::log::get() << logger_prefix << "failed to set control mode!"
                      << evo::error;
      error_occured |= true;
   }

   if(!motor->setSpeedKp(motor_config.kp))
   {
      evo::log::get() << logger_prefix << "failed to set kp!" << evo::error;
      error_occured |= true;
   }

   if(!motor->setSpeedKi(motor_config.ki))
   {
      evo::log::get() << logger_prefix << "failed to set ki!" << evo::error;
      error_occured |= true;
   }

   if(!motor->setSpeedKd(motor_config.kd))
   {
      evo::log::get() << logger_prefix << "failed to set kd!" << evo::error;
      error_occured |= true;
   }

   if(evo_mbed::MOTOR_TYPE_DRIVE == motor_config.type)
   {
      if(!motor->setEncoderResolution(motor_config.encoder_res))
      {
         evo::log::get() << logger_prefix << "failed to set encoder resolution!"
                        << evo::error;
         error_occured |= true;
      }

      if(!motor->setGearRatio(motor_config.gear_ratio))
      {
         evo::log::get() << logger_prefix << "failed to set gear ratio!" << evo::error;
         error_occured |= true;
      }
      if(!motor->setMaxSpeedRPM(motor_config.rpm_limit))
      {
         evo::log::get() << logger_prefix << "failed to set rpm limit!" << evo::error;
         error_occured |= true;
      }
   }  

   else if(evo_mbed::MOTOR_TYPE_LIFT == motor_config.type)
   {
      if(!motor->setConvFacAdcMMPerTick(motor_config.adc_conv_mm_per_tick))
      {
         evo::log::get() << logger_prefix << "failed to set adc conversion factor!" << evo::error;
         error_occured |= true;
      }

      if(!motor->setOffsAdcMM(motor_config.adc_offs_mm))
      {
         evo::log::get() << logger_prefix << "failed to set adc offset!" << evo::error;
         error_occured |= true;
      }
   }

   // try to enable the motor to see if the config would work
   if(!motor->setOperationStatus(evo_mbed::MOTOR_STS_ENABLED))
   {
      evo::log::get() << logger_prefix << "failed to enable motor! Config is faulty!"
                      << evo::error;
      error_occured |= true;
   }

   // disable again
   if(!motor->setOperationStatus(evo_mbed::MOTOR_STS_DISABLED))
   {
      evo::log::get() << logger_prefix << "failed to disable motor!" << evo::error;
      evo::log::get() << logger_prefix
                      << "DANGER! could move with faulty config! DANGER!"
                      << evo::warn;
      error_occured |= true;
   }

   return !error_occured;
}

void MotorManager::setConfig(const std::vector<MotorShieldConfig> mc_config,
                             const bool reset_manager)
{
   if(reset_manager)
   {
      evo::log::get() << _logger_prefix << "set new config, resetting motor manager!"
                      << evo::warn;
      disableAllMotors();
      _vec_motor_shields.clear();
   }
   _vec_ms_configs.clear();
   _vec_ms_configs = mc_config;
}

const bool MotorManager::initFromConfig()
{
   if(!_can_interface->isInitialized())
   {
      evo::log::get()
          << _logger_prefix
          << "Unable to create from params! CAN interface not initialized!"
          << evo::error;
      return false;
   }
   _vec_motor_shields.clear();
   bool clean_init = true;
   for(MotorShieldConfig& ms_config : _vec_ms_configs)
   {
      evo::log::get() << _logger_prefix << "init motorshield with id "
                      << +ms_config.id << evo::info;
      std::shared_ptr<evo_mbed::MotorShield> ms =
          std::make_shared<evo_mbed::MotorShield>(ms_config.id, _can_interface);
      if(!ms->init())
      {
         evo::log::get() << _logger_prefix << "failed to init ms" << +ms_config.id
                         << evo::error;
         return false;
      }

      if(!ms->setComTimeout(ms_config.timeout_ms))
      {
         evo::log::get() << _logger_prefix << "failed to set timeout on ms"
                         << +ms_config.id << evo::error;
         clean_init = false;
      }
      else
      {
         evo::log::get() << _logger_prefix << "init " << +ms_config.n_motors
                         << " motors on ms" << +ms_config.id << evo::info;
         for(uint8_t motor_id = 0; motor_id < ms_config.n_motors; motor_id++)
         {
            std::shared_ptr<evo_mbed::Motor> motor = ms->getMotor(motor_id);
            std::string logger_prefix = _logger_prefix + "ms" +
                                        std::to_string(ms_config.id) + "-motor" +
                                        std::to_string(motor_id) + ": ";
            if(!initMotor(motor, logger_prefix, ms_config.motor_configs.at(motor_id)))
            {
               evo::log::get()
                   << logger_prefix << "init not successfull!" << evo::error;
               clean_init = false;
            }
         }
         // add sucessfull mc config to vector
         _vec_motor_shields.push_back(ms);
      }
   }
   return clean_init;
}

const bool MotorManager::checkStatus()
{
   bool is_sts_ok = true;
   for(auto& mc : _vec_motor_shields)
   {
      switch(mc->getState())
      {
      case evo_mbed::MOTOR_SHIELD_STS_OK: break;

      case evo_mbed::MOTOR_SHIELD_STS_ERR:
      {
         evo::log::get() << _logger_prefix << "Critical error with can-interface!" 
                        << "(Disconnected?) -> Exiting"
             << evo::error;
         std::raise(SIGTERM);
      }
      break;

      case evo_mbed::MOTOR_SHIELD_SYNC_ERR:
      {
         evo::log::get() << _logger_prefix << "Resyncing" << evo::warn;
         mc->resyncShield();
         is_sts_ok = false;
      }
      break;

      default: { is_sts_ok = false;
      }
      break;
      };
   }

   return is_sts_ok;
}

const bool MotorManager::setOperationStatus(const bool enable)
{
   bool clean_operation = true;
   int ms_counter       = 0;
   for(auto& ms : _vec_motor_shields)
   {
      // load config again..
      MotorShieldConfig ms_config;
      bool found_matching_cfg = false;
      for(MotorShieldConfig& ms_cfg : _vec_ms_configs)
      {
         if(ms_cfg.id == ms->getComID())
         {
            found_matching_cfg = true;
            ms_config          = ms_cfg;
         }
      }
      if(found_matching_cfg)
      {
         for(uint8_t motor_id = 0; motor_id < ms_config.n_motors; motor_id++)
         {
            std::shared_ptr<evo_mbed::Motor> motor = ms->getMotor(motor_id);
            if(enable)
            {
               if(!motor->setOperationStatus(evo_mbed::MOTOR_STS_ENABLED))
               {
                  evo::log::get() << _logger_prefix << "failed to enable ms" << +ms_config.id
                      << "-motor" << +motor_id 
                  << evo::error;
                  clean_operation = false;
               }
               else
               {
                  evo::log::get() << _logger_prefix << "enabled ms" << +ms_config.id
                                  << "-motor" << +motor_id << evo::info;
               }
            }
            else
            {
               if(!motor->setOperationStatus(evo_mbed::MOTOR_STS_DISABLED))
               {
                  evo::log::get()
                      << _logger_prefix << "failed to disable ms" << +ms_config.id
                      << "-motor" << +motor_id << evo::error;
                  clean_operation = false;
               }
               else
               {
                  evo::log::get() << _logger_prefix << "disabled ms" << +ms_config.id
                                  << "-motor" << +motor_id << evo::info;
               }
            }
         }
      }
      else
      {
         evo::log::get() << _logger_prefix << "failed to enable motors on ms"
                         << +ms_config.id << " - couldn't find matching config data"
                         << evo::error;
         clean_operation = false;
      }

      ++ms_counter;
   }
   return clean_operation;
}

const bool MotorManager::enableAllMotors()
{
   return setOperationStatus(true);
}

const bool MotorManager::disableAllMotors()
{
   return setOperationStatus(false);
}

const bool MotorManager::enableAllDriveMotors()
{
   bool success = true;

   for(auto& shield : _vec_motor_shields) {
      for(auto drive_idx = 0u; drive_idx < evo_mbed::MOTOR_SHIELD_DRIVES; drive_idx++) {
         auto drive = shield->getMotor(drive_idx);

         if(evo_mbed::MOTOR_TYPE_DRIVE == drive->getType()) {
            if(evo_mbed::MOTOR_STS_DISABLED == drive->getOperationStatus()) {
               if(!drive->setOperationStatus(evo_mbed::MOTOR_STS_ENABLED)) {
                  success = false;
               }
            }
         }

         std::this_thread::sleep_for(std::chrono::milliseconds(75u));
      }
   }

   return success;
}

const bool MotorManager::disableAllDriveMotors()
{
   bool success = true;

   for(auto& shield : _vec_motor_shields) {
      for(auto drive_idx = 0u; drive_idx < evo_mbed::MOTOR_SHIELD_DRIVES; drive_idx++) {
         auto drive = shield->getMotor(drive_idx);

         if(evo_mbed::MOTOR_TYPE_DRIVE == drive->getType()) {
            if(evo_mbed::MOTOR_STS_ENABLED == drive->getOperationStatus()) {
               if(!drive->setOperationStatus(evo_mbed::MOTOR_STS_DISABLED)) {
                  success = false;
               }
            }
         }
      }
   }

   return success;
}

} // namespace evo
