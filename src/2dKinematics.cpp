#include "evo_robot_base_interface/2dKinematics.h"

namespace evo
{

Drive2d::Drive2d() :
    _logger_prefix("Drive2d: "), 
    _is_initialized(false),
    _wheel_radius_in_m(0.0), 
    _wheel_separation_length_in_m(0.0),
    _wheel_separation_width_in_m(0.0), 
    _wheel_separation_sum_in_m(0.0)
{
   evo::log::init("");
}

Drive2d::~Drive2d()
{
}


void Drive2d::setWheelRadiusInM(const double wheel_radius_in_m)
{
   _wheel_radius_in_m = wheel_radius_in_m;
   _ms2rpm            = 60.0 / (2.0 * _wheel_radius_in_m * M_PI);
   _rpm2ms            = 1.0 / _ms2rpm;
   _rot2m             = (2.0 * _wheel_radius_in_m * M_PI);
}
void Drive2d::setWheelSeparationLengthInM(const double wheel_separation_length_in_m)
{
   _wheel_separation_length_in_m = wheel_separation_length_in_m;
   _wheel_separation_sum_in_m =
       _wheel_separation_length_in_m + _wheel_separation_width_in_m;
}
void Drive2d::setWheelSeparationWidthInM(const double wheel_separation_Width_in_m)
{
   _wheel_separation_width_in_m = wheel_separation_Width_in_m;
   _wheel_separation_sum_in_m =
       _wheel_separation_length_in_m + _wheel_separation_width_in_m;
}
void Drive2d::setWheelDistanceFrontBackInM(const double wheel_distance_front_back_in_m)
{
   setWheelSeparationLengthInM(wheel_distance_front_back_in_m / 2.0);
}
void Drive2d::setWheelDistanceLeftRightInM(const double wheel_distance_left_right_in_m)
{
   setWheelSeparationWidthInM(wheel_distance_left_right_in_m / 2.0);
}

bool Drive2d::checkInitState()
{
   bool _is_initialized = checkMotors() && checkParams();
   evo::log::get() << _logger_prefix << "init state: " << _is_initialized
                   << evo::info;
   return _is_initialized;
}

bool Drive2d::checkParams()
{
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

   return parameters_set;
}

bool Drive2d::resetEncoders()
{
   if(_is_initialized)
   {
      bool success = true;
      // reset all motors
      for(auto &motor : _motor_refs)
      {
         success &= motor->resetRevs();
      }
      
      // reset last position
      _last_position = WheelData();

      return success;
   }
   else
   {
      evo::log::get() << _logger_prefix << "not initialized yet! -> check" << evo::error;
      checkInitState();
      return false;
   }
}

bool Drive2d::getOdomComplete(Vel2d& odom_vel, Pose2d& odom_pose_increment,
                        WheelData& current_position, WheelData& current_velocity)
{
   if(!_is_initialized)
   {      
      evo::log::get() << _logger_prefix << "not initialized yet! -> check"
                      << evo::error;
      checkInitState();
      return false;
   }
   else
   {
      // update 
      if(!readWheelData()) return false;

      wheelData2OdomVel(_current_rpm, odom_vel);

      //wheelData2OdomPose(_current_position, odom_pose);
      wheelData2OdomPoseInc(_current_position, _last_position, odom_pose_increment);

      current_position = _current_position;
      current_position *= 2 * M_PI;

      current_velocity = _current_rpm;
      current_velocity *= M_PI / 30.0;

      return true;
   }
}



bool Drive2d::checkMotors()
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

   return motors_connected;
}

void Drive2d::setMotorRef(std::shared_ptr<evo_mbed::Motor> motor,
                               const uint8_t motor_mapping)
{
   switch(motor_mapping)
   {
   case MOTOR_MAPPING::FRONT_LEFT:
   {
      evo::log::get() << _logger_prefix << "motor front left mapped!" << evo::info;
      _motor_front_left = motor;
      break;
   }
   case MOTOR_MAPPING::FRONT_RIGHT:
   {
      evo::log::get() << _logger_prefix << "motor front right mapped!" << evo::info;
      _motor_front_right = motor;
      break;
   }
   case MOTOR_MAPPING::BACK_RIGHT:
   {
      evo::log::get() << _logger_prefix << "motor back right mapped!" << evo::info;
      _motor_back_right = motor;
      break;
   }
   case MOTOR_MAPPING::BACK_LEFT:
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

   // add motor ref to vector (only convenience reference)
   _motor_refs.push_back(motor);
}



void Drive2d::printWheelData(const WheelData& wd, const std::string& nametag)
{
   evo::log::get() << _logger_prefix << "Printing [" << nametag << "] wheeldata:" << evo::info;
   evo::log::get() << _logger_prefix << "Front Left: "  << wd.front_left << evo::info;
   evo::log::get() << _logger_prefix << "Front Right: " << wd.front_right << evo::info;
   evo::log::get() << _logger_prefix << "Back Left: "   << wd.back_left << evo::info;
   evo::log::get() << _logger_prefix << "Back Right: "  << wd.back_right << evo::info;
}


// new version: cleaner code 
// read all data in one cycle
bool Drive2d::readWheelData()
{
   if(_is_initialized)
   {
      // get wheel speeds
      _current_rpm.front_left  = _motor_front_left->getSpeedRPM();
      _current_rpm.front_right = _motor_front_right->getSpeedRPM();

      _current_rpm.back_left  = _motor_back_left->getSpeedRPM();
      _current_rpm.back_right = _motor_back_right->getSpeedRPM();

      // get positions
      _current_position.front_left  = _motor_front_left->getRevolutions();
      _current_position.front_right = _motor_front_right->getRevolutions();

      _current_position.back_left  = _motor_back_left->getRevolutions();
      _current_position.back_right = _motor_back_right->getRevolutions();

      return true;
   }
   else
   {
      evo::log::get() << _logger_prefix << "not initialized yet! -> check" << evo::error;
      checkInitState();
      return false;
   }
}

bool Drive2d::setCmdVel(const Vel2d& cmd_vel)
{
   if(_is_initialized)
   {
      WheelData cmd_wd;
      cmdVel2wheelData(cmd_vel, cmd_wd);

      
      // limit output speeds if cmd vel exceeds maximum

      // MMA: this could also be done previously in calculation functions
      // it affects the behaviour and is located in a hw access function atm
      // doesnt really belong here, but also accesses motor vals which dont belong into pure calc classes

      // MMA STYLE: is there a better way to prevent division by zero?
      double max_ratio = 1;
      double ratio = std::abs(cmd_wd.front_left / (_motor_front_left->getMaxSpeedRPM() + 0.0000001));
      if(ratio > max_ratio){max_ratio = ratio;}
      
      ratio = std::abs(cmd_wd.front_right / (_motor_front_right->getMaxSpeedRPM() + 0.0000001));
      if(ratio > max_ratio){max_ratio = ratio;}

      ratio = std::abs(cmd_wd.back_left / (_motor_back_left->getMaxSpeedRPM() + 0.0000001));
      if(ratio > max_ratio){max_ratio = ratio;}

      ratio = std::abs(cmd_wd.back_right / (_motor_back_right->getMaxSpeedRPM() + 0.0000001));
      if(ratio > max_ratio){max_ratio = ratio;}

      if(max_ratio > 1)
      {
         evo::log::get() << _logger_prefix 
         << "Exceeding max RPM limits! limiting with factor " << max_ratio 
         <<  evo::warn;
      }
      cmd_wd.front_left /= max_ratio;
      cmd_wd.front_right /= max_ratio;
      cmd_wd.back_left /= max_ratio;
      cmd_wd.back_right /= max_ratio;

      // apply to motors
      bool error_flag = false;
      if(!_motor_front_left->setTargetSpeed(cmd_wd.front_left))   error_flag |= true;
      if(!_motor_front_right->setTargetSpeed(cmd_wd.front_right)) error_flag |= true;
      if(!_motor_back_left->setTargetSpeed(cmd_wd.back_left))     error_flag |= true;
      if(!_motor_back_right->setTargetSpeed(cmd_wd.back_right))   error_flag |= true;

      if(error_flag)
      {
         evo::log::get() << _logger_prefix << "Couldn't set target speed!" << evo::error;
         return false;
      }
      return true;
   }
   else
   {
      evo::log::get() << _logger_prefix << "not initialized yet! -> check" << evo::error;
      checkInitState();
      return false;
   }
}


/// this function should not exist in this class. rather a function that discovers
/// all motors in motor handler
void Drive2d::debugMotorMapping()
{
   evo::log::get() << _logger_prefix << "Debug Motor Mapping is NOT implemented!"
                   << evo::error;
   return;

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