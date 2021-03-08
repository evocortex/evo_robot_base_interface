//###############################################################
//# Copyright (C) 2021, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file 2dKinematics.h
 * @author Evocortex GmbH (MMA)
 *
 * @brief Class for two-dimensional drive models
 *
 * @version 0.1
 * @date 2021-02-19
 *
 * @copyright Copyright (c) 2020 Evocortex GmbH
 *
 */

#pragma once

#include "evo_motor_shield_interface/Motor.h"

#include "evo_logger/log/Logger.h"

namespace evo {

struct WheelData
{
   double front_left  = 0.0;
   double front_right = 0.0;
   double back_left   = 0.0;
   double back_right  = 0.0;

   inline WheelData& operator*=(const double factor)
   {
      this->front_left *= factor;
      this->front_right *= factor;
      this->back_left *= factor;
      this->back_right *= factor;
      return *this;
   }
};

struct Vel2d
{
   double _x_ms     = 0.0;
   double _y_ms     = 0.0;
   double _yaw_rads = 0.0;
};

struct Pose2d
{
   double _x_m     = 0.0;
   double _y_m     = 0.0;
   double _yaw_rad = 0.0;

   void reset() { *this = Pose2d(); }

   void updatePoseFromVel(const Vel2d& vel, const double cycle_time)
   {
      this->_x_m += (std::cos(this->_yaw_rad) * vel._x_ms -
                     std::sin(this->_yaw_rad) * vel._y_ms) *
                    cycle_time;
      this->_y_m += (std::sin(this->_yaw_rad) * vel._x_ms +
                     std::cos(this->_yaw_rad) * vel._y_ms) *
                    cycle_time;
      this->_yaw_rad += vel._yaw_rads * cycle_time;
   }

   void updatePoseFromIncrement(const Pose2d& inc)
   {
      this->_x_m += std::cos(this->_yaw_rad) * inc._x_m 
                  - std::sin(this->_yaw_rad) * inc._y_m;
      this->_y_m += std::sin(this->_yaw_rad) * inc._x_m 
                  + std::cos(this->_yaw_rad) * inc._y_m;
      this->_yaw_rad += inc._yaw_rad;
   }
};

struct Cov2d
{
   double cov_pos_x   = 0.0;
   double cov_pos_y   = 0.0;
   double cov_pos_yaw = 0.0;
   double cov_vel_x   = 0.0;
   double cov_vel_y   = 0.0;
   double cov_vel_yaw = 0.0;
};

enum MOTOR_MAPPING
{
   NO_POSITION = 0,
   FRONT_LEFT,
   FRONT_RIGHT,
   BACK_RIGHT,
   BACK_LEFT
};

class Drive2d
{
 protected:
   // references to motors of motor handler
   std::shared_ptr<evo_mbed::Motor> _motor_front_left;
   std::shared_ptr<evo_mbed::Motor> _motor_front_right;
   std::shared_ptr<evo_mbed::Motor> _motor_back_left;
   std::shared_ptr<evo_mbed::Motor> _motor_back_right;

   // used in some general functions
   std::vector<std::shared_ptr<evo_mbed::Motor>> _motor_refs;

   // robot dimensions
   double _wheel_radius_in_m;
   double _wheel_separation_length_in_m;
   double _wheel_separation_width_in_m;
   double _wheel_separation_sum_in_m;

   // helper values for calculations
   double _ms2rpm;
   double _rpm2ms;
   double _rot2m;

   std::string _logger_prefix;

   bool _is_initialized;
   bool _verbose;

   WheelData _last_position;
   WheelData _current_position;
   WheelData _current_rpm;

   //-------------
 public:
   Drive2d();
   virtual ~Drive2d();

   // Set the robot dimensions for this mecanum drive configuration
   void setWheelRadiusInM(const double wheel_radius_in_m);
   void setWheelSeparationLengthInM(const double wheel_separation_length_in_m);
   void setWheelSeparationWidthInM(const double wheel_separation_Width_in_m);
   void setWheelDistanceFrontBackInM(const double wheel_distance_front_back_in_m);
   void setWheelDistanceLeftRightInM(const double wheel_distance_left_right_in_m);

   // uses checkParams and checkMotors
   bool checkInitState();
   bool checkParams();


   bool resetEncoders();

   bool getOdomComplete(Vel2d& odom_vel, Pose2d& odom_pose_increment,
                        WheelData& current_position, WheelData& current_velocity);


   /*
   * Default Functions for 4 motors
   * may be overwritten if necessary
   */
   virtual bool checkMotors();

   virtual void setMotorRef(std::shared_ptr<evo_mbed::Motor> motor,
                     const uint8_t motor_mapping);

   virtual void printWheelData(const WheelData& wd, const std::string& nametag);

   
   // sending and receiving data
   virtual bool readWheelData();
   virtual bool setCmdVel(const Vel2d& cmd_vel);

   /*
   *  The following functions must be implemented by a real drive model class:
   *  e.g mecanum, differential, tank
   */

   virtual void wheelData2OdomVel(const WheelData& wd, Vel2d& vel)   = 0;
   virtual void wheelData2OdomPose(const WheelData& wd, Pose2d& pose) = 0;
   virtual void wheelData2OdomPoseInc(const WheelData& wd, WheelData& lwd, Pose2d& poin) = 0;

   virtual void cmdVel2wheelData(const Vel2d& cmd_vel, WheelData& cmd_wd) = 0;



   // dont use this
   virtual void debugMotorMapping();
};
} // namespace evo
