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

namespace evo {

struct MecanumWheelData
{
   double front_left = 0.0;
   double front_right = 0.0;
   double back_left = 0.0;
   double back_right = 0.0;

   inline MecanumWheelData& operator*=(const double factor)
   {
      this->front_left *= factor;
      this->front_right *= factor;
      this->back_left *= factor;
      this->back_right *= factor;
      return *this;
   }
};

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


// MMA: split the class here? -> separation of mecanum maths and hw access
// new class MecanumMaths or so
// we should think about a software redesign (keep a lot of code but organize different)

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

   MecanumWheelData _last_position;
   MecanumWheelData _current_position;
   MecanumWheelData _current_rpm;
   //-------------
 public:
      MecanumDrive();

    /**
     * @brief checks if all motor references and robot dimensions are set
     * 
     * @return true if initialized correct 
     */
    bool checkInitState();

    /**
     * @brief Set the virtual Motor position for the mecanum drive
     *        Extremly important to get this right, otherwise the kinematic model is incorrect!
     * 
     * @param motor - motor object from evo_mbed
     * @param motor_mapping - see ENUM MOTOR_MAPPING_MECANUM for correct motor mapping
     */
    void setMotorRef(std::shared_ptr<evo_mbed::Motor> motor, const uint8_t motor_mapping);

    // Set the robot dimensions for this mecanum drive configuration
    void setWheelRadiusInM(const double wheel_radius_in_m);
    void setWheelSeparationLengthInM(const double wheel_separation_length_in_m);
    void setWheelSeparationWidthInM(const double wheel_separation_Width_in_m);
    void setWheelDistanceFrontBackInM(const double wheel_distance_front_back_in_m);
    void setWheelDistanceLeftRightInM(const double wheel_distance_left_right_in_m);

    // update all wheel data 
    bool readWheelData();

    void wheelData2OdomVel(const MecanumWheelData& wd, MecanumVel& mv);
    void wheelData2OdomPose(const MecanumWheelData& wd, MecanumPose& mp);
    void wheelData2OdomPoseInc(const MecanumWheelData& wd, MecanumWheelData& lwd, MecanumPose& mpi);


    bool getOdomComplete(MecanumVel& odom_vel, 
                         MecanumPose& odom_pose_increment,
                         MecanumWheelData& current_position,
                         MecanumWheelData& current_velocity);

    void cmdVel2wheelData(const MecanumVel& cmd_vel, MecanumWheelData& cmd_wd);

    bool setCmdVel(const MecanumVel& cmd_vel);


    // dont use this 
    void debugMotorMapping();
};
} // namespace evo
#endif // MECANUMDRIVE_H
