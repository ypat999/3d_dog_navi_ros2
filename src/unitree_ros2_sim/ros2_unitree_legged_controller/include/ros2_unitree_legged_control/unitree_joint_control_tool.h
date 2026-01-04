/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

// #ifndef _UNITREE_JOINT_CONTROL_TOOL_H_
// #define _UNITREE_JOINT_CONTROL_TOOL_H_
#ifndef _LAIKAGO_CONTROL_TOOL_H_
#define _LAIKAGO_CONTROL_TOOL_H_

#include <stdio.h>
#include <stdint.h>
#include <algorithm>
#include <math.h>

#define posStopF (2.146E+9f)  // stop position control mode
#define velStopF (16000.0f)   // stop velocity control mode

typedef struct 
{
    uint8_t mode;
    double pos;
    double posStiffness;
    double vel;
    double velStiffness;
    double torque;
} ServoCmd;

inline double clamp(double& value, double min_val, double max_val) {
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}  // eg. clamp(1.5, -1, 1) = 1

inline double computeVel(double current_position, double last_position, double last_velocity, double duration) {
  // Simple velocity calculation using finite difference
  return (current_position - last_position) / duration;
}  // get current velocity

inline double computeTorque(double current_position, double current_velocity, ServoCmd& cmd) {
  // Simple PD controller for torque calculation
  double position_error = cmd.pos - current_position;
  double torque = cmd.posStiffness * position_error - cmd.velStiffness * current_velocity;
  return torque;
}  // get torque

#endif