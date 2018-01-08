/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

#ifndef TURTLEBOT3_CORE_CONFIG_H_
#define TURTLEBOT3_CORE_CONFIG_H_

#include <math.h>

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <turtlebot3_msgs/SensorState.h>

#include <IMU.h>
#include <RC100.h>

#include "turtlebot3_motor_driver.h"
#include "DynamixelConfig.h"

#define CONTROL_MOTOR_SPEED_PERIOD       100   //hz
#define IMU_PUBLISH_PERIOD               100  //hz
#define SENSOR_STATE_PUBLISH_PERIOD      100   //hz
#define CMD_VEL_PUBLISH_PERIOD           100   //hz
#define DRIVE_INFORMATION_PUBLISH_PERIOD 100  //hz
#define VOLTAGE_PUBLISH_PERIOD           1    //hz

#define WHEEL_RADIUS                     0.033           // meter
#define WHEEL_SEPARATION                 0.287           // meter (BURGER : 0.160, WAFFLE : 0.287)
#define TURNING_RADIUS                   0.1435          // meter (BURGER : 0.080, WAFFLE : 0.1435)
#define ROBOT_RADIUS                     0.220           // meter (BURGER : 0.105, WAFFLE : 0.220)
#define ENCODER_MIN                      -2147483648     // raw
#define ENCODER_MAX                      2147483648      // raw

#define LEFT                             0
#define RIGHT                            1
#define ARM_0                            2
#define ARM_1                            3
#define ARM_2                            4
#define ARM_3                            5
#define ARM_4                            6

#define ARM_JOINT_COUNT                  5
#define WHEEL_JOINT_COUNT                2 

#define VELOCITY_CONSTANT_VALUE          41.69998509  // V = (60 RPM / 0.229 (1 RPM)) / 2pi

#define MAX_LINEAR_VELOCITY              0.25   // m/s (BURGER => 0.22, WAFFLE => 0.25)
#define MAX_ANGULAR_VELOCITY             1.82   // rad/s (BURGER => 2.84, WAFFLE => 1.82)
#define VELOCITY_STEP                    0.01   // m/s
#define VELOCITY_LINEAR_X                0.01   // m/s
#define VELOCITY_ANGULAR_Z               0.1    // rad/s
#define SCALE_VELOCITY_LINEAR_X          1
#define SCALE_VELOCITY_ANGULAR_Z         1

#define TICK2RAD                         0.001533981    // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f
#define RAD2TICK                         651.898646917  // 180/PI[deg] * 11,377777778[tick]

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define WAIT_FOR_BUTTON_PRESS            0
#define WAIT_SECOND                      1
#define CHECK_BUTTON_RELEASED            2

// Callback function prototypes
void updateServoConfigCallback(const turtlebot3_msgs::DynamixelConfig& servo_config_msg);
void commandTorqueEnableCallback(const std_msgs::Bool& torque_enable);
void tetrisCallback(const std_msgs::Int32& tetris_enable);

void commandWheelJointLeftCallback(const std_msgs::Float64& position_msg);
void commandWheelJointRightCallback(const std_msgs::Float64& position_msg);

void commandArmJoint1PositionCallback(const std_msgs::Float64& position_msg);  
void commandArmJoint1VelocityCallback(const std_msgs::Float64& velocity_msg);
void commandArmJoint1EffortCallback(const std_msgs::Float64& effort_msg);
void commandArmJoint1ModeCallback(const std_msgs::Int32& mode_msg);

void commandArmJoint2PositionCallback(const std_msgs::Float64& position_msg);  
void commandArmJoint2VelocityCallback(const std_msgs::Float64& position_msg);
void commandArmJoint2EffortCallback(const std_msgs::Float64& position_msg);
void commandArmJoint2ModeCallback(const std_msgs::Int32& position_msg);

void commandArmJoint3PositionCallback(const std_msgs::Float64& position_msg);  
void commandArmJoint3VelocityCallback(const std_msgs::Float64& position_msg);
void commandArmJoint3EffortCallback(const std_msgs::Float64& position_msg);
void commandArmJoint3ModeCallback(const std_msgs::Int32& position_msg);

void commandArmJoint4PositionCallback(const std_msgs::Float64& position_msg);  
void commandArmJoint4VelocityCallback(const std_msgs::Float64& position_msg);
void commandArmJoint4EffortCallback(const std_msgs::Float64& position_msg);
void commandArmJoint4ModeCallback(const std_msgs::Int32& position_msg);

void commandArmJoint5PositionCallback(const std_msgs::Float64& position_msg);  
void commandArmJoint5VelocityCallback(const std_msgs::Float64& position_msg);
void commandArmJoint5EffortCallback(const std_msgs::Float64& position_msg);
void commandArmJoint5ModeCallback(const std_msgs::Int32& position_msg);

// Function prototypes
void publishVoltageMsg(void);
void publishImuMsg(void);
void publishJointStateMsg(void);

void updateJoint(void);

void controlWheelSpeed(void);
void controlMotorEffort(void);
void controlMotorVelocity(void);
void controlMotorPosition(void);

float checkVoltage(void);
void updateVoltageCheck(void);
void updateGyroCali(void);
void showLedStatus(void);
void updateRxTxLed(void);

void setPowerOn();
void setPowerOff();

#endif // TURTLEBOT3_CORE_CONFIG_H_

