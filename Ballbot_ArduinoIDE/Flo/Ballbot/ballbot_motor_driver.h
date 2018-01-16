
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

#ifndef BALLBOT_MOTOR_DRIVER_H_
#define BALLBOT_MOTOR_DRIVER_H_

#include <DynamixelSDK.h>

// Control table address (Dynamixel X-series) Okay
#define ADDR_X_MODE_CHANGE              11
#define ADDR_X_TORQUE_ENABLE            64
#define ADDR_X_GOAL_EFFORT              102
#define ADDR_X_GOAL_VELOCITY            104
#define ADDR_X_GOAL_POSITION            116
#define ADDR_X_REALTIME_TICK            120
#define ADDR_X_PRESENT_EFFORT           126
#define ADDR_X_PRESENT_VELOCITY         128
#define ADDR_X_PRESENT_POSITION         132

#define ADDR_X_BOARDRATE                8

// Limit values (XM430-W350-R) Okay
#define LIMIT_X_MAX_VELOCITY            240

// Data Byte Length
#define LEN_X_TORQUE_ENABLE             1
#define LEN_X_GOAL_EFFORT               2
#define LEN_X_GOAL_VELOCITY             4
#define LEN_X_GOAL_POSITION             4
#define LEN_X_REALTIME_TICK             2
#define LEN_X_PRESENT_EFFORT            2
#define LEN_X_PRESENT_VELOCITY          4
#define LEN_X_PRESENT_POSITION          4

#define PROTOCOL_VERSION                2.0     // Dynamixel protocol version 2.0

#define DXM_1_ID                        1
#define DXM_2_ID                        2
#define DXM_3_ID                        3
#define BAUDRATE                        4500000 // baurd rate of Dynamixel
#define DEVICENAME                      ""      // no need setting on OpenCR

#define WHEEL_JOINT_COUNT               3       

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

#define GOAL_POS                        2000
#define DXL__POS_THRESHOLD              1 


#define DEBUG_MOTOR

class BallbotMotorDriver
{
 public:
  BallbotMotorDriver();
  ~BallbotMotorDriver();
  bool init(void);
  void closeDynamixel(void);
  bool writeServoConfig(uint8_t id, uint8_t length, uint8_t address, int data);
  bool setTorque(uint8_t id, bool onoff);
  bool changeMode(uint8_t id, uint16_t mode);
  bool readWheelStates(int32_t wheel_effort_values[], int32_t wheel_velocity_values[], int32_t wheel_position_values[]);

 private:
  uint32_t baudrate_;
  float  protocol_version_;
  uint8_t wheel_1_id_;
  uint8_t wheel_2_id_;
  uint8_t wheel_3_id_;
 

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  dynamixel::GroupSyncWrite *groupSyncWriteEffort_;
  
  dynamixel::GroupBulkRead *groupBulkReadWheels_;

};

#endif // BALLBOT_MOTOR_DRIVER_H_

