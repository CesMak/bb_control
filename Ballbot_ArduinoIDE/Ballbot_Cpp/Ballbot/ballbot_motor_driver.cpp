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

#include "ballbot_motor_driver.h"

Turtlebot3MotorDriver::Turtlebot3MotorDriver()
//: baudrate_(BAUDRATE),
  //protocol_version_(PROTOCOL_VERSION),
  //left_wheel_id_(DXL_LEFT_ID),
  //right_wheel_id_(DXL_RIGHT_ID),
  //arm_0_id_(DXL_ARM_0_ID),
  //arm_1_id_(DXL_ARM_1_ID),
  //arm_2_id_(DXL_ARM_2_ID),
  //arm_3_id_(DXL_ARM_3_ID),
  //arm_4_id_(DXL_ARM_4_ID)
{
}

Turtlebot3MotorDriver::~Turtlebot3MotorDriver()
{
  closeDynamixel();
}

bool Turtlebot3MotorDriver::init(void)
{
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort())
  {
    #ifdef DEBUG
    sprintf(log_msg, "Port is Opened");
    nh.loginfo(log_msg);
    #endif
  }
  else
  {
    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_))
  {
    #ifdef DEBUG
    sprintf(log_msg, "Baudrate is set");
    nh.loginfo(log_msg);
    #endif
  }
  else
  {
    return false;
  }

  changeMode(left_wheel_id_, 1);
  changeMode(right_wheel_id_, 1);
  changeMode(arm_0_id_, 3);
  changeMode(arm_1_id_, 3);
  changeMode(arm_2_id_, 3);
  changeMode(arm_3_id_, 3);
  changeMode(arm_4_id_, 5);

  setTorque(left_wheel_id_, false);
  setTorque(right_wheel_id_, false);
  setTorque(arm_0_id_, false);
  setTorque(arm_1_id_, false);
  setTorque(arm_2_id_, false);
  setTorque(arm_3_id_, false);
  setTorque(arm_4_id_, false);
  
  groupSyncWriteEffort_           = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_EFFORT,   LEN_X_GOAL_EFFORT);
  groupSyncWriteVelocity_         = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  groupSyncWritePosition_         = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_POSITION, LEN_X_GOAL_POSITION);

  // init bulk reader for wheels (TTL)
  groupBulkReadWheels_ = new dynamixel::GroupBulkRead(portHandler_, packetHandler_);
  
  int wheel_ids[2] = {left_wheel_id_, right_wheel_id_};  
  for(int i = 0; i < WHEEL_JOINT_COUNT; i++)
  {
    if(!groupBulkReadWheels_->addParam(wheel_ids[i], ADDR_X_PRESENT_EFFORT, LEN_X_PRESENT_EFFORT+LEN_X_PRESENT_VELOCITY+LEN_X_PRESENT_POSITION))
      return false;
  }

  // init bulk reader for arm (RS485)
  groupBulkReadArm_ = new dynamixel::GroupBulkRead(portHandler_, packetHandler_);
  
  int arm_ids[ARM_JOINT_COUNT] = {arm_0_id_, arm_1_id_, arm_2_id_, arm_3_id_, arm_4_id_}; 
  for(int i = 0; i < ARM_JOINT_COUNT; i++)
  {
    if(!groupBulkReadArm_->addParam(arm_ids[i], ADDR_X_PRESENT_EFFORT, LEN_X_PRESENT_EFFORT+LEN_X_PRESENT_VELOCITY+LEN_X_PRESENT_POSITION))
      return false;
  }

  return true;
}

void Turtlebot3MotorDriver::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  setTorque(left_wheel_id_, false);
  setTorque(right_wheel_id_, false);

  // Close port
  portHandler_->closePort();
}

bool Turtlebot3MotorDriver::writeServoConfig(uint8_t id, uint8_t length, uint8_t address, uint8_t data)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  switch(length)
  {
    case 1:
      dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, address, data, &dxl_error);
      break;
    case 2:
      dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, address, data, &dxl_error);
      break;
    case 4:
      dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, address, data, &dxl_error);
      break;
  }
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->printRxPacketError(dxl_error);
  }

  int newBaudrate = data == 5 ? 3000000 : 2000000;

  if(address == ADDR_X_BOARDRATE && id == DXL_ARM_4_ID) 
  {
    if (portHandler_->setBaudRate(newBaudrate))
    {
      char logMsg[50];
      sprintf(logMsg, "Baudrate is set to %d", newBaudrate);
      nh.loginfo(logMsg);      
    } else {
      nh.logerror("Error in changing the Baudrate");
      return false;
    }
  }
  
  return true;
}

bool Turtlebot3MotorDriver::setTorque(uint8_t id, bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->printRxPacketError(dxl_error);
  }
}

bool Turtlebot3MotorDriver::changeMode(uint8_t id, uint16_t mode)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  setTorque(id, false);
  dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_X_MODE_CHANGE, mode, &dxl_error);
  setTorque(id, true);

  char errorLog[50];
  
  if(dxl_comm_result != COMM_SUCCESS)
  {
    sprintf(errorLog, "Error in changeMode communication (comm_result): %d", (int)dxl_comm_result); 
    nh.logerror(errorLog);
    packetHandler_->printTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    sprintf(errorLog, "Error in changeMode communication (dxl_error): %d", (int)dxl_error); 
    nh.logerror(errorLog);
    packetHandler_->printRxPacketError(dxl_error);
  }
}

bool Turtlebot3MotorDriver::readWheelStates(int32_t wheel_effort_values[], int32_t wheel_velocity_values[], int32_t wheel_position_values[])
{
  int  wheel_ids[2] = {left_wheel_id_, right_wheel_id_};  
  int  dxl_comm_result = COMM_TX_FAIL;
  bool dxl_getdata_result = false;
  
  dxl_comm_result = groupBulkReadWheels_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    packetHandler_->printTxRxResult(dxl_comm_result);

  for(int i = 0; i < WHEEL_JOINT_COUNT; i++)
  {
    dxl_getdata_result = groupBulkReadWheels_->isAvailable(wheel_ids[i], ADDR_X_PRESENT_EFFORT, LEN_X_PRESENT_EFFORT+LEN_X_PRESENT_VELOCITY+LEN_X_PRESENT_POSITION);
    if (dxl_getdata_result)
    {
      wheel_effort_values[i] = groupBulkReadWheels_->getData(wheel_ids[i], ADDR_X_PRESENT_EFFORT, LEN_X_PRESENT_EFFORT);
      wheel_velocity_values[i] = groupBulkReadWheels_->getData(wheel_ids[i], ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
      wheel_position_values[i] = groupBulkReadWheels_->getData(wheel_ids[i], ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    }
  }
  
  return dxl_getdata_result;
}

bool Turtlebot3MotorDriver::readArmStates(int32_t arm_effort_values[], int32_t arm_velocity_values[], int32_t arm_position_values[])
{
  int  arm_ids[ARM_JOINT_COUNT] = {arm_0_id_, arm_1_id_, arm_2_id_, arm_3_id_, arm_4_id_}; 
  int  dxl_comm_result = COMM_TX_FAIL;
  bool dxl_getdata_result = false;
  
  dxl_comm_result = groupBulkReadArm_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    packetHandler_->printTxRxResult(dxl_comm_result);

  for(int i = 0; i < ARM_JOINT_COUNT; i++)
  {
    dxl_getdata_result = groupBulkReadArm_->isAvailable(arm_ids[i], ADDR_X_PRESENT_EFFORT, LEN_X_PRESENT_EFFORT+LEN_X_PRESENT_VELOCITY+LEN_X_PRESENT_POSITION);
    if (dxl_getdata_result)
    {
      arm_effort_values[i] = groupBulkReadArm_->getData(arm_ids[i], ADDR_X_PRESENT_EFFORT, LEN_X_PRESENT_EFFORT);
      arm_velocity_values[i] = groupBulkReadArm_->getData(arm_ids[i], ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
      arm_position_values[i] = groupBulkReadArm_->getData(arm_ids[i], ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    }
  }
  
  return dxl_getdata_result;
}

//bool Turtlebot3MotorDriver::wheelSpeedControl(int64_t goal_vel_1, int64_t goal_vel_2)
//{
//  bool dxl_addparam_result_;
//  int8_t dxl_comm_result_;  
//
//  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(left_wheel_id_, (uint8_t*)&goal_vel_1);
//  if (dxl_addparam_result_ != true)
//    return false;
//
//  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(right_wheel_id_, (uint8_t*)&goal_vel_2);
//  if (dxl_addparam_result_ != true)
//    return false;
//
//  dxl_comm_result_ = groupSyncWriteVelocity_->txPacket();
//  if (dxl_comm_result_ != COMM_SUCCESS)
//  {
//    packetHandler_->printTxRxResult(dxl_comm_result_);
//    return false;
//  }
//
//  groupSyncWriteVelocity_->clearParam();
//  return true;
//}
//
//bool Turtlebot3MotorDriver::effortControlArm(int64_t goal_eff_1, int64_t goal_eff_2, int64_t goal_eff_3, int64_t goal_eff_4, int64_t goal_eff_5)
//{
//  bool dxl_addparam_result_;
//  int8_t dxl_comm_result_;
//
//  dxl_addparam_result_ = groupSyncWriteEffort_->addParam(arm_0_id_, (uint8_t*)&goal_eff_1);
//  if (dxl_addparam_result_ != true)
//    return false;
//
//  dxl_addparam_result_ = groupSyncWriteEffort_->addParam(arm_1_id_, (uint8_t*)&goal_eff_2);
//  if (dxl_addparam_result_ != true)
//    return false;
//
//  dxl_addparam_result_ = groupSyncWriteEffort_->addParam(arm_2_id_, (uint8_t*)&goal_eff_3);
//  if (dxl_addparam_result_ != true)
//    return false;
//
//  dxl_addparam_result_ = groupSyncWriteEffort_->addParam(arm_3_id_, (uint8_t*)&goal_eff_4);
//  if (dxl_addparam_result_ != true)
//    return false;
//
//  dxl_addparam_result_ = groupSyncWriteEffort_->addParam(arm_4_id_, (uint8_t*)&goal_eff_5);
//  if (dxl_addparam_result_ != true)
//    return false;
//
//  dxl_comm_result_ = groupSyncWriteEffort_->txPacket();
//  if (dxl_comm_result_ != COMM_SUCCESS)
//  {
//    packetHandler_->printTxRxResult(dxl_comm_result_);
//    return false;
//  }
//
//  groupSyncWriteEffort_->clearParam();
//  return true;
//}

//bool Turtlebot3MotorDriver::velocityControlArm(int64_t goal_vel_1, int64_t goal_vel_2, int64_t goal_vel_3, int64_t goal_vel_4, int64_t goal_vel_5)
//{
//  bool dxl_addparam_result_;
//  int8_t dxl_comm_result_;  
//
//  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(arm_0_id_, (uint8_t*)&goal_vel_1);
//  if (dxl_addparam_result_ != true)
//    return false;
//
//  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(arm_1_id_, (uint8_t*)&goal_vel_2);
//  if (dxl_addparam_result_ != true)
//    return false;
//
//  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(arm_2_id_, (uint8_t*)&goal_vel_3);
//  if (dxl_addparam_result_ != true)
//    return false;
//
//  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(arm_3_id_, (uint8_t*)&goal_vel_4);
//  if (dxl_addparam_result_ != true)
//    return false;
//
//  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(arm_4_id_, (uint8_t*)&goal_vel_5);
//  if (dxl_addparam_result_ != true)
//    return false;
//
//  dxl_comm_result_ = groupSyncWriteVelocity_->txPacket();
//  if (dxl_comm_result_ != COMM_SUCCESS)
//  {
//    packetHandler_->printTxRxResult(dxl_comm_result_);
//    return false;
//  }
//
//  groupSyncWriteVelocity_->clearParam();
//  return true;
//}
//
//bool Turtlebot3MotorDriver::positionControlArm(int64_t goal_pos_1, int64_t goal_pos_2, int64_t goal_pos_3, int64_t goal_pos_4, int64_t goal_pos_5)
//{
//  bool dxl_addparam_result_;
//  int8_t dxl_comm_result_;
//
//  dxl_addparam_result_ = groupSyncWritePosition_->addParam(arm_0_id_, (uint8_t*)&goal_pos_1);
//  if (dxl_addparam_result_ != true)
//    return false;
//
//  dxl_addparam_result_ = groupSyncWritePosition_->addParam(arm_1_id_, (uint8_t*)&goal_pos_2);
//  if (dxl_addparam_result_ != true)
//    return false;
//
//  dxl_addparam_result_ = groupSyncWritePosition_->addParam(arm_2_id_, (uint8_t*)&goal_pos_3);
//  if (dxl_addparam_result_ != true)
//    return false;
//
//  dxl_addparam_result_ = groupSyncWritePosition_->addParam(arm_3_id_, (uint8_t*)&goal_pos_4);
//  if (dxl_addparam_result_ != true)
//    return false;
//
//  dxl_addparam_result_ = groupSyncWritePosition_->addParam(arm_4_id_, (uint8_t*)&goal_pos_5);
//  if (dxl_addparam_result_ != true)
//    return false;
//
//  dxl_comm_result_ = groupSyncWritePosition_->txPacket();
//  if (dxl_comm_result_ != COMM_SUCCESS)
//  {
//    packetHandler_->printTxRxResult(dxl_comm_result_);
//    return false;
//  }
//
//  groupSyncWritePosition_->clearParam();
//  return true;
//}

