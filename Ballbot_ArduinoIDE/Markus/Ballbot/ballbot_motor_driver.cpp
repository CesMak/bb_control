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

extern BallbotMotorDriver motor_driver;

BallbotMotorDriver::BallbotMotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION),
  wheel_1_id_(DXM_1_ID),
  wheel_2_id_(DXM_2_ID),
  wheel_3_id_(DXM_3_ID)
{

}

BallbotMotorDriver::~BallbotMotorDriver()
{
  //closeDynamixel();
}

bool BallbotMotorDriver::init(void)
{
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort())
  {
    #ifdef DEBUG
    sprintf(log_msg, "Port of Port Handler is Opened");
    nh.loginfo(log_msg);
    #endif

    #ifdef DEBUG_MOTOR
    Serial.println("Port of Port Handler is Opened");
    #endif
  }
  else
  {
    return false;
  }

  // config the Baudrate of the port handler must be the same as for t he motors
  if (portHandler_->setBaudRate(baudrate_))
  {
    #ifdef DEBUG
    sprintf(log_msg, "Baudrate of Porthandler is set to ",baudrate_);
    nh.loginfo(log_msg);
    #endif

    #ifdef DEBUG_MOTOR
    Serial.print("Baudrate of Porthandler is set to "); Serial.println(baudrate_);
    #endif
  }
  else
  {
    return false;
  }

  //Wheel
  changeMode(wheel_1_id_,3);
  changeMode(wheel_2_id_,3);
  changeMode(wheel_3_id_,3);

  setTorque(wheel_1_id_, false); 
  setTorque(wheel_2_id_, false);
  setTorque(wheel_3_id_, false);

  // Objekt um Radposition etc. auszulesen:
  groupSyncWriteEffort_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_EFFORT,   LEN_X_GOAL_EFFORT);

  // init bulk reader for wheels (TTL / )
  groupBulkReadWheels_ = new dynamixel::GroupBulkRead(portHandler_, packetHandler_);
  
  int wheel_ids[3] ={wheel_1_id_,wheel_2_id_,wheel_3_id_};
  for(int i = 0; i < WHEEL_JOINT_COUNT; i++)
  {
    if(!groupBulkReadWheels_->addParam(wheel_ids[i], ADDR_X_PRESENT_EFFORT, LEN_X_PRESENT_EFFORT+LEN_X_PRESENT_VELOCITY+LEN_X_PRESENT_POSITION))
      return false;
  }

  return true;
}

void BallbotMotorDriver::referenzFahrt(void)
{
  Serial.println("Start Referenzfahrt");
  setTorque(DXM_1_ID,false);
  setTorque(DXM_2_ID,false);
  setTorque(DXM_3_ID,false);

  changeMode(DXM_1_ID,3);
  changeMode(DXM_2_ID,3);
  changeMode(DXM_3_ID,3);

  setTorque(DXM_1_ID,true);
  setTorque(DXM_2_ID,true);
  setTorque(DXM_3_ID,true);
  
  //ZeroPosition
  int32_t wheel_effort_values[3] = {0,0,0};
  int32_t wheel_velocity_values[3] = {0,0,0};
  int32_t wheel_position_values[3] = {0,0,0};
  int goal = 0; 
 
  writeServoConfig(DXM_1_ID, 4 , ADDR_X_GOAL_POSITION , goal);
  do
  { 
  readWheelStates(wheel_effort_values,wheel_velocity_values,wheel_position_values);
  }while(abs(goal - wheel_position_values[0]) > DXL__POS_THRESHOLD);

  Serial.print("  M1 okay");
  
  writeServoConfig(DXM_2_ID, 4 , ADDR_X_GOAL_POSITION , goal);
  do
  { 
    readWheelStates(wheel_effort_values,wheel_velocity_values,wheel_position_values);
  }while(abs(goal - wheel_position_values[1]) > DXL__POS_THRESHOLD);

  Serial.print("  M2 okay");
  
  writeServoConfig(DXM_3_ID, 4 , ADDR_X_GOAL_POSITION , goal);
  do
  { 
    readWheelStates(wheel_effort_values,wheel_velocity_values,wheel_position_values);
  }while(abs(goal - wheel_position_values[2]) > DXL__POS_THRESHOLD);

  Serial.print("  M3 okay");
  Serial.println("End Referenzfahrt");
}

void BallbotMotorDriver::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  setTorque(wheel_1_id_, false);
  setTorque(wheel_2_id_, false);
  setTorque(wheel_3_id_, false);

  // Close port
  portHandler_->closePort();
}

bool BallbotMotorDriver::writeServoConfig(uint8_t id, uint8_t length, uint8_t address, int data)
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
  
  return true;
}

int BallbotMotorDriver::readServoConfig(uint8_t id, uint8_t length, uint16_t address)
{
  // turn on motors first to read out values of the motors!
  uint8_t  data1 = 0;
  uint16_t data2 = 0;
  uint32_t data3 = 0;
  uint8_t  dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  switch(length)
  {
    case 1:
      dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, id, address, (uint8_t*)&data1, &dxl_error);
      return (int) data1;
      break;
    case 2:
      dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, address, &data2, &dxl_error);
      return (int) data2;
      break;
    case 4:
      dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, id, address, &data3, &dxl_error);
      return (int) data3;
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
  
  return dxl_comm_result; // this is just zero if no error is there!
}

bool BallbotMotorDriver::setTorque(uint8_t id, bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    #ifdef DEBUG
    packetHandler_->printTxRxResult(dxl_comm_result);
    #endif

    #ifdef DEBUG_MOTOR
    Serial.println("Error in Setting Torque (comm_result not COMM_SUCCESS)");
    Serial.println(dxl_comm_result);
    #endif
  }
  else if(dxl_error != 0)
  {
    #ifdef DEBUG
    packetHandler_->printRxPacketError(dxl_error);
    #endif

    #ifdef DEBUG_MOTOR
    Serial.println("Error in Setting Torque (dxm_error not 0)");
    #endif
    
  }
}

bool BallbotMotorDriver::changeMode(uint8_t id, uint16_t mode)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  setTorque(id, false);
  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_X_MODE_CHANGE, mode, &dxl_error);
  setTorque(id, true);

  char errorLog[50];
  
  if(dxl_comm_result != COMM_SUCCESS)
  {
    #ifdef DEBUG
    sprintf(errorLog, "Error in changeMode communication (comm_result): %d", (int)dxl_comm_result); 
    //nh.logerror(errorLog);
    packetHandler_->printTxRxResult(dxl_comm_result);
    #endif
    
    #ifdef DEBUG_MOTOR
    Serial.println("Error in changeMode communication (comm_result)");
    #endif
    
  }
  else if(dxl_error != 0)
  {
    #ifdef DEBUG
    sprintf(errorLog, "Error in changeMode communication (dxl_error): %d", (int)dxl_error); 
    //nh.logerror(errorLog);
    packetHandler_->printRxPacketError(dxl_error);
    #endif

    #ifdef DEBUG_MOTOR
    Serial.println("Error in changeMode communication (dxm_error)");
    #endif
    
  }
}

bool BallbotMotorDriver::readWheelStates(int32_t wheel_effort_values[], int32_t wheel_velocity_values[], int32_t wheel_position_values[])
{ 
  int wheel_ids[3] ={wheel_1_id_, wheel_2_id_, wheel_3_id_};
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

// new baud numbers 1-7 3:=1M 7:=4.5M  
bool BallbotMotorDriver::writeBaudrate(int newBaud)
{
  writeServoConfig(wheel_1_id_, 1, 8, newBaud);
  writeServoConfig(wheel_2_id_, 1, 8, newBaud);
  writeServoConfig(wheel_3_id_, 1, 8, newBaud);
}



