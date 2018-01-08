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

#include "turtlebot3_core_config.h"
#include "turtlebot3_signal_manager.h"
#include "JointControllerState.h"

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<turtlebot3_msgs::DynamixelConfig> cmd_config_servo("joints/update_servo_config", updateServoConfigCallback);

ros::Subscriber<std_msgs::Int32> cmd_tetris("signals/tetris", tetrisCallback);

ros::Subscriber<std_msgs::Bool> cmd_torque_enable("joints/torque_enable", commandTorqueEnableCallback);

ros::Subscriber<std_msgs::Float64> cmd_joint_wheel_left("joints/wheel_left_controller/command", commandWheelJointLeftCallback);
ros::Subscriber<std_msgs::Float64> cmd_joint_wheel_right("joints/wheel_right_controller/command", commandWheelJointRightCallback);

ros::Subscriber<std_msgs::Float64> cmd_joint_1_position("joints/arm_joint_1_position/command", commandArmJoint1PositionCallback);
ros::Subscriber<std_msgs::Float64> cmd_joint_1_velocity("joints/arm_joint_1_velocity/command", commandArmJoint1VelocityCallback);
ros::Subscriber<std_msgs::Float64> cmd_joint_1_effort("joints/arm_joint_1_effort/command", commandArmJoint1EffortCallback);
ros::Subscriber<std_msgs::Int32> cmd_joint_1_mode("joints/arm_joint_1_mode", commandArmJoint1ModeCallback);

ros::Subscriber<std_msgs::Float64> cmd_joint_2_position("joints/arm_joint_2_position/command", commandArmJoint2PositionCallback);
ros::Subscriber<std_msgs::Float64> cmd_joint_2_velocity("joints/arm_joint_2_velocity/command", commandArmJoint2VelocityCallback);
ros::Subscriber<std_msgs::Float64> cmd_joint_2_effort("joints/arm_joint_2_effort/command", commandArmJoint2EffortCallback);
ros::Subscriber<std_msgs::Int32> cmd_joint_2_mode("joints/arm_joint_2_mode", commandArmJoint2ModeCallback);

ros::Subscriber<std_msgs::Float64> cmd_joint_3_position("joints/arm_joint_3_position/command", commandArmJoint3PositionCallback);
ros::Subscriber<std_msgs::Float64> cmd_joint_3_velocity("joints/arm_joint_3_velocity/command", commandArmJoint3VelocityCallback);
ros::Subscriber<std_msgs::Float64> cmd_joint_3_effort("joints/arm_joint_3_effort/command", commandArmJoint3EffortCallback);
ros::Subscriber<std_msgs::Int32> cmd_joint_3_mode("joints/arm_joint_3_mode", commandArmJoint3ModeCallback);

ros::Subscriber<std_msgs::Float64> cmd_joint_4_position("joints/arm_joint_4_position/command", commandArmJoint4PositionCallback);
ros::Subscriber<std_msgs::Float64> cmd_joint_4_velocity("joints/arm_joint_4_velocity/command", commandArmJoint4VelocityCallback);
ros::Subscriber<std_msgs::Float64> cmd_joint_4_effort("joints/arm_joint_4_effort/command", commandArmJoint4EffortCallback);
ros::Subscriber<std_msgs::Int32> cmd_joint_4_mode("joints/arm_joint_4_mode", commandArmJoint4ModeCallback);

ros::Subscriber<std_msgs::Float64> cmd_joint_5_position("joints/gripper_servo_joint_position/command", commandArmJoint5PositionCallback);
ros::Subscriber<std_msgs::Float64> cmd_joint_5_velocity("joints/gripper_servo_joint_velocity/command", commandArmJoint5VelocityCallback);
ros::Subscriber<std_msgs::Float64> cmd_joint_5_effort("joints/gripper_servo_joint_effort/command", commandArmJoint5EffortCallback);
ros::Subscriber<std_msgs::Int32> cmd_joint_5_mode("joints/gripper_servo_joint_mode", commandArmJoint5ModeCallback);


/*******************************************************************************
* Publisher
*******************************************************************************/
// IMU of Turtlebot3
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("sensor/imu", &imu_msg);

// Joint states control messages of each joint
sensor_msgs::JointState joint_states_msg;
ros::Publisher joint_states_pub("joints/joint_states", &joint_states_msg);

// Joint controller states of each joint
control_msgs::JointControllerState wheel_left_state_msg;
ros::Publisher wheel_left_state_pub("joints/wheel_left_controller/state", &wheel_left_state_msg);
control_msgs::JointControllerState wheel_right_state_msg;
ros::Publisher wheel_right_state_pub("joints/wheel_right_controller/state", &wheel_right_state_msg);

// Voltage Publisher
std_msgs::Float64 voltage_msg;
ros::Publisher voltage_pub("sensor/voltage", &voltage_msg);

/*******************************************************************************
* SoftwareTimer of Turtlebot3
*******************************************************************************/
static uint32_t tTime[7];

/*******************************************************************************
* Declaration for arm motor controlling
*******************************************************************************/
int64_t cmd_position[5] = {2048, 2048, 2048, 2048, 2048};
double cmd_velocity[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
double cmd_effort[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
double cmd_velocity_wheels[2] = {0.0, 0.0};
int8_t mode[5] = {3, 3, 3, 3, 3};
bool torque_joints = false;

bool arm_comm = true;
bool wheel_comm = true;
bool error = false;
bool connection = false;

/*******************************************************************************
* Signal Manager
*******************************************************************************/
Turtlebot3SignalManager signal_manager(BDPIN_BUZZER);

/*******************************************************************************
* Declaration for motor
*******************************************************************************/
Turtlebot3MotorDriver motor_driver(nh);
float current_effort_[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float current_velocity_[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float current_position_[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

float position_offset[7] = {0.0, 0.0, 0.0, 0.0, -1.57079632679, 0.0, 0.0};

/*******************************************************************************
* Declaration for IMU
*******************************************************************************/
cIMU imu;

/*******************************************************************************
* Declaration for driving
*******************************************************************************/
int32_t effort_data_arm[5] = {0, 0, 0, 0, 0};
int32_t velocity_data_arm[5] = {0, 0, 0, 0, 0};
int32_t position_data_arm[5] = {0, 0, 0, 0, 0};

int32_t effort_data_wheels[2] = {0, 0};
int32_t velocity_data_wheels[2] = {0, 0};
int32_t position_data_wheels[2] = {0, 0};

/*******************************************************************************
* Declaration for test drive
*******************************************************************************/
bool start_move = false;
bool start_rotate = false;

/*******************************************************************************
* Declaration for joint states
*******************************************************************************/
char *joint_states_name[] = {"wheel_left_joint", "wheel_right_joint", "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "gripper_servo_joint"};

/*******************************************************************************
* Declaration for LED
*******************************************************************************/
#define LED_TXD         0
#define LED_RXD         1
#define LED_LOW_BATTERY 2
#define LED_ROS_CONNECT 3

/*******************************************************************************
* Declaration for Battery
*******************************************************************************/
#define BATTERY_POWER_OFF             0
#define BATTERY_POWER_STARTUP         1
#define BATTERY_POWER_NORMAL          2
#define BATTERY_POWER_CHECK           3
#define BATTERY_POWER_WARNNING        4

static bool    setup_end       = false;
static uint8_t battery_voltage = 0;
static float   battery_valtage_raw = 0;
static uint8_t battery_state   = BATTERY_POWER_OFF;

/*******************************************************************************
* Setup function
*******************************************************************************/
void setup()
{  
  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(3000000);

  nh.subscribe(cmd_config_servo);
  nh.subscribe(cmd_torque_enable);
  nh.subscribe(cmd_tetris);

  nh.subscribe(cmd_joint_wheel_left);
  nh.subscribe(cmd_joint_wheel_right);

  nh.subscribe(cmd_joint_1_position);
  nh.subscribe(cmd_joint_1_velocity);
  nh.subscribe(cmd_joint_1_effort);
  nh.subscribe(cmd_joint_1_mode);

  nh.subscribe(cmd_joint_2_position);
  nh.subscribe(cmd_joint_2_velocity);
  nh.subscribe(cmd_joint_2_effort);
  nh.subscribe(cmd_joint_2_mode);

  nh.subscribe(cmd_joint_3_position);
  nh.subscribe(cmd_joint_3_velocity);
  nh.subscribe(cmd_joint_3_effort);
  nh.subscribe(cmd_joint_3_mode);

  nh.subscribe(cmd_joint_4_position);
  nh.subscribe(cmd_joint_4_velocity);
  nh.subscribe(cmd_joint_4_effort);
  nh.subscribe(cmd_joint_4_mode);

  nh.subscribe(cmd_joint_5_position);
  nh.subscribe(cmd_joint_5_velocity);
  nh.subscribe(cmd_joint_5_effort);
  nh.subscribe(cmd_joint_5_mode);

  nh.advertise(imu_pub);
  nh.advertise(joint_states_pub);
  nh.advertise(wheel_left_state_pub);
  nh.advertise(wheel_right_state_pub);
  nh.advertise(voltage_pub);

  nh.loginfo("Connected to OpenCR board!");

  // Setting for Dynamixel motors
  motor_driver.init();

  // Set default operation mode for the arm to position control

  // Setting for IMU
  imu.begin();
  
  // Setting up the joint state messages
  joint_states_msg.header.frame_id = "base_link";
  joint_states_msg.name            = joint_states_name;

  joint_states_msg.name_length     = 7;
  joint_states_msg.position_length = 7;
  joint_states_msg.velocity_length = 7;
  joint_states_msg.effort_length   = 7;

  joint_states_msg.effort   = current_effort_;
  joint_states_msg.velocity = current_velocity_;
  joint_states_msg.position = current_position_;

  pinMode(13, OUTPUT);

  SerialBT2.begin(57600);

  setup_end = true;
}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop()
{
  // torque off motors on disconnect
  if (nh.connected() && !connection)
  {
    connection = true;
    nh.loginfo("Upboard connected");
    signal_manager.playTone(H, 500);
    signal_manager.playTone(E*2, 500);
  }
  if (!nh.connected() && connection)
  {
    signal_manager.playTone(E*2, 500);
    signal_manager.playTone(H, 500);
    connection = false;
    nh.logerror("Connection lost");
    std_msgs::Bool enable;
    enable.data = false;
    commandTorqueEnableCallback(enable);
  }
  
  int timer = millis();
  
  if ((timer-tTime[0]) >= (1000 / DRIVE_INFORMATION_PUBLISH_PERIOD))
  {
    publishJointStateMsg();
    tTime[0] = timer;

    if(!arm_comm && !error)
    {
      error = true;
      nh.logerror("Error occured in communication with the arm.");
    } 
    if(!wheel_comm && !error)
    {
      error = true;
      nh.logerror("Error occured in communication with the wheels.");
    }
  }
  
  if ((timer-tTime[1]) >= (1000 / CONTROL_MOTOR_SPEED_PERIOD))
  {
    for(int i = 0; i < ARM_JOINT_COUNT && arm_comm && torque_joints; i++)
    {
      if(mode[i] == 0)
      {
        controlMotorEffort();
      }
    }
    tTime[1] = timer;
  }

  if ((timer-tTime[2]) >= (1000 / CONTROL_MOTOR_SPEED_PERIOD))
  {
    for(int i = 0; i < ARM_JOINT_COUNT && arm_comm && torque_joints; i++)
    {
      if(mode[i] == 1)
      {
        controlMotorVelocity();
      }
    }
    tTime[2] = timer;
  }

  if ((timer-tTime[3]) >= (1000 / CONTROL_MOTOR_SPEED_PERIOD))
  {
    for(int i = 0; i < ARM_JOINT_COUNT && arm_comm && torque_joints; i++)
    {
      if(mode[i] == 3)
      {
        controlMotorPosition();
      }
    }
    tTime[3] = timer;
  }

  if ((timer-tTime[4]) >= (1000 / CONTROL_MOTOR_SPEED_PERIOD))
  {
    if(wheel_comm && torque_joints)
    {
      controlWheelSpeed();
    }
    tTime[4] = timer;
  }

  if ((timer-tTime[5]) >= (1000 / IMU_PUBLISH_PERIOD))
  {
    publishImuMsg();
    tTime[5] = timer;
  }

  if ((timer-tTime[6]) >= (1000 / VOLTAGE_PUBLISH_PERIOD))
  {
    publishVoltageMsg();
    tTime[6] = timer;
  }

  // Update the IMU unit
  imu.update();

  // Start Gyro Calibration after ROS connection
  updateGyroCali();

  // Show LED status
  showLedStatus();

  // Update Voltage
  updateVoltageCheck();

  // Print warnings in case of comm error
  if (!wheel_comm)
    nh.logerror("Error in Communication with the wheels");
  if (!arm_comm)
    nh.logerror("Error in Communication with the arm");

  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();
}

/*******************************************************************************
* Callback functions for motor controlling
*******************************************************************************/
void updateServoConfigCallback(const turtlebot3_msgs::DynamixelConfig& servo_config_msg)
{
  uint8_t id = servo_config_msg.id;
  uint8_t length = servo_config_msg.length;
  uint8_t address = servo_config_msg.address;
  uint32_t data = servo_config_msg.data;

  bool comm_error = true;
  
  if((torque_joints == false || address >= 64) && id != 0)
  {
    comm_error = motor_driver.writeServoConfig(id, length, address, data);
  } else if((torque_joints == false || address < 64) && id == 0) { 
    comm_error = comm_error && motor_driver.writeServoConfig(1, length, address, data);
    comm_error = comm_error && motor_driver.writeServoConfig(2, length, address, data);
    comm_error = comm_error && motor_driver.writeServoConfig(3, length, address, data);
    comm_error = comm_error && motor_driver.writeServoConfig(4, length, address, data);
    comm_error = comm_error && motor_driver.writeServoConfig(5, length, address, data);
    comm_error = comm_error && motor_driver.writeServoConfig(6, length, address, data);
    comm_error = comm_error && motor_driver.writeServoConfig(7, length, address, data);
  }else {
    nh.logerror("Writing addresses in EEPROM area is not allowed in torque mode");
  }

  if(!comm_error) {
    nh.logerror("Writing the servo configuration was not successfull: Communication error");
    return;
  }

  nh.logwarn("Changed servo configuration, restart of System is recommended");
}

void tetrisCallback(const std_msgs::Int32& tetris_enable)
{
  signal_manager.playTetris();
}

void commandTorqueEnableCallback(const std_msgs::Bool& torque_enable)
{
  bool torque_switch = torque_enable.data;

  for(int i = 0; i < 5; i++) 
  {
    double tmp = current_position_[i+2] - position_offset[i+2];
    if(tmp <= 0 && tmp <= -PI)
    {
      tmp = tmp + (2 * PI);
    }
    else if(tmp > 0 && tmp > PI)
    {
      tmp = tmp - (2 * PI);
    }
  
    cmd_position[i] = ((int64_t)(RAD2TICK * tmp) + 2048) % 4096;
  }
  
  if(torque_switch == true)
  {
    for(int i = 1; i < 8; i++)
    {
      motor_driver.setTorque(i, true);
      torque_joints = true;
    } 
  }
  else
  {
    for(int i = 1; i < 8; i++)
    {
      motor_driver.setTorque(i, false);
      torque_joints = false;
    }
  }
}

void commandWheelJointLeftCallback(const std_msgs::Float64& velocity_msg) 
{
  cmd_velocity_wheels[0] = velocity_msg.data * VELOCITY_CONSTANT_VALUE;
  wheel_left_state_msg.command = cmd_velocity_wheels[0];
}

void commandWheelJointRightCallback(const std_msgs::Float64& velocity_msg)
{
  cmd_velocity_wheels[1] = velocity_msg.data * VELOCITY_CONSTANT_VALUE;
  wheel_right_state_msg.command = cmd_velocity_wheels[1];
}

void commandArmJoint1PositionCallback(const std_msgs::Float64& position_msg) 
{
  double tmp = position_msg.data - position_offset[2];
  while(tmp > abs(PI))
  {
    tmp = (tmp > PI) ? (tmp - (2 * PI)) : (tmp + (2 * PI));
  }
  
  cmd_position[0] = ((int64_t)(RAD2TICK * tmp) + 2048) % 4096;
}

void commandArmJoint1VelocityCallback(const std_msgs::Float64& velocity_msg)
{
  cmd_velocity[2] = velocity_msg.data * VELOCITY_CONSTANT_VALUE;
}

void commandArmJoint1EffortCallback(const std_msgs::Float64& effort_msg)
{
  cmd_effort[0] = effort_msg.data;
}

void commandArmJoint1ModeCallback(const std_msgs::Int32& mode_msg)
{
  cmd_velocity[0] = 0;
  cmd_effort[0] = 0;
  cmd_position[0] = ((int64_t)(RAD2TICK * current_position_[2]) + 2048) % 4096;
  mode[0] = mode_msg.data; 
  motor_driver.changeMode(3, mode_msg.data);
}

void commandArmJoint2PositionCallback(const std_msgs::Float64& position_msg) 
{
  double tmp = position_msg.data - position_offset[3];
  while(tmp > abs(PI))
  {
    tmp = (tmp > PI) ? (tmp - (2 * PI)) : (tmp + (2 * PI));
  }
  
  cmd_position[1] = ((int64_t)(RAD2TICK * tmp) + 2048) % 4096;
}

void commandArmJoint2VelocityCallback(const std_msgs::Float64& velocity_msg)
{
  cmd_velocity[1] = velocity_msg.data * VELOCITY_CONSTANT_VALUE;;
}

void commandArmJoint2EffortCallback(const std_msgs::Float64& effort_msg)
{
  cmd_effort[1] = effort_msg.data;
}

void commandArmJoint2ModeCallback(const std_msgs::Int32& mode_msg)
{
  cmd_velocity[1] = 0;
  cmd_effort[1] = 0;
  cmd_position[1] = ((int64_t)(RAD2TICK * current_position_[3]) + 2048) % 4096;
  mode[1] = mode_msg.data; 
  motor_driver.changeMode(4, mode_msg.data);
}
void commandArmJoint3PositionCallback(const std_msgs::Float64& position_msg) 
{
  double tmp = position_msg.data - position_offset[4];
  while(tmp > abs(PI))
  {
    tmp = (tmp > PI) ? (tmp - (2 * PI)) : (tmp + (2 * PI));
  }
  
  cmd_position[2] = ((int64_t)(RAD2TICK * tmp) + 2048) % 4096;
}

void commandArmJoint3VelocityCallback(const std_msgs::Float64& velocity_msg)
{
  cmd_velocity[2] = velocity_msg.data * VELOCITY_CONSTANT_VALUE;;
}

void commandArmJoint3EffortCallback(const std_msgs::Float64& effort_msg)
{
  cmd_effort[2] = effort_msg.data;
}

void commandArmJoint3ModeCallback(const std_msgs::Int32& mode_msg)
{
  cmd_velocity[2] = 0;
  cmd_effort[2] = 0;
  cmd_position[2] = ((int64_t)(RAD2TICK * current_position_[4]) + 2048) % 4096;
  mode[2] = mode_msg.data; 
  motor_driver.changeMode(5, mode_msg.data);
}

void commandArmJoint4PositionCallback(const std_msgs::Float64& position_msg) 
{
  double tmp = position_msg.data - position_offset[5];
  while(tmp > abs(PI))
  {
    tmp = (tmp > PI) ? (tmp - (2 * PI)) : (tmp + (2 * PI));
  }
  
  cmd_position[3] = ((int64_t)(RAD2TICK * tmp) + 2048) % 4096;
}

void commandArmJoint4VelocityCallback(const std_msgs::Float64& velocity_msg)
{
  cmd_velocity[3] = velocity_msg.data * VELOCITY_CONSTANT_VALUE;;
}

void commandArmJoint4EffortCallback(const std_msgs::Float64& effort_msg)
{
  cmd_effort[3] = effort_msg.data;
}

void commandArmJoint4ModeCallback(const std_msgs::Int32& mode_msg)
{
  cmd_velocity[3] = 0;
  cmd_effort[3] = 0;
  cmd_position[3] = ((int64_t)(RAD2TICK * current_position_[5]) + 2048) % 4096;
  mode[3] = mode_msg.data; 
  motor_driver.changeMode(6, mode_msg.data);
}
void commandArmJoint5PositionCallback(const std_msgs::Float64& position_msg) 
{
  double tmp = position_msg.data - position_offset[6];
  while(tmp > abs(PI))
  {
    tmp = (tmp > PI) ? (tmp - (2 * PI)) : (tmp + (2 * PI));
  }

  int64_t goal = ((int64_t)(RAD2TICK * tmp) + 2048) % 4096;

  if(goal > GRIPPER_MAX_LIMIT)
  {
    goal = GRIPPER_MAX_LIMIT;
  } else if(goal < GRIPPER_MIN_LIMIT){
    goal = GRIPPER_MIN_LIMIT;
  }
  
  cmd_position[4] = goal;
}

void commandArmJoint5VelocityCallback(const std_msgs::Float64& velocity_msg)
{
  cmd_velocity[4] = velocity_msg.data * VELOCITY_CONSTANT_VALUE;;
}

void commandArmJoint5EffortCallback(const std_msgs::Float64& effort_msg)
{
  cmd_effort[4] = effort_msg.data;
}

void commandArmJoint5ModeCallback(const std_msgs::Int32& mode_msg)
{
  cmd_velocity[4] = 0;
  cmd_effort[4] = 0;
  cmd_position[4] = current_position_[6];
  mode[4] = mode_msg.data; 
  motor_driver.changeMode(7, mode_msg.data);
}

/*******************************************************************************
* Publish current voltage of the battery
*******************************************************************************/
void publishVoltageMsg(void) 
{
  voltage_msg.data = battery_valtage_raw;
  voltage_pub.publish(&voltage_msg);
}

/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg.header.stamp    = nh.now();
  imu_msg.header.frame_id = "imu_link";

  imu_msg.angular_velocity.x = imu.SEN.gyroADC[1] * -1;
  imu_msg.angular_velocity.y = imu.SEN.gyroADC[0];
  imu_msg.angular_velocity.z = imu.SEN.gyroADC[2];
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0.02;

  imu_msg.linear_acceleration.x = imu.SEN.accADC[1] * -1;
  imu_msg.linear_acceleration.y = imu.SEN.accADC[0];
  imu_msg.linear_acceleration.z = imu.SEN.accADC[2];
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  imu_msg.orientation.w = imu.quat[0];
  imu_msg.orientation.x = imu.quat[2] * -1;
  imu_msg.orientation.y = imu.quat[1];
  imu_msg.orientation.z = imu.quat[3];

  imu_msg.orientation_covariance[0] = 0.0025;
  imu_msg.orientation_covariance[1] = 0;
  imu_msg.orientation_covariance[2] = 0;
  imu_msg.orientation_covariance[3] = 0;
  imu_msg.orientation_covariance[4] = 0.0025;
  imu_msg.orientation_covariance[5] = 0;
  imu_msg.orientation_covariance[6] = 0;
  imu_msg.orientation_covariance[7] = 0;
  imu_msg.orientation_covariance[8] = 0.0025;

  imu_pub.publish(&imu_msg);
}

/*******************************************************************************
* Publish joint state of all joints
*******************************************************************************/
void publishJointStateMsg(void)
{
  wheel_comm = motor_driver.readWheelStates(effort_data_wheels, velocity_data_wheels, position_data_wheels); 
  arm_comm = motor_driver.readArmStates(effort_data_arm, velocity_data_arm, position_data_arm);
  
  joint_states_msg.header.stamp = nh.now();

  if (wheel_comm)
  {
    for (int i = 0; i < 2; i++)
    {
      current_effort_[i] = effort_data_wheels[i];
      current_velocity_[i] = velocity_data_wheels[i] / VELOCITY_CONSTANT_VALUE;
  
      int tmp;
      tmp = ((position_data_wheels[i] + 2048) % 4096);
      if(tmp >= 0 && tmp <= 2048)
      {
        current_position_[i] = tmp * TICK2RAD + position_offset[i];
      }
      else
      {
        current_position_[i] = (tmp - 4096) * TICK2RAD + position_offset[i]; 
      }
    }
  } 
    
  if (arm_comm)
  {
    for (int i = 0; i < 5; i++)
    {
      current_effort_[i+2] = effort_data_arm[i];
      current_velocity_[i+2] = velocity_data_arm[i] / VELOCITY_CONSTANT_VALUE;
  
      int tmp;
      tmp = ((position_data_arm[i] + 2048) % 4096);
      if(tmp >= 0 && tmp <= 2048)
      {
        current_position_[i+2] = tmp * TICK2RAD + position_offset[i+2];
      }
      else
      {
        current_position_[i+2] = (tmp - 4096) * TICK2RAD + position_offset[i+2]; 
      }
    }
  }

  if(arm_comm && wheel_comm) error = false;
  
  joint_states_pub.publish(&joint_states_msg);

  
  wheel_left_state_msg.header.stamp = joint_states_msg.header.stamp;
  wheel_left_state_msg.process_value = current_velocity_[0];
  wheel_left_state_pub.publish(&wheel_left_state_msg);
  
  wheel_right_state_msg.header.stamp = joint_states_msg.header.stamp;
  wheel_right_state_msg.process_value = current_velocity_[1];
  wheel_right_state_pub.publish(&wheel_right_state_msg);

  // TODO: Handle arm joints!
}

/*******************************************************************************
* Control motor velocity
*******************************************************************************/
void controlWheelSpeed(void)
{
  bool dxl_comm_result = false;

  int wheel_speed[2] = {0, 0};
  for(int i = 0; i < 2; i++)
  {
    wheel_speed[i] = cmd_velocity_wheels[i];

    if(wheel_speed[i] > LIMIT_X_MAX_VELOCITY)
    {
      wheel_speed[i] = LIMIT_X_MAX_VELOCITY;
    }
    else if(wheel_speed[i] < -LIMIT_X_MAX_VELOCITY)
    {
      wheel_speed[i] = -LIMIT_X_MAX_VELOCITY;
    }
  }
  
  wheel_comm = motor_driver.wheelSpeedControl(wheel_speed[0], wheel_speed[1]);
}

/*******************************************************************************
* Control motor effort
*******************************************************************************/
void controlMotorEffort(void)
{
  bool dxl_comm_result = false;
  
  arm_comm = motor_driver.effortControlArm(cmd_effort[0], cmd_effort[1], cmd_effort[2], cmd_effort[3], cmd_effort[4]);
}

/*******************************************************************************
* Control motor velocity
*******************************************************************************/
void controlMotorVelocity(void)
{
  bool dxl_comm_result = false;

  int motor_speed[5] = {0, 0, 0, 0, 0};
  for(int i = 0; i < 5; i++)
  {
    motor_speed[i] = cmd_velocity[i];

    if(motor_speed[i] > LIMIT_X_MAX_VELOCITY)
    {
      motor_speed[i] = LIMIT_X_MAX_VELOCITY;
    }
    else if(motor_speed[i] < -LIMIT_X_MAX_VELOCITY)
    {
      motor_speed[i] = -LIMIT_X_MAX_VELOCITY;
    }
  }
  
  arm_comm = motor_driver.velocityControlArm(motor_speed[0], motor_speed[1], motor_speed[2], motor_speed[3], motor_speed[4]);
}

/*******************************************************************************
* Control motor position
*******************************************************************************/
void controlMotorPosition(void)
{
  bool dxl_comm_result = false;
  
  arm_comm = motor_driver.positionControlArm(cmd_position[0], cmd_position[1], cmd_position[2], cmd_position[3], cmd_position[4]);
}

/*******************************************************************************
* Check voltage
*******************************************************************************/
float checkVoltage(void)
{
  float vol_value;

  vol_value = getPowerInVoltage();

  return vol_value;
}

/*******************************************************************************
* updateVoltageCheck
*******************************************************************************/
void updateVoltageCheck(void)
{
  static bool startup = false;
  static int vol_index = 0;
  static int prev_state = 0;
  static int alram_state = 0;
  static int check_index = 0;

  int i;
  float vol_sum;
  float vol_value;

  static uint32_t process_time[8] = {0,};
  static float    vol_value_tbl[10] = {0,};

  float voltage_ref       = 11.0 + 0.0;
  float voltage_ref_warn  = 11.0 + 0.0;


  if (startup == false)
  {
    startup = true;
    for (i=0; i<8; i++)
    {
      process_time[i] = millis();
    }
  }

  if (millis()-process_time[0] > 100)
  {
    process_time[0] = millis();

    vol_value_tbl[vol_index] = getPowerInVoltage();

    vol_index++;
    vol_index %= 10;

    vol_sum = 0;
    for(i=0; i<10; i++)
    {
        vol_sum += vol_value_tbl[i];
    }
    vol_value = vol_sum/10;
    battery_valtage_raw = vol_value;

    //Serial.println(vol_value);

    battery_voltage = vol_value;
  }


  if (millis()-process_time[1] > 1000)
  {
    process_time[1] = millis();

    //Serial.println(battery_state);

    switch(battery_state)
    {
      case BATTERY_POWER_OFF:
        if (setup_end == true)
        {
          alram_state = 0;
          if(battery_valtage_raw > 5.0)
          {
            check_index   = 0;
            prev_state    = battery_state;
            battery_state = BATTERY_POWER_STARTUP;
          }
          else
          {
            noTone(BDPIN_BUZZER);
          }
        }
        break;

      case BATTERY_POWER_STARTUP:
        if(battery_valtage_raw > voltage_ref)
        {
          check_index   = 0;
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_NORMAL;
          setPowerOn();
        }

        if(check_index < 5)
        {
          check_index++;
        }
        else
        {
          if (battery_valtage_raw > 5.0)
          {
            prev_state    = battery_state;
            battery_state = BATTERY_POWER_CHECK;
          }
          else
          {
            prev_state    = battery_state;
            battery_state = BATTERY_POWER_OFF;
          }
        }
        break;

      case BATTERY_POWER_NORMAL:
        alram_state = 0;
        if(battery_valtage_raw < voltage_ref)
        {
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_CHECK;
          check_index   = 0;
        }
        break;

      case BATTERY_POWER_CHECK:
        if(check_index < 5)
        {
          check_index++;
        }
        else
        {
          if(battery_valtage_raw < voltage_ref_warn)
          {
            setPowerOff();
            prev_state    = battery_state;
            battery_state = BATTERY_POWER_WARNNING;
          }
        }
        if(battery_valtage_raw >= voltage_ref)
        {
          setPowerOn();
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_NORMAL;
        }
        break;

      case BATTERY_POWER_WARNNING:
        alram_state ^= 1;
        if(alram_state)
        {
          tone(BDPIN_BUZZER, 1000, 500);
        }

        if(battery_valtage_raw > voltage_ref)
        {
          setPowerOn();
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_NORMAL;
        }
        else
        {
          setPowerOff();
        }

        if(battery_valtage_raw < 5.0)
        {
          setPowerOff();
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_OFF;
        }
        break;

      default:
        break;
    }
  }
}

/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(void)
{
  static bool gyro_cali = false;
  uint32_t pre_time;
  uint32_t t_time;

  char log_msg[50];

  if (nh.connected())
  {
    if (gyro_cali == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);

      imu.SEN.gyro_cali_start();

      t_time   = millis();
      pre_time = millis();
      while(!imu.SEN.gyro_cali_get_done())
      {
        imu.update();

        if (millis()-pre_time > 5000)
        {
          break;
        }
        if (millis()-t_time > 100)
        {
          t_time = millis();
          setLedToggle(LED_ROS_CONNECT);
        }
      }
      gyro_cali = true;

      sprintf(log_msg, "Calibration End");
      nh.loginfo(log_msg);
    }
  }
  else
  {
    gyro_cali = false;
  }
}

/*******************************************************************************
* Show LED status
*******************************************************************************/
void showLedStatus(void)
{
  static uint32_t t_time = millis();

  if ((millis()-t_time) >= 500)
  {
    t_time = millis();
    digitalWrite(13, !digitalRead(13));
  }

  if (getPowerInVoltage() < 11.1)
  {
    setLedOn(LED_LOW_BATTERY);
  }
  else
  {
    setLedOff(LED_LOW_BATTERY);
  }

  if (nh.connected())
  {
    setLedOn(LED_ROS_CONNECT);
  }
  else
  {
    setLedOff(LED_ROS_CONNECT);
  }

  updateRxTxLed();
}

void updateRxTxLed(void)
{
  static uint32_t rx_led_update_time;
  static uint32_t tx_led_update_time;
  static uint32_t rx_cnt;
  static uint32_t tx_cnt;

  if ((millis()-tx_led_update_time) > 50)
  {
    tx_led_update_time = millis();

    if (tx_cnt != Serial.getTxCnt())
    {
      setLedToggle(LED_TXD);
    }
    else
    {
      setLedOff(LED_TXD);
    }

    tx_cnt = Serial.getTxCnt();
  }

  if ((millis()-rx_led_update_time) > 50)
  {
    rx_led_update_time = millis();

    if (rx_cnt != Serial.getRxCnt())
    {
      setLedToggle(LED_RXD);
    }
    else
    {
      setLedOff(LED_RXD);
    }

    rx_cnt = Serial.getRxCnt();
  }
}

void setPowerOn()
{
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
}

void setPowerOff()
{
  digitalWrite(BDPIN_DXL_PWR_EN, LOW);
}

