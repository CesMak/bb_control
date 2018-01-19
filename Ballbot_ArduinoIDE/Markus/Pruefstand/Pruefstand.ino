#include <DynamixelSDK.h>


// Control table address (XM430-W210-R)
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_GOAL_CURRENT           102
#define ADDR_PRO_PRESENT_CURRENT        126
#define ADDR_PRO_OPERATING_MODE         11

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define MOTOR_ID_                       2
#define BAUDRATE                        4500000
#define DEVICENAME                      ""      // Check which port is being used on your controller // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      100                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL__POS_THRESHOLD              10                  // Dynamixel moving status threshold
#define DXL_MINIMUM_CURRENT_VALUE       0
#define DXL_MAXIMUM_CURRENT_VALUE       1000
#define CURRENT_CONTROL                 0


#define ESC_ASCII_VALUE                 0x1b



void setup() {
  Serial.begin(115200);

  delay(3000);

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int dxl_comm_result = COMM_TX_FAIL;                // Communication result

  // Open port
  if (portHandler->openPort())
  {
    Serial.print("Succeeded to open the port!\n");
  }
  else
  {
    Serial.print("Failed to open the port!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    Serial.print("Succeeded to change the baudrate!\n");
  }
  else
  {
    Serial.print("Failed to change the baudrate!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

  // Setting Dynamixels Operation Mode TO CURRENT CONTROL
  uint8_t dxl_error_ = 0;                          // Dynamixel error
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, MOTOR_ID_, ADDR_PRO_OPERATING_MODE, CURRENT_CONTROL, &dxl_error_);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error_ != 0)
  {
    packetHandler->printRxPacketError(dxl_error_);
  }
  else
  {
    Serial.print("Dynamixel Motor operation mode has succesfully been set to --torque controlling mode-- \n");
  }

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, MOTOR_ID_, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error_);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error_ != 0)
  {
    packetHandler->printRxPacketError(dxl_error_);
  }
  else
  {
    Serial.print("Dynamixel Motor Torque has been successfully enabled \n");
  }

  Serial.print("Starting the experiment:");

  int startgoalCurrent = 50; //Units
  write_goalCurrent(packetHandler, portHandler, 10);
  while(1)
  {
  read_goalCurrent(packetHandler, portHandler);
  }
//  for (int i = startgoalCurrent; i < 400; i = i + 25)
//  {
//    write_goalCurrent(packetHandler, portHandler, i);
//
//    int tTime = 6000+millis();
//
//    int current_time = millis();
//    
//    while (tTime - current_time > 0 ) {
//      current_time=millis();
//      Serial.print("time left: "); Serial.println(tTime - current_time);
//      read_goalCurrent(packetHandler, portHandler);
//    }
//
//    blink_LED(packetHandler, portHandler);
//  }


  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, MOTOR_ID_, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error_);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error_ != 0)
  {
    packetHandler->printRxPacketError(dxl_error_);
  }

  // Close port
  portHandler->closePort();
}

void loop()
{

}


void write_goalCurrent(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler, uint16_t current_goal_units)
{
  // Write goal Current:
  uint8_t error = 0;
  int dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_ID_, ADDR_PRO_GOAL_CURRENT, current_goal_units, &error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (error != 0)
  {
    packetHandler->printRxPacketError(error);
  }
}

int read_goalCurrent(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler)
{
  // Read present current
  uint8_t read_error = 0;
  int output = 0;
  int dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, MOTOR_ID_, ADDR_PRO_PRESENT_CURRENT, (uint16_t*)&output, &read_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (read_error != 0)
  {
    packetHandler->printRxPacketError(read_error);
  }
  Serial.print("current_units: "); Serial.println(output);
  return output;
}

void blink_LED(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler)
{
  uint8_t led_error = 0;
  packetHandler->write1ByteTxRx(portHandler, MOTOR_ID_, 65, 1, &led_error);
  delay(1000);
  packetHandler->write1ByteTxRx(portHandler, MOTOR_ID_, 65, 0, &led_error);
}

