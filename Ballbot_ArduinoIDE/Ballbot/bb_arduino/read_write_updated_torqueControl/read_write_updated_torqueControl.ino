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
#define DXL_ID                          3                   // Dynamixel ID: 1
#define BAUDRATE                        3000000
#define DEVICENAME                      "COM4"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      100                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     1                   // Dynamixel moving status threshold
#define DXL_MINIMUM_CURRENT_VALUE       0
#define DXL_MAXIMUM_CURRENT_VALUE       1000
#define CURRENT_CONTROL                 0


#define ESC_ASCII_VALUE                 0x1b



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);


  Serial.println("Start..");


}

void loop() {
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  //int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  //int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position

  uint8_t dxl_error = 0;                          // Dynamixel error
  int32_t dxl_present_position = 0;               // Present position
  uint32_t dxl_present_current = 0;

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

  // Setting Dynamixel Operation Mode TO CURRENT CONTROL
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_OPERATING_MODE, CURRENT_CONTROL, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->printRxPacketError(dxl_error);
  }
  else
  {
    Serial.print("Dynamixel operation mode has succesfully set to --torque controlling mode-- \n");
  }

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->printRxPacketError(dxl_error);
  }
  else
  {
    Serial.print("Dynamixel has been successfully connected \n");
  }



  Serial.print("Press any key to continue! (or press q to quit!)\n");


  while(Serial.available()==0);

  int ch;

  ch = Serial.read();
  if (ch == 'q'){
    Serial.print("You quit");
  }

  else{

      //First run
      int goalCurrent = ;
      //
      //
      //
       // Write goal current
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_CURRENT, goalCurrent, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->printTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->printRxPacketError(dxl_error);
        }
    
        do
        {
          // Read present current
          dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_CURRENT, (uint16_t*)&dxl_present_current, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->printTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->printRxPacketError(dxl_error);
          }
    
//          Serial.print("[ID:");      Serial.print(DXL_ID);
//          Serial.print(" GoalCurrent:"); Serial.print(goalCurrent);
//          Serial.print(" PresCurrent:");  Serial.print(dxl_present_current);
//          Serial.println(" ");
//          Serial.println("DXL_MOVING_STATUS_THRESHOLD"); Serial.println(DXL_MOVING_STATUS_THRESHOLD);
//          Serial.println(" ");
//          Serial.print("Working Status:"); Serial.print((abs(goalCurrent - dxl_present_current) > DXL_MOVING_STATUS_THRESHOLD));
//          Serial.println(" ");
            Serial.print(dxl_present_current);
    
    
        }while(1);//while((abs(goalCurrent - dxl_present_current) > DXL_MOVING_STATUS_THRESHOLD));

        //Second run
      goalCurrent = 100;
      //
      //
      //
       // Write goal current
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_CURRENT, goalCurrent, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->printTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->printRxPacketError(dxl_error);
        }
    
        do
        {
          // Read present current
          dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_CURRENT, (uint16_t*)&dxl_present_current, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->printTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->printRxPacketError(dxl_error);
          }
    
          Serial.print("[ID:");      Serial.print(DXL_ID);
          Serial.print(" GoalCurrent:"); Serial.print(goalCurrent);
          Serial.print(" PresCurrent:");  Serial.print(dxl_present_current);
          Serial.println(" ");
          Serial.println("DXL_MOVING_STATUS_THRESHOLD"); Serial.println(DXL_MOVING_STATUS_THRESHOLD);
          Serial.println(" ");
          Serial.print("Working Status:"); Serial.print((abs(goalCurrent - dxl_present_current) > DXL_MOVING_STATUS_THRESHOLD));
          Serial.println(" ");
    
    
        }while((abs(goalCurrent - dxl_present_current) > DXL_MOVING_STATUS_THRESHOLD));

    
  
    // Disable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->printRxPacketError(dxl_error);
    }
  
    // Close port
    portHandler->closePort();
  }

}
