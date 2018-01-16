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
#define DXL_ID_M3                       3                   // Dynamixel ID: 3
#define DXL_ID_M1                       1                   // Dynamixel ID: 1
#define BAUDRATE                        3000000
#define DEVICENAME                      "COM3"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      100                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL__POS_THRESHOLD              10                  // Dynamixel moving status threshold
#define DXL_MINIMUM_CURRENT_VALUE       0
#define DXL_MAXIMUM_CURRENT_VALUE       1000
#define CURRENT_CONTROL                 0
#define POSITION_CONTROL                3


#define ESC_ASCII_VALUE                 0x1b



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
//  while(!Serial);


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

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  
  uint8_t dxl_error_M1 = 0;                          // Dynamixel error
  uint8_t dxl_error_M3 = 0;                          // Dynamixel error
  int32_t dxl_present_position_M1 = 0;               // Present position
  uint16_t dxl_present_current_M1 = 0;
  int32_t dxl_present_position_M3 = 0;               // Present position
  uint16_t dxl_present_current_M3 = 0;

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

  
  //
  //
  //
  //
  //
  //
  
  //StartProcedure
  Serial.print("Press any key to continue the start procedure! (or press q to quit!)\n");

//  while(Serial.available()==0);
//
//  int ch;
//
//  ch = Serial.read();
//  if (ch == 'q'){
//    Serial.print("You quit");
//  }
//
//  else{

      //
      //
      //
      // Setting Dynamixels Operation Mode TO POSITION CONTROL
      //M1
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_M1, ADDR_PRO_OPERATING_MODE, POSITION_CONTROL, &dxl_error_M1);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->printTxRxResult(dxl_comm_result);
      }
      else if (dxl_error_M1 != 0)
      {
        packetHandler->printRxPacketError(dxl_error_M1);
      }
      else
      {
        Serial.print("Dynamixel M1 operation mode has succesfully set to --position controlling mode-- \n");
      }
      // M3
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_M3, ADDR_PRO_OPERATING_MODE, POSITION_CONTROL, &dxl_error_M3);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->printTxRxResult(dxl_comm_result);
      }
      else if (dxl_error_M3 != 0)
      {
        packetHandler->printRxPacketError(dxl_error_M3);
      }
      else
      {
        Serial.print("Dynamixel M3 operation mode has succesfully set to --position controlling mode-- \n");
      }
    
    
    
    
      // Enable Dynamixel Torque
      //M1
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_M1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error_M1);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->printTxRxResult(dxl_comm_result);
      }
      else if (dxl_error_M1 != 0)
      {
        packetHandler->printRxPacketError(dxl_error_M1);
      }
      else
      {
        Serial.print("Dynamixel M1 has been successfully connected \n");
      }
      // M3
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_M3, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error_M3);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->printTxRxResult(dxl_comm_result);
      }
      else if (dxl_error_M3 != 0)
      {
        packetHandler->printRxPacketError(dxl_error_M3);
      }
      else
      {
        Serial.print("Dynamixel M3 has been successfully connected \n");
      }


      //First run
      int goalPosition_Start = 100;
      //
      //
      //
       // Write goal position M1
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID_M1, ADDR_PRO_GOAL_POSITION, goalPosition_Start, &dxl_error_M1);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->printTxRxResult(dxl_comm_result);
        }
        else if (dxl_error_M1 != 0)
        {
          packetHandler->printRxPacketError(dxl_error_M1);
        }

        // Write goal position M3
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID_M3, ADDR_PRO_GOAL_POSITION, goalPosition_Start, &dxl_error_M3);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->printTxRxResult(dxl_comm_result);
        }
        else if (dxl_error_M3 != 0)
        {
          packetHandler->printRxPacketError(dxl_error_M3);
        }
    
        do
        {
          // Read present position M1
          dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID_M1, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position_M1, &dxl_error_M1);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->printTxRxResult(dxl_comm_result);
          }
          else if (dxl_error_M1 != 0)
          {
            packetHandler->printRxPacketError(dxl_error_M1);
          }

          // Read present position M3
          dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID_M3, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position_M3, &dxl_error_M3);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->printTxRxResult(dxl_comm_result);
          }
          else if (dxl_error_M3 != 0)
          {
            packetHandler->printRxPacketError(dxl_error_M3);
          }
          Serial.println(" ");
          Serial.println(" ");
          Serial.print("[ID:");      Serial.print(DXL_ID_M1);
          Serial.print(" GoalPositionM1:"); Serial.print(goalPosition_Start);
          Serial.print(" PresentPositionM1:");  Serial.print(dxl_present_position_M1);
          Serial.println(" ");
          Serial.println(" ");
          Serial.print("[ID:");      Serial.print(DXL_ID_M3);
          Serial.print(" GoalPositionM3:"); Serial.print(goalPosition_Start);
          Serial.print(" PressentPositionM3:");  Serial.print(dxl_present_position_M3);
          Serial.println(" ");
          Serial.println(" ");
          Serial.println("-------------------------------------------");
             
        }while((abs(goalPosition_Start - dxl_present_position_M1) > DXL__POS_THRESHOLD)||(abs(goalPosition_Start - dxl_present_position_M3) > DXL__POS_THRESHOLD));

    
  
    // Disable Dynamixel Torque
    //M1
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_M1, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error_M1);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error_M1 != 0)
    {
      packetHandler->printRxPacketError(dxl_error_M1);
    }
    // M3
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_M3, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error_M3);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error_M3 != 0)
    {
      packetHandler->printRxPacketError(dxl_error_M3);
    }
  
//  }


    delay(5000);


  //
  //
  //
  //
  //
  //
  //
  //Moving with defined torques
  
//  Serial.print("Press any key to continue the torque procedure! (or press q to quit!)\n");
//
//  int c;
//  
//  c = Serial.read();
//  if (c == 'q'){
//    Serial.print("You quit");
//  }
//
//  else{

      // Setting Dynamixels Operation Mode TO CURRENT CONTROL
      //M1
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_M1, ADDR_PRO_OPERATING_MODE, CURRENT_CONTROL, &dxl_error_M1);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->printTxRxResult(dxl_comm_result);
      }
      else if (dxl_error_M1 != 0)
      {
        packetHandler->printRxPacketError(dxl_error_M1);
      }
      else
      {
        Serial.print("Dynamixel M1 operation mode has succesfully set to --torque controlling mode-- \n");
      }
      // M3
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_M3, ADDR_PRO_OPERATING_MODE, CURRENT_CONTROL, &dxl_error_M3);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->printTxRxResult(dxl_comm_result);
      }
      else if (dxl_error_M3 != 0)
      {
        packetHandler->printRxPacketError(dxl_error_M3);
      }
      else
      {
        Serial.print("Dynamixel M3 operation mode has succesfully set to --torque controlling mode-- \n");
      }
       
    
      // Enable Dynamixel Torque
      //M1
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_M1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error_M1);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->printTxRxResult(dxl_comm_result);
      }
      else if (dxl_error_M1 != 0)
      {
        packetHandler->printRxPacketError(dxl_error_M1);
      }
      else
      {
        Serial.print("Dynamixel M1 has been successfully connected \n");
      }
      // M3
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_M3, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error_M3);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->printTxRxResult(dxl_comm_result);
      }
      else if (dxl_error_M3 != 0)
      {
        packetHandler->printRxPacketError(dxl_error_M3);
      }
      else
      {
        Serial.print("Dynamixel M3 has been successfully connected \n");
      }

      //First run
      int goalCurrent = 20;
      int goalPosition = 60000;
      //
      //
      //
       // Write goal current M1
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID_M1, ADDR_PRO_GOAL_CURRENT, -goalCurrent, &dxl_error_M1);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->printTxRxResult(dxl_comm_result);
        }
        else if (dxl_error_M1 != 0)
        {
          packetHandler->printRxPacketError(dxl_error_M1);
        }

        // Write goal current M3
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID_M3, ADDR_PRO_GOAL_CURRENT, goalCurrent, &dxl_error_M3);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->printTxRxResult(dxl_comm_result);
        }
        else if (dxl_error_M3 != 0)
        {
          packetHandler->printRxPacketError(dxl_error_M3);
        }
    
        do
        {
          // Read present current M1
          dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID_M1, ADDR_PRO_PRESENT_CURRENT, (uint16_t*)&dxl_present_current_M1, &dxl_error_M1);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->printTxRxResult(dxl_comm_result);
          }
          else if (dxl_error_M1 != 0)
          {
            packetHandler->printRxPacketError(dxl_error_M1);
          }
          // Read present position M1
          dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID_M1, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position_M1, &dxl_error_M1);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->printTxRxResult(dxl_comm_result);
          }
          else if (dxl_error_M1 != 0)
          {
            packetHandler->printRxPacketError(dxl_error_M1);
          }

          // Read present current M3
          dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID_M3, ADDR_PRO_PRESENT_CURRENT, (uint16_t*)&dxl_present_current_M3, &dxl_error_M3);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->printTxRxResult(dxl_comm_result);
          }
          else if (dxl_error_M3 != 0)
          {
            packetHandler->printRxPacketError(dxl_error_M3);
          }
          // Read present position M3
          dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID_M3, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position_M3, &dxl_error_M3);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->printTxRxResult(dxl_comm_result);
          }
          else if (dxl_error_M3 != 0)
          {
            packetHandler->printRxPacketError(dxl_error_M3);
          }
          Serial.println(" ");
          Serial.println(" ");
          Serial.print("[ID:");      Serial.print(DXL_ID_M1);
          Serial.print(" GoalPositionM1:"); Serial.print(goalPosition);
          Serial.print(" PresentPositionM1:");  Serial.print(dxl_present_position_M1);
          Serial.print(" PresentCurrentM1:");  Serial.print(dxl_present_current_M1);
          Serial.println(" ");
          Serial.println(" ");
          Serial.print("[ID:");      Serial.print(DXL_ID_M3);
          Serial.print(" GoalPositionM3:"); Serial.print(goalPosition);
          Serial.print(" PressentPositionM3:");  Serial.print(dxl_present_position_M3);
          Serial.print(" PresentCurrentM3:");  Serial.print(dxl_present_current_M3);
          Serial.println(" ");
          Serial.println(" ");
          Serial.println("-------------------------------------------");
             
        }while((abs(goalPosition - dxl_present_position_M1) > DXL__POS_THRESHOLD)&&(abs(goalPosition - dxl_present_position_M3) > DXL__POS_THRESHOLD));

    
  
    // Disable Dynamixel Torque
    //M1
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_M1, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error_M1);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error_M1 != 0)
    {
      packetHandler->printRxPacketError(dxl_error_M1);
    }
    // M3
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_M3, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error_M3);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error_M3 != 0)
    {
      packetHandler->printRxPacketError(dxl_error_M3);
    }
  
    // Close port
    portHandler->closePort();
//  }

}
