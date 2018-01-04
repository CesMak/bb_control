#include <DynamixelSDK.h>


// Control table address (XM430-W210-R)
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_OPERATING_MODE         11
#define ADDR_PRO_GOAL_VELOCITY          104
#define ADDR_PRO_PRESENT_VELOCITY       128
#define ADDR_PRO_PRESENT_CURRENT        126

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
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold
#define OPERATING_MODE                  1

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
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_OPERATING_MODE, OPERATING_MODE, &dxl_error);
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
    Serial.print("Dynamixel operation mode has succesfully set to --position controlling mode-- \n");
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

    for (int i=0; i <= 2; i++){

      //First run
      int goalPos = 0;
      goalPos = 100;
      //
      //
      //
      // put your main code here, to run repeatedly:
       // Write goal position
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_VELOCITY, goalPos, &dxl_error);
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
          // Read present position
          dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_CURRENT, (uint16_t*)&dxl_present_position, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->printTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->printRxPacketError(dxl_error);
          }
    
          Serial.print("[ID:");      Serial.print(DXL_ID);
          Serial.print(" GoalPos:"); Serial.print(goalPos);
          Serial.print(" PresCurr:");  Serial.print(dxl_present_position);
          Serial.println(" ");
    
    
        }while(1);//while((abs(goalPos - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));
    
      
    
    
     //Second run
     goalPos = 10;
      //
      //
      //
      // put your main code here, to run repeatedly:
       // Write goal position
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_VELOCITY, goalPos, &dxl_error);
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
          // Read present position
          dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_CURRENT, (uint16_t*)&dxl_present_position, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->printTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->printRxPacketError(dxl_error);
          }
    
          Serial.print("[ID:");      Serial.print(DXL_ID);
          Serial.print(" GoalPos:"); Serial.print(goalPos);
          Serial.print(" PresCurr:");  Serial.print(dxl_present_position);
          Serial.println(" ");
    
    
        }while(1);//while((abs(goalPos - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));  
    }
    
  
  
  
    
  
  
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
