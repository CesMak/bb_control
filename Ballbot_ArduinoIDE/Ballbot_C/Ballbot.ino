#include "controller.h"
 

cIMU    IMU;
HardwareTimer Timer(TIMER_CH1);


void setup()
{
  Serial.begin(115200);
  while(!Serial);

  Serial.println("Starting..."); 
 
  // Initialize PortHandler instance, set port path, get methods and members 
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
   // Initialize PacketHandler instance, set the protocol version, get methods and members of Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  
  if(!portHandler->openPort()){
    Serial.println("No Port can be opened, the Robot can not be started");
    return;
  }
  
  if(!portHandler->setBaudRate(BAUDRATE)){
    Serial.println("The Baudrate can not be changed, the Robot can not be startet");
    return;
  }
  
  Serial.println("Port and Baudrate has been correctly initialized");
  // Enable Dynamixel Torque
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  uint8_t dxl_error = 0;                          // Dynamixel error

  for (int deviceID=1; deviceID<=3; deviceID++){
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, deviceID, ADDR_LED, LED_ENABLE, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, deviceID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    
    if (dxl_comm_result != COMM_SUCCESS){    
      packetHandler->printTxRxResult(dxl_comm_result);
      return;
    }
    
    else if (dxl_error != 0){
      packetHandler->printRxPacketError(dxl_error);
      return;
    }
  }
  Serial.println("Motors has been successfully connected");
  
  
  
  // Initialization of the IMU
  if(IMU.begin()==0){
    Serial.println("IMU started.");
  }
  else {
    Serial.println("Start of IMU failed");
  }
  
  Serial.println("Please press any key to start the Robot");
  while(Serial.available()==0);
  Timer.stop();
  Timer.setPeriod(100000);
  Timer.attachInterrupt(executeController);
  Timer.start();

}

void loop()
{

  IMU.update();
  
}

void executeController()
{
    readIMU(IMU,&packetHandler);
    
}


