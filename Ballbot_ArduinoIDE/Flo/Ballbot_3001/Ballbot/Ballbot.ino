#include <DynamixelSDK.h>
#include <IMU.h>

#define ADDR_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_OPERATING_MODE         11
#define ADDR_GOAL_TORQUE            102
#define ADDR_PRESENT_CURRENT        126

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        4500000
#define DEVICENAME                      "COM3"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0
#define CURRENT_MODE                    0       


#define SAMPLE_TIME                     7000
#define GRES                            2000.0/32768.0

#define K_X                             -4.2609
#define K_X_DOT                         -1.0295
#define K_Y                             -4.2584
#define K_Y_DOT                         -1.0268

#define K1                              125
#define K2                              132
#define K3                              207


#define ALPHA                           PI/4
#define BETA                            0
#define COS_ALPHA                       cos(ALPHA)       // in rad.
#define SIN_ALPHA                       sin(ALPHA)
#define SIN_BETA                        sin(BETA)
#define COS_BETA                        cos(BETA)
#define SQRT3                           1.73205080757

#define FILTER_ON
//#define FILTER_OFF
//#define REAL_TORQUES
//#define UNITS_COMPUTED
//#define PRESENT_UNITS




cIMU imu;
HardwareTimer Timer(TIMER_CH1);
dynamixel::PacketHandler *pHandler;
dynamixel::PortHandler *poHandler;



void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    ; 
  }
  
  Serial.println("Init started");

  imu.begin();
  motors_init(); 
  
  delay(3000);
  
  Timer.stop();
  Timer.setPeriod(SAMPLE_TIME);
  Timer.attachInterrupt(executeController);
  Timer.start();

  Serial.println("Init finished");

}

void loop()
{


}

void executeController(void)
{
    imu.update(); 
    readIMU();
}



void motors_init()
{
     dynamixel::PortHandler *portHandler =  dynamixel::PortHandler::getPortHandler(DEVICENAME);
     dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

     pHandler = packetHandler;
     poHandler = portHandler;

    int index = 0;
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
   

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
      return;
    }

    // Disable Dynamixel Torque
    for(int i=1; i<=3; i++)
    {
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->printTxRxResult(dxl_comm_result);
        return;
      }
      else if (dxl_error != 0)
      {
        packetHandler->printRxPacketError(dxl_error);
        return;
      }
    }

    // Set Current Mode
    for(int i=1; i<=3; i++)
    {
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_OPERATING_MODE, CURRENT_MODE, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->printTxRxResult(dxl_comm_result);
        return;
      }
      else if (dxl_error != 0)
      {
        packetHandler->printRxPacketError(dxl_error);
        return;
      }
    }


    //Enable Dynamixel Torque
    for(int i=1; i<=3; i++)
    {
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->printTxRxResult(dxl_comm_result);
        return;
      }
      else if (dxl_error != 0)
      {
        packetHandler->printRxPacketError(dxl_error);
        return;
      }
    }


    Serial.print("Dynamixel has been successfully connected \n");
}

void readIMU()
{

  static float* theta_x = new float[3];
  static float* theta_y = new float[3];
  static float* theta_x_dot = new float[3];
  static float* theta_y_dot = new float[3];

  float filter_theta_x = 0; 
  float filter_theta_y = 0; 
  float filter_theta_x_dot = 0; 
  float filter_theta_y_dot = 0; 

  //Aktuellen Wert einlesen
  theta_x[0] = convert2radiand(imu.rpy[1])+0.04;
  theta_y[0] = convert2radiand(imu.rpy[0])-0.02;
  theta_x_dot[0] = convert2radiand(imu.gyroData[1]*GRES);
  theta_y_dot[0] = convert2radiand(imu.gyroData[0]*GRES);

  #ifdef FILTER_OFF
  Serial.print(theta_x[0]); Serial.print("\t");Serial.print(theta_y[0]); Serial.print("\t");Serial.print(theta_x_dot[0]); Serial.print("\t");Serial.print(theta_y_dot[0]); Serial.print("\n");
  #endif

  for(int i=0; i<3; i++)
  {
    filter_theta_x += theta_x[i];
    filter_theta_y += theta_y[i];
    filter_theta_x_dot += theta_x_dot[i];
    filter_theta_y_dot += theta_y_dot[i];
  }


  filter_theta_x = filter_theta_x/3;
  filter_theta_y = filter_theta_y/3;
  filter_theta_x_dot = filter_theta_x_dot/3;
  filter_theta_y_dot = filter_theta_y_dot/3;

  //Shiften der Werte

  theta_x[2] = theta_x[1];
  theta_x[1] = theta_x[0];

  theta_y[2] = theta_y[1];
  theta_y[1] = theta_y[0];

  theta_x_dot[2] = theta_x_dot[1];
  theta_x_dot[1] = theta_x_dot[0];

  theta_y_dot[2] = theta_y_dot[1];
  theta_y_dot[1] = theta_y_dot[0];

  controller(filter_theta_x,filter_theta_y,filter_theta_x_dot,filter_theta_y_dot);

  #ifdef FILTER_ON
  Serial.print(filter_theta_x); Serial.print("\t");Serial.print(filter_theta_y); Serial.print("\t");Serial.print(filter_theta_x_dot); Serial.print("\t");  Serial.print(filter_theta_y_dot); Serial.print("\n");
  #endif
  
}


void controller(float theta_x, float theta_y, float theta_x_dot, float theta_y_dot)
{

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  uint8_t dxl_error = 0;                          // Dynamixel error

  //Compute Tx,Ty
  static float* virtual_torques = new float[3];
  static float* real_torques = new float[3];  

  // Computation virtual Torques
  // Torque in the yz Planar --> T_x
  virtual_torques[0] = (theta_x * K_X + theta_x_dot * K_X_DOT )*-1;

  // Torque in the xz Planar --> T_y
  virtual_torques[1] = (theta_y * K_Y + theta_y_dot * K_Y_DOT )*-1;

  virtual_torques[2] = 0.0;

  
  //Computation real Torques
  // Torque T1
  real_torques[0] = 0.333333333 * (virtual_torques[2] + (2 / COS_ALPHA) * (virtual_torques[0] * COS_BETA - virtual_torques[1] * SIN_BETA));

  //Torque T2
  real_torques[1] = 0.333333333 * (virtual_torques[2] + (1 / COS_ALPHA) * (SIN_BETA * (-virtual_torques[0] * SQRT3 + virtual_torques[1]) - COS_BETA * (virtual_torques[0] + SQRT3 * virtual_torques[1])));

  //
  real_torques[2] = 0.333333333 * (virtual_torques[2] + (1 / COS_ALPHA) * (SIN_BETA * (virtual_torques[0] * SQRT3 + virtual_torques[1]) + COS_BETA * (-virtual_torques[0] + SQRT3 * virtual_torques[1])));

  #ifdef REAL_TORQUES
    Serial.print(real_torques[0]); Serial.print("\t");Serial.print(real_torques[1]); Serial.print("\t");Serial.print(real_torques[2]); Serial.print("\n");
  #endif


  int16_t* current_units = compute2currentunits(real_torques);

  #ifdef UNITS_COMPUTED
  Serial.print(current_units[0]); Serial.print("\t");Serial.print(current_units[1]); Serial.print("\t");Serial.print(current_units[2]); Serial.print("\n");
  #endif

  //Read Present Units
  uint32_t temp_unit = 0; 
  uint32_t* present_units_arr =  new uint32_t[3]; 

  for(int i=1; i<3; i++)
  {
      dxl_comm_result = pHandler->read4ByteTxRx(poHandler, i, ADDR_PRESENT_CURRENT, (uint32_t*)& temp_unit, &dxl_error);
      
      if (dxl_comm_result != COMM_SUCCESS)
      {
        pHandler->printTxRxResult(dxl_comm_result);
      }
      else if (dxl_error != 0)
      {
        pHandler->printRxPacketError(dxl_error);
      }

      present_units_arr[i-1]=temp_unit;
  }

  #ifdef PRESENT_UNITS
  Serial.print(present_units[0]); Serial.print("\t");Serial.print(present_units[1]); Serial.print("\t");Serial.print(present_units[2]); Serial.print("\n");
  #endif
 
  for(int i=1; i<=3; i++)
  {
      dxl_comm_result = pHandler->write2ByteTxRx(poHandler, i, ADDR_GOAL_TORQUE, current_units[i-1], &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        pHandler->printTxRxResult(dxl_comm_result);
        return;
      }
      else if (dxl_error != 0)
      {
        pHandler->printRxPacketError(dxl_error);
        return;
      }
    
  }
}


int16_t *compute2currentunits(float real_torques_arr[]) 
{

  static int16_t* ret_arr = new int16_t[3];

  ret_arr[0] = (int16_t)(K1 * real_torques_arr[0]);
  ret_arr[1] = (int16_t)(K2 * real_torques_arr[1]);
  ret_arr[2] = (int16_t)(K3 * real_torques_arr[2]);

  return ret_arr;
}


float convert2radiand(float val_deg)
{
  return (val_deg * PI) / 180;
}
