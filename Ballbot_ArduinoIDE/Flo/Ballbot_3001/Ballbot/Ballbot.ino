#include <DynamixelSDK.h>
#include <IMU.h>

#define ADDR_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_OPERATING_MODE         11
#define ADDR_GOAL_TORQUE            102
#define ADDR_PRESENT_CURRENT        126
#define ADDR_VELOCITY_LIMIT         44
#define ADDR_GOAL_POSITION          116
#define ADDR_PRESENT_POSITION       132
#define ADDR_PRESENT_VELOCITY       128


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
#define VELOCITY_LIMIT                  1023
#define POSITION_MODE                   3
#define DXL__POS_THRESHOLD              0

#define SAMPLE_TIME                     10
#define GRES                            2000.0/32768.0

#define K_X                             -8.6953 
#define K_X_DOT                         -2.0932
#define K_X_PHI_DOT                     -0.1166 
#define K_Y                             -8.6953 
#define K_Y_DOT                         -2.0932
#define K_Y_PHI_DOT                     -0.1166


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

//#define FILTER_ON
//#define FILTER_OFF
//#define REAL_TORQUES
//#define UNITS_COMPUTED
//#define PRESENT_UNITS
//#define VELOCITY
#define GOAL_POS
#define PHI_DOT



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

//    // Set POSITION MODE
//    for(int i=1; i<=3; i++)
//    {
//      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_OPERATING_MODE, POSITION_MODE, &dxl_error);
//      if (dxl_comm_result != COMM_SUCCESS)
//      {
//        packetHandler->printTxRxResult(dxl_comm_result);
//        return;
//      }
//      else if (dxl_error != 0)
//      {
//        packetHandler->printRxPacketError(dxl_error);
//        return;
//      }
//    }
//
//    //Enable Dynamixel Torque
//    for(int i=1; i<=3; i++)
//    {
//      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
//      if (dxl_comm_result != COMM_SUCCESS)
//      {
//        packetHandler->printTxRxResult(dxl_comm_result);
//        return;
//      }
//      else if (dxl_error != 0)
//      {
//        packetHandler->printRxPacketError(dxl_error);
//        return;
//      }
//    }
//
//
//    //ReferenzFahrt
//    int goalPosition_Start = 0;
//    uint32_t dxl_present_position_M1 = 0; 
//    uint32_t dxl_present_position_M2 = 0;
//    uint32_t dxl_present_position_M3 = 0;
//
//    uint8_t dxl_error_M1 = 0;
//    uint8_t dxl_error_M2 = 0; 
//    uint8_t dxl_error_M3 = 0;  
//    
//    for(int i=1; i<=3; i++)
//    {
//      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDR_GOAL_POSITION, goalPosition_Start, &dxl_error);
//
//        if (dxl_comm_result != COMM_SUCCESS)
//
//        {
//
//          packetHandler->printTxRxResult(dxl_comm_result);
//
//        }
//
//        else if (dxl_error_M1 != 0)
//
//        {
//
//          packetHandler->printRxPacketError(dxl_error);
//
//        }
//    }
//
//     do
//
//        {
//
//          // Read present position M1
//
//          dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, 1, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position_M1, &dxl_error_M1);
//
//          if (dxl_comm_result != COMM_SUCCESS)
//
//          {
//
//            packetHandler->printTxRxResult(dxl_comm_result);
//
//          }
//
//          else if (dxl_error_M1 != 0)
//
//          {
//
//            packetHandler->printRxPacketError(dxl_error_M1);
//
//          }
//
//          //Rad present position M2
//
//          dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, 2, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position_M2, &dxl_error_M2);
//
//          if (dxl_comm_result != COMM_SUCCESS)
//
//          {
//
//            packetHandler->printTxRxResult(dxl_comm_result);
//
//          }
//
//          else if (dxl_error_M1 != 0)
//
//          {
//
//            packetHandler->printRxPacketError(dxl_error_M2);
//
//          }
//
// 
//
//          // Read present position M3
//
//          dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, 3, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position_M3, &dxl_error_M3);
//
//          if (dxl_comm_result != COMM_SUCCESS)
//
//          {
//
//            packetHandler->printTxRxResult(dxl_comm_result);
//
//          }
//
//          else if (dxl_error_M3 != 0)
//
//          {
//
//            packetHandler->printRxPacketError(dxl_error_M3);
//
//          }
//
//          
//        }while((abs(goalPosition_Start - dxl_present_position_M1) > DXL__POS_THRESHOLD) || (abs(goalPosition_Start - dxl_present_position_M2) > DXL__POS_THRESHOLD) || (abs(goalPosition_Start - dxl_present_position_M3) > DXL__POS_THRESHOLD));
//
//  
//    
//    #ifdef GOAL_POS
//    Serial.print(dxl_present_position_M1); Serial.print("\t");Serial.print(dxl_present_position_M2); Serial.print("\t");Serial.print(dxl_present_position_M3); Serial.print("\n");
//    #endif
//
//
//
//    // Disable Dynamixel Torque
//    for(int i=1; i<=3; i++)
//    {
//      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
//      if (dxl_comm_result != COMM_SUCCESS)
//      {
//        packetHandler->printTxRxResult(dxl_comm_result);
//        return;
//      }
//      else if (dxl_error != 0)
//      {
//        packetHandler->printRxPacketError(dxl_error);
//        return;
//      }
//    }

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

     //Velocity

    int limit = 1023; 
    for(int i=1; i<=3; i++)
    {
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDR_VELOCITY_LIMIT, limit, &dxl_error);
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

  
    //Ausgabe Velocity Limit

    uint32_t temp_limit = 0; 
    uint32_t* velocity_limit =  new uint32_t[3]; 
     
     for(int i=1; i<=3; i++)
    {
      dxl_comm_result = pHandler->read4ByteTxRx(poHandler, i, ADDR_VELOCITY_LIMIT, (uint32_t*)&temp_limit, &dxl_error);
      
      if (dxl_comm_result != COMM_SUCCESS)
      {
        pHandler->printTxRxResult(dxl_comm_result);
      }
      else if (dxl_error != 0)
      {
        pHandler->printRxPacketError(dxl_error);
      }

      velocity_limit[i-1] = temp_limit;
      temp_limit=0;
    }

    
    #ifdef VELOCITY
    Serial.print(velocity_limit[0]); Serial.print("\t");Serial.print(velocity_limit[1]); Serial.print("\t");Serial.print(velocity_limit[2]); Serial.print("\n");
    #endif

    
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
  Serial.print(theta_x[0]*180/PI); Serial.print("\t");Serial.print(theta_y[0]*180/PI); Serial.print("\t");Serial.print(theta_x_dot[0]*180/PI); Serial.print("\t");Serial.print(theta_y_dot[0]*180/PI); Serial.print("\n");
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

  float theta[2] = {filter_theta_x, filter_theta_y};
  float theta_dot[2] = {filter_theta_x_dot, filter_theta_y_dot};

  
  

  //Shiften der Werte

  theta_x[2] = theta_x[1];
  theta_x[1] = theta_x[0];

  theta_y[2] = theta_y[1];
  theta_y[1] = theta_y[0];

  theta_x_dot[2] = theta_x_dot[1];
  theta_x_dot[1] = theta_x_dot[0];

  theta_y_dot[2] = theta_y_dot[1];
  theta_y_dot[1] = theta_y_dot[0];



  // Read Motor speed data
  static float* motor_velocity = new float[3];

  for(int i=1; i<=3; i++)
  {
    motor_velocity[i-1] = (float)readMotorData(i-1, 4, ADDR_PRESENT_VELOCITY)*0.229*(2*PI)/(60); 
    
  }

  
  static float* phi_dot = new float[3];
  phi_dot = compute_phi_dot(motor_velocity, theta, theta_dot);  

  #ifdef PHI_DOT
  Serial.print(phi_dot[0]*180/PI); Serial.print("\t");Serial.print(phi_dot[1]*180/PI); Serial.print("\t");Serial.print(phi_dot[2]*180/PI); Serial.print("\n"); 
  #endif
  

  controller(filter_theta_x,filter_theta_y,filter_theta_x_dot,filter_theta_y_dot, phi_dot);

  #ifdef FILTER_ON
  Serial.print(filter_theta_x*180/PI); Serial.print("\t");Serial.print(filter_theta_y*180/PI); Serial.print("\t");Serial.print(filter_theta_x_dot*180/PI); Serial.print("\t");  Serial.print(filter_theta_y_dot*180/PI); Serial.print("\n");
  #endif
  
}


void controller(float theta_x, float theta_y, float theta_x_dot, float theta_y_dot, float phi_dot[])
{

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  uint8_t dxl_error = 0;                          // Dynamixel error

  //Compute Tx,Ty
  static float* virtual_torques = new float[3];
  static float* real_torques = new float[3];  

  // Computation virtual Torques
  // Torque in the yz Planar --> T_x
  virtual_torques[0] = (theta_x * K_X + phi_dot[0] * K_X_PHI_DOT + theta_x_dot * K_X_DOT )*-1;

  // Torque in the xz Planar --> T_y
  virtual_torques[1] = (theta_y * K_Y + phi_dot[1] * K_Y_PHI_DOT + theta_y_dot * K_Y_DOT )*-1;

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
//  uint32_t temp_unit = 0; 
//  uint32_t* present_units_arr =  new uint32_t[3]; 
//
//  for(int i=1; i<3; i++)
//  {
//      dxl_comm_result = pHandler->read4ByteTxRx(poHandler, i, ADDR_PRESENT_CURRENT, (uint32_t*)& temp_unit, &dxl_error);
//      
//      if (dxl_comm_result != COMM_SUCCESS)
//      {
//        pHandler->printTxRxResult(dxl_comm_result);
//      }
//      else if (dxl_error != 0)
//      {
//        pHandler->printRxPacketError(dxl_error);
//      }
//
//      present_units_arr[i-1]=temp_unit;
//  }

  int* present_units_arr =  new int[3]; 
  for(int i=1; i<=3; i++)
  {
    present_units_arr[i-1] = readMotorData(i, 4 , ADDR_PRESENT_CURRENT);
  }

  #ifdef PRESENT_UNITS
  Serial.print(present_units[0]); Serial.print("\t");Serial.print(present_units[1]); Serial.print("\t");Serial.print(present_units[2]); Serial.print("\n");
  #endif


  for (int i = 0; i < 3; i++)
  {
    if (i==1) {
      if (current_units[i] > 0)
      {
        current_units[i] += 9;//18;
      }
      else {
      current_units[i] -= 9;//18;
      }
    }
    else if (i==2) {
      if (current_units[i] > 0)
      {
        current_units[i] += 12;//25;
      }
      else {
      current_units[i] -= 12;//25;
      }
    }
    else if (i==3) {
      if (current_units[i] > 0)
      {
        current_units[i] += 11;//23;
      }
      else {
      current_units[i] -= 11;//23;
      }
   }
  }
    
 
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


int readMotorData(uint8_t id, uint8_t length, uint16_t address)
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
      dxl_comm_result = pHandler->read1ByteTxRx(poHandler, id, address, (uint8_t*)&data1, &dxl_error);
      return (int) data1;
      break;
    case 2:
      dxl_comm_result = pHandler->read2ByteTxRx(poHandler, id, address, &data2, &dxl_error);
      return (int) data2;
      break;
    case 4:
      dxl_comm_result = pHandler->read4ByteTxRx(poHandler, id, address, &data3, &dxl_error);
      return (int) data3;
      break;
  }
  if(dxl_comm_result != COMM_SUCCESS)
  {
    pHandler->printTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    pHandler->printRxPacketError(dxl_error);
  }
  
  return dxl_comm_result; // this is just zero if no error is there!
}

float *compute_phi_dot(float psi_dot[], float theta[], float theta_dot[]) 
{

  static float* ret_arr = new float[3];

  ret_arr[0] = (2*cos(theta[1])*((6*theta_dot[0])/25 + (3*pow(2,0.5)*(psi_dot[1] - 2*psi_dot[0] + psi_dot[2]))/100))/75 + (pow(2,0.5)*cos(theta[0])*sin(theta[1])*(psi_dot[0] + psi_dot[1] + psi_dot[2]))/1250 - (pow(2,0.5)*pow(3,0.5)*sin(theta[0])*sin(theta[1])*(psi_dot[1] - psi_dot[2]))/1250;
  
  ret_arr[1] = -1*(4*theta_dot[1])/625 - (pow(2,0.5)*sin(theta[0])*(psi_dot[0] + psi_dot[1] + psi_dot[2]))/1250 - (pow(2,0.5)*pow(3,0.5)*cos(theta[0])*(psi_dot[1] - psi_dot[2]))/1250;
 
  ret_arr[2] = 0;

  return ret_arr;
}

