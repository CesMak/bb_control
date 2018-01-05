#ifndef TURTLEBOT3_SIGNAL_MANAGER_H_
#define TURTLEBOT3_SIGNAL_MANAGER_H_

#include "turtlebot3_motor_driver.h"
#include "turtlebot3_core_config.h"

#define C                 261
#define Cis               277
#define D                 293
#define Dis               311
#define E                 329
#define F                 349
#define Fis               369
#define G                 391
#define Gis               415
#define A                 440
#define Ais               466
#define H                 493
#define Takt              1700

class Turtlebot3SignalManager
{
 public:
  Turtlebot3SignalManager(uint8_t buzzer_pin);
  
  void playTone(unsigned int frequency, unsigned long duration);
  
  void playTetris();

 private:
  uint8_t buzzer_pin_;
};

#endif // TURTLEBOT3_SIGNAL_MANAGER_H_
