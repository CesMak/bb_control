#include "turtlebot3_signal_manager.h"

#include <DynamixelSDK.h>

Turtlebot3SignalManager::Turtlebot3SignalManager(uint8_t buzzer_pin) : buzzer_pin_(buzzer_pin){}

void Turtlebot3SignalManager::playTone(unsigned int frequency, unsigned long duration)
{
  tone(buzzer_pin_, frequency, duration);
  delay(duration);
}

void Turtlebot3SignalManager::playTetris() {  
  playTone(E * 2, Takt / 4);
  playTone(H * 1, Takt / 8);
  playTone(C * 2, Takt / 8);
  playTone(D * 2, Takt / 4);
  playTone(C * 2, Takt / 8);
  playTone(H * 1, Takt / 8);
  playTone(A * 1, Takt / 4);
  playTone(A * 1, Takt / 8);
  playTone(C * 2, Takt / 8);
  playTone(E * 2, Takt / 8);
  playTone(E * 2, Takt / 8);
  playTone(D * 2, Takt / 8);
  playTone(C * 2, Takt / 8);
  playTone(H * 1, Takt / 2.5);
  playTone(C * 2, Takt / 8);
  playTone(D * 2, Takt / 4);
  playTone(E * 2, Takt / 4);
  playTone(C * 2, Takt / 4);
  playTone(A * 1, Takt / 4);
  playTone(A * 1, Takt / 4);
  
  delay(Takt / (8 / 3) + Takt / 4);
  
  playTone(D * 2, Takt / 3.25);
  playTone(F * 2, Takt / 8);
  playTone(A * 2, Takt / 8);
  playTone(A * 2, Takt / 8);
  playTone(G * 2, Takt / 8);
  playTone(F * 2, Takt / 8);
  playTone(E * 2, Takt / 3);
  playTone(C * 2, Takt / 8);
  playTone(E * 2, Takt / 8);
  playTone(E * 2, Takt / 8);
  playTone(D * 2, Takt / 8);
  playTone(C * 2, Takt / 8);
  playTone(H * 1, Takt / 4);
  playTone(H * 1, Takt / 8);
  playTone(C * 2, Takt / 8);
  playTone(D * 2, Takt / 4);
  playTone(E * 2, Takt / 4);
  playTone(C * 2, Takt / 4);
  playTone(A * 1, Takt / 4);
  playTone(A * 1, Takt / 4);
}
