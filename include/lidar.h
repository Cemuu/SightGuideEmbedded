#ifndef LIDAR_H
#define LIDAR_H
#include <thijs_rplidar.h>

extern bool keepSpinning;
extern uint32_t debugPrintTimer;
extern const uint32_t dubugPrintInterval;

struct lidarMotorHandler
{
  const uint8_t pin;
  const uint32_t freq; // Hz
  // const uint8_t res; //bits (commented because i want to keep this thing simple, and changing variable sizes (templates?) is not
  const uint8_t channel; // an ESP32 ledc specific thing
  const bool activeHigh; // depends on your specific hardware setup (CTRL_MOTO should be driven to the same voltage as 5V_MOTO (which can range from 5 to 9V), i think)
  lidarMotorHandler(const uint8_t pin, const bool activeHigh = true, const uint32_t freq = 500, /*const uint8_t res=8,*/ const uint8_t channel = 0) : pin(pin), freq(freq), /*res(res),*/ channel(channel), activeHigh(activeHigh) {}
  void init()
  {
    ledcSetup(channel, freq, 8);
    ledcAttachPin(pin, channel);
    setPWM(0);
  }
  inline void setPWM(uint8_t newPWMval) { ledcWrite(channel, activeHigh ? newPWMval : (255 - newPWMval)); }
};

void dataHandler(RPlidar *lidarPtr, uint16_t dist, uint16_t angle_q6, uint8_t newRotFlag, int8_t quality);

#endif // LIDAR_H