#include <thijs_rplidar.h>
#include <lidar.h>

bool keepSpinning = true;
uint32_t debugPrintTimer;
const uint32_t dubugPrintInterval = 5000; // micros between prints

void dataHandler(RPlidar *lidarPtr, uint16_t dist, uint16_t angle_q6, uint8_t newRotFlag, int8_t quality)
{
  float distFloat = dist;                        // unit is mm directly
  float angleDegreesFloat = angle_q6 * 0.015625; // angle comes in 'q6' format, so divide by (1<<6)=64 (or multiply by 1/64) (or bitshift to the right by 6) to get angle in degrees
  // alternatively, you could use bitshifting to divide the angleDegreesFloat slightly faster. Something like:
  //  float angleDegreesFloat = angle_q6;   angleDegreesFloat = (float&)(((uint32_t&)angleDegreesFloat)-=(((uint32_t)6)<<23)); // subtract 6 from the float's exponent, thereby dividing it by 2^6=64
  //
  //  debugPrintCounter++;
  //  if(debugPrintCounter >= debugPrintThreshold) {  // (debugPrintCounter >= (lidarPtr->lidarSerial.available())) {  // dynamic?
  //    debugPrintCounter = 0;
  if ((micros() - debugPrintTimer) >= dubugPrintInterval)
  { // (debugPrintCounter >= (lidarPtr->lidarSerial.available())) {  // dynamic?
    debugPrintTimer = micros();
    //// printing all the data is too slow (there's too much data), so this may cause packet loss (due to buffer overflow).
    // Serial.println(lidarPtr->lidarSerial.available());
    // Serial.print("DH: "); Serial.print(dist); Serial.print("  \t"); Serial.print(angle_q6); Serial.print('\t'); Serial.print(newRotFlag); Serial.print('\t'); Serial.println(quality);
    String dataToPrint = String(millis()) + '\t';
    dataToPrint += String(dist) + "  \t" + String(angle_q6);
    dataToPrint += '\t' + String(lidarPtr->packetCount) + '\t' + String(lidarPtr->rotationCount);
    dataToPrint += '\t' + String(newRotFlag) + '\t' + String(quality);
    dataToPrint += '\t' + String(lidarPtr->rawAnglePerMillisecond()) + '\t' + String(lidarPtr->RPM());
    Serial.println(dataToPrint);
  }
}