#include <Arduino.h>
#include "tof.h"

Adafruit_VL6180X vl = Adafruit_VL6180X();

uint8_t lastRangeFromGround = 0;
bool tofSensorAvailable = false;


/**
 * @brief Initializes the Time-of-Flight sensor.
 * 
 * This function sets up the necessary configurations and parameters for the Time-of-Flight sensor.
 * It should be called once during the setup phase of the program.
 */
void setupTOF() {
  Serial.println("Adafruit VL6180x test!");
  if (!vl.begin()) {
    Serial.println("Failed to find sensor");
  } else {
    Serial.println("Sensor found!");
    tofSensorAvailable = true;
  }
}

/**
 * @brief This function is responsible for the main loop of the Time-of-Flight (TOF) sensor.
 * 
 * It continuously reads data from the TOF sensor and performs necessary operations.
 * 
 * @return void
 */
void loopTOF() {
  if (tofSensorAvailable) {

    uint8_t range = vl.readRange();
    uint8_t status = vl.readRangeStatus();

    if (status == VL6180X_ERROR_NONE) {
      if (range == lastRangeFromGround) {

      } else {

        //convert to string
        String rangeString = "Range: " + String(range);

        //send to display
        addToLogBuffer(rangeString.c_str());        

        lastRangeFromGround = range;
      }
    } else {
      lastRangeFromGround = static_cast<uint8_t>(999);
    }



    // Some error occurred, print it out!

    if ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
      Serial.println("System error");
    } else if (status == VL6180X_ERROR_ECEFAIL) {
      Serial.println("ECE failure");
    } else if (status == VL6180X_ERROR_NOCONVERGE) {
      Serial.println("No convergence");
    } else if (status == VL6180X_ERROR_RANGEIGNORE) {
      Serial.println("Ignoring range");
    } else if (status == VL6180X_ERROR_SNR) {
      Serial.println("Signal/Noise error");
    } else if (status == VL6180X_ERROR_RAWUFLOW) {
      Serial.println("Raw reading underflow");
    } else if (status == VL6180X_ERROR_RAWOFLOW) {
      Serial.println("Raw reading overflow");
    } else if (status == VL6180X_ERROR_RANGEUFLOW) {
      Serial.println("Range reading underflow");
    } else if (status == VL6180X_ERROR_RANGEOFLOW) {
      Serial.println("Range reading overflow");
    }
  }
}