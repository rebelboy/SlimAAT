
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_HMC5883_U.h"

// Get the mag declination of your location here http://www.magnetic-declination.com/ and insert in # defined in next line
#define DECLINATION  -18.9 // In degrees
 
 Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

//******************************************************************************
uint8_t Initialise_Compass() {

  if(!mag.begin()) {
    Serial.println("No HMC5883 compass found!");
    return 0;
  }

  #if defined Debug_All || defined Debug_Compass
    sensor_t sensor;
    mag.getSensor(&sensor);
//    Serial.println("----- Compass Found -----");
//    Serial.print  ("Sensor:       "); Serial.println(sensor.name);
//    Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
//    Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
//    Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
//    Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
//    Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
//    Serial.println("--------------------------");
//    Serial.println("");
 #endif

 return 1;
}

//******************************************************************************
float GetMagHeading() {
  
  // Read Magnetometer
  sensors_event_t event; 
  mag.getEvent(&event);
  
  float fHeading = RadToDeg(atan2(event.magnetic.y, event.magnetic.x));  //All in degrees now
  
  fHeading += DECLINATION;  // Add magnetic declination

  fHeading = Normalise_360(fHeading);
  
  #if defined Debug_All || defined Debug_Compass
    /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
 //   Log.print("X: "); Log.print(event.magnetic.x); Log.print("  ");
 //   Log.print("Y: "); Log.print(event.magnetic.y); Log.print("  ");
 //   Log.print("Z: "); Log.print(event.magnetic.z); Log.print("  ");Log.println("uT");
    Log.print("Heading = "); Log.println(fHeading,0); 
  #endif   

  return fHeading;
}

//******************************************************************************

int16_t Normalise_360(int16_t arg) {
  if (arg < 0) arg += 360;
  if (arg > 359) arg -= 360;
  return arg;
}
//***************************************************
float RadToDeg (float _Rad) {
  return _Rad * 180 / PI;  
}
//***************************************************
