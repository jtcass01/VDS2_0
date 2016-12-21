#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

sensors_event_t event;

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(9600);
 
  handShake();
  delay(3000);
  Serial.println("Orientation Sensor Raw Data Test;");;

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!;");
  } else {
    Serial.println("BNO055 detected.;");
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C;");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated;");
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.print(mag, DEC);
  Serial.println(";");

  getVerticalAcceleration();

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void handShake() {
   while (Serial.available() <= 0) {
     Serial.write('~');   // send a capital A
     delay(300);
   }
}

float getVerticalAcceleration(void){
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  imu::Vector<3> linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  float linearDotGravity = 0, theta = 0, defOfProduct = 0, magOfVerticalAcceleration = 0, verticalAcceleration = 0, magL = 0, magG = 0;
  float xG=0, yG=0, zG=0, xL=0, yL=0, zL=0;

  xG = (float)gravity.x();
  yG = (float)gravity.y();
  zG = (float)gravity.z();

  xL = (float)linear.x();
  yL = (float)linear.y();
  zL = (float)linear.z();
  
  linearDotGravity = (xG*xL)+(yG*yL)+(zG*zL);

  magG = pow(((xG*xG)+(yG*yG)+(zG*zG)),.5);

  verticalAcceleration = linearDotGravity / magG;

   return verticalAcceleration;

//  Serial.print("Acceleration of gravity: (");
//  Serial.print(xG);
//  Serial.print(",");
//  Serial.print(yG);
//  Serial.print(",");
//  Serial.print(zG);
//  Serial.print(",");
//  Serial.print(magG);
//  Serial.println(");");
//
//  Serial.print("Linear Acceleration: (");
//  Serial.print(xL);
//  Serial.print(",");
//  Serial.print(yL);
//  Serial.print(",");
//  Serial.print(zL);
//  Serial.print(",");
//  Serial.print(magL);
//  Serial.println(");");
//
////  test = (float)linear.x();
//  
//  Serial.print(linearDotGravity);
//  Serial.println("; LINEAR.GRAVITY");
//  Serial.print(defOfProduct);
//  Serial.println("; defOfProduct");
//  Serial.print(verticalAcceleration);
//  Serial.println("; verticalAcceleration");
//  

}
