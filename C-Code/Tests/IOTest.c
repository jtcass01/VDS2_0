#include <SdFat.h>
#include <SPI.h>

SdFatSdio sd;

File myFile;
long lVal = 0;
float fVal = 0;


void writeToFile(unsigned long time, long altitude, float velocity, float acceleration, unsigned long kAlt, float kVel, float kAccel);
void readFromFile(void);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial){
    
  }

  handShake();

  Serial.print("Initializing SD card...;");
  if(!sd.begin()){
    Serial.println("initialization failed!;");
  } else {
    Serial.println("initialization done.;");
  }

  Serial.println("Attempting to open file...");
  File myFile = sd.open("test.dat",FILE_WRITE);

  if(myFile){
    Serial.println("File successfully opened.");
    myFile.println("time, altitude, velocity, acceleration, kAlt, kVel, kAccel");
  } else {
    Serial.println("Error opening file.");
  }

  myFile.close();
}


void loop() {
  writeToFile(lVal, lVal, fVal, fVal, lVal, fVal, fVal);

  lVal += 1;
  fVal += 1;

  readFromFile();
}

void handShake() {
  while (Serial.available() <= 0) {
    Serial.write('~');   // send a capital A
    delay(300);
  }
} //END handShake()

void writeToFile(unsigned long time, long altitude, float velocity, float acceleration, unsigned long kAlt, float kVel, float kAccel){
  File myFile = sd.open("test.dat", FILE_WRITE);

  if(myFile) {
    Serial.print("Writing data to test.dat;");
  } else {
    Serial.print("Unable to open test.dat;");
  }
    myFile.print(time);
    myFile.print(",");
    myFile.print(altitude);
    myFile.print(",");
    myFile.print(velocity);
    myFile.print(",");
    myFile.print(acceleration);
    myFile.print(",");
    myFile.print(kAlt);
    myFile.print(",");
    myFile.print(kVel);
    myFile.print(",");
    myFile.print(kAccel);
    myFile.println("");
    myFile.close();
}


void readFromFile(void){
  File myFile = sd.open("8_6_16_launch.dat", FILE_READ);

  if(myFile){
    Serial.print("8_6_16_launch.dat:;");

    while(myFile.available()) {
      Serial.write(myFile.read());
      Serial.print(";");
    }

    myFile.close();
  } else {
    Serial.print("error opening the text file!;");
  }
}
