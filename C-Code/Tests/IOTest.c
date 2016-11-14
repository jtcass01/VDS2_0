#include <SdFat.h>
#include <SPI.h>
#include <math.h>
#include <stdio.h>

SdFatSdio sd;

File myFile;
long lVal = 0;
float fVal = 0;
float accel=0, alt = 0, vel = 0;


void writeToFile(unsigned long time, long altitude, float velocity, float acceleration, unsigned long kAlt, float kVel, float kAccel);
void readFromFile(void);
void getLine(char*);
void resetNumber(char*);
float numToFloat(char*);

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
  Serial.print("Removing old test file...;");
  sd.remove("test.dat");

  Serial.println("Attempting to open file...;");
  File myFile = sd.open("test.dat",FILE_WRITE);

  if(myFile){
    Serial.println("File successfully opened.;");
    myFile.println("time, altitude, velocity, acceleration, kAlt, kVel, kAccel");
  } else {
    Serial.println("Error opening file.;");
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
  char place = '\n';
  char number[20] = {NULL};
  char nextLine[100] = {NULL};
  short numPlace = 0, numCount = 0;
  float value = 0;
  int lineIndex = 0, lineCount = 0;
  static int linePlaceHolder = 0;

  

  if(myFile){
    while(myFile.available()) {
      place = myFile.read();

      if(isdigit(place)){
        number[numPlace] = place;
      } else if (place == '.') {
        number[numPlace] = place;
      } else if (place == '-'){
        number[numPlace] = place;
      } else if (place == 'e' || place == '+'){
        number[numPlace] = place;
      } else if (place == ','){
        value = numToFloat(number);
        numCount++;
        numPlace = -1;
        switch(numCount-(lineCount*3)){
        case 1:
          vel = value;
          break;
          
        case 2:
          alt = value;
          break;
        }
        resetNumber(number);
      } else {
        value = numToFloat(number);
        numCount++;
        numPlace = -1;
        resetNumber(number);
        lineCount++;
        accel = value;
      }

      if(numCount == 0) {
        
      } else {
        if((numCount % ((linePlaceHolder+1)*3)) == 0){
          Serial.print("Velocity: ");
          Serial.print(vel);
          Serial.print(";");
          Serial.print("Altitude: ");
          Serial.print(alt);
          Serial.print(";");
          Serial.print("Acceleration: ");
          Serial.print(accel);
          Serial.print(";");
          linePlaceHolder++;
          break;
        }
      }
      numPlace++;
      
    }

    myFile.close();
  } else {
    Serial.print("error opening the text file!;");
  }
}

void resetNumber(char* number){
  for(short i = 0; i<20; i++){
    number[i] = NULL;
  }
}

float charToFloat(char input){
  int temp = input - '\0';
  temp -= 48;
  return float(temp);
}

float numToFloat(char* number){
  short index = 0, decimalIndex = 0;
  boolean decimal = false, e = false, negative = false, one = false;
  float result = 0, temp = 0;

  while(number[index] != NULL){
    if(number[index] == 'e'){
      e = true;
    } else if (number[index] == '.'){
      decimal = true;
    } else if (number[index] == '-'){
      negative = true;
    } else if (number[index] == '+'){
      ;
    } else {
      temp = charToFloat(number[index]);
      if(e){
        result *= (float)pow(10,temp);
      } else if (decimal) {
        temp *= pow(10,(-1*(decimalIndex+1)));
        result += temp;
        decimalIndex++;
      } else {
        if(one){
          result *= 10;
        } else {
           one = true;
        }
        result += temp;
      }
    }
    index++;
  }
  if(negative){
      result *= -1;
  }
  return result;
}
