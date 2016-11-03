 /*
  Communications Matlab <--> Arduino
  Arduino file 1 for use with Matlab file 1 
  L. Toms
  establishContact() routine by:
  by Tom Igoe and Scott Fitzgerald
  http://www.arduino.cc/en/Tutorial/SerialCallResponse
  other ideas from
  robot grrrl
  http://robotgrrl.com/blog/2010/01/15/arduino-to-matlab-read-in-sensor-data/
  */
   int ledPin=13;
   int i = 1;
   char response = '\0';
   
   void setup() {
     
   // start serial port at 9600 bps:
   Serial.begin(9600);
   
   establishContact();  // send a byte to establish contact until receiver responds
   Serial.print("Test");
   Serial.flush();
 }

 void loop() {
   if(Serial.available() >= 4){
     response = Serial.read();
     
     if(response == 'B'){
       Serial.print("B RECIEVED");
       Serial.flush();
     }
     
     if(response == 'E'){
       Serial.print("E RECIEVED");
       Serial.flush();     
     }
     
     /*Serial.write(response);
     Serial.flush();
     response = response + 1;
     Serial.write(response);
     Serial.flush();
     delay(1000);*/
   }
   
   /*
   Serial.println(i);
   i=i+1;  
   delay(1000);*/
 }

 void establishContact() {
     while (Serial.available() <= 0) {
       Serial.write('A');   // send a capital A
       delay(300);
     }
 }
