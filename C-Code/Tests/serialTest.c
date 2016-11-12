char response = '\0';
   
void setup() {
     
   // start serial port at 9600 bps:
   Serial.begin(9600);
   
   handShake();  // send a byte to establish contact until receiver responds
}

 void loop() {
  if(Serial.available() >= 4){
    response = Serial.read();
     
    switch(response){
    case 'B':
      Serial.print("B RECIEVED");
      Serial.flush();
      break;
    case 'E':
      Serial.print("E RECIEVED");
      Serial.flush();     
    }
  }
 }

 void handShake() {
     while (Serial.available() <= 0) {
       Serial.write('A');   // send a capital A
       delay(300);
     }
 }
