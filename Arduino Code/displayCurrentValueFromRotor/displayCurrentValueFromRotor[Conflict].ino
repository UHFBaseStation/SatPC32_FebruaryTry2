const int TxPin = 6;
const int upLed = 3;
const int downLed = 9;
String inputString = ""; // a string to hold incoming data
int satelliteElevation = 0; 
boolean readElevationFromMatlabComplete = false;
boolean readAzimuthFromMatlabComplete = false;
int sensorValue = 0;
// Convert the analog reading (which goes from 2 - 962) to an elevation ( 0 -> 180 )
int currentElevation = 0;
int currentAzimuth = 0;
int differenceInElevation = 0;
int timerID;
int degreesOff = 1;
int rateOfElevationMovement = 460; // 460 millisec / degree
#include <SoftwareSerial.h>
#include <SimpleTimer.h>
SimpleTimer timerLCD;
SimpleTimer timerCommand;
SoftwareSerial mySerial(5, TxPin);
void setup()
{
  Serial.begin(9600);
  inputString.reserve(200);
  mySerial.begin(9600);
  pinMode(upLed, OUTPUT);
  pinMode(downLed, OUTPUT);
  pinMode(TxPin, OUTPUT);
  digitalWrite(upLed, LOW);
  digitalWrite(downLed, LOW);
  digitalWrite(TxPin, HIGH);
  mySerial.write(22);
  mySerial.write(12); // Clear screen
  delay(5); // necessary delay for clear
  sensorValue = analogRead(A2); // Read elevation value from rotor
  //  Serial.println("Sensor Value: " + String(sensorValue));
  currentElevation = map(sensorValue, 0, 962, 0, 18000); // convert sensorValue to degrees * 100
  //  Serial.print("Current Elevation: " + String(currentElevation));
  satelliteElevation = currentElevation; // Set equal to sat elevation so no initial movement of rotor
  timerID = timerCommand.setInterval(degreesOff*(rateOfElevationMovement+230), giveRotorMovementCommands); // Make up/down/stay adjustments every 1 second
  timerLCD.setInterval(1000, PrintToLCD); // Prints to LCD every 1 second
}
void loop() 
{
  timerLCD.run(); // starts timer and prints to lcd every 1 second
  timerCommand.run(); // starts movement command timer to check distance from Sat elevation
  if (readElevationFromMatlabComplete){
    // Convert elevation sent by matlab to an integer
    satelliteElevation = inputString.toInt(); // 00000 to 18000
    inputString = ""; // clear the string
    readElevationFromMatlabComplete = false;
    timerCommand.restartTimer(timerID);
  }
  if (readAzimuthFromMatlabComplete) {
    currentAzimuth = inputString.toInt();
    inputString = ""; // clear the string
    readAzimuthFromMatlabComplete = false;
  }
  // Read current rotor elevation
  sensorValue = analogRead(A2); 
  currentElevation = map(sensorValue, 0, 962, 0, 18000); // Convert to degrees * 100
  Serial.println(String(currentElevation));
  differenceInElevation = satelliteElevation - currentElevation;
}

/*
      SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() 
{
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    delay(5);
    switch (inChar) {
    case 'e': // if first character is 'e'
      while (Serial.available()){
        int inNum = Serial.read();
        delay(5);
        if (isDigit(inNum)) {
          // convert the incoming byte to a char 
          // and add it to the string:
          inputString += (char)inNum; 
        }
        // if you get a newline, print the string,
        // then the string's value:
        if (inNum == '\n') {
          readElevationFromMatlabComplete = true;
        }
      }
      break;
    case 'a': // if first character is 'a'
      while (Serial.available()){
        int inNum = Serial.read();
        delay(5);
        if (isDigit(inNum)) {
          // convert the incoming byte to a char 
          // and add it to the string:
          inputString += (char)inNum; 
        }
        // if you get a newline, print the string,
        // then the string's value:
        if (inNum == '\n') {
          readAzimuthFromMatlabComplete = true;
        }
      }
      break;
    case 'r': // if first character is 'r', do below code only and return to loop
      Serial.println(String(currentElevation)); // sends Matlab current elevation
      delay(5);
      Serial.read(); // reads '\n'
      break;
    case 's':
      satelliteElevation = currentElevation;
      while (Serial.available()){
        Serial.read();
        delay(5);
      }
      break;
    case 'u':
      satelliteElevation = 18000;
      Serial.read();
      break;
    case 'd':
      satelliteElevation = 0;
      Serial.read();
      break;
    case 'o':
      Serial.println(String(sensorValue)); // sends Matlab current elevation
      delay(5);
      Serial.read(); // reads '\n'
      break;
    case 'l': // backlight on
      mySerial.write(17); // reads '\n'
      Serial.read();
      break;
    case 'p': // backlight off
      mySerial.write(18); // reads '\n'
      Serial.read();
      break;
    case 'g': // send to matlab the degrees precision before sending a move command
      Serial.println(String(degreesOff));
      delay(5);
      while (Serial.available()){
        Serial.read();
        delay(5);
      }
      break;
    case 'm':
      Serial.print("Difference In Elevation: ");
      Serial.println(String(differenceInElevation));
      Serial.print("Satellite Elevation: ");
      Serial.println(String(satelliteElevation));
      Serial.print("Current Elevation: ");
      Serial.println(String(currentElevation));
      while (Serial.available()){
        Serial.read();
        delay(5);
      }
      break;
    case 'h': // Set new degrees precision
      while (Serial.available()){
        int inNum = Serial.read();
        delay(5);
        if (isDigit(inNum)) {
          // convert the incoming byte to a char 
          // and add it to the string:
          inputString += (char)inNum; 
          degreesOff = inputString.toInt();
        }
        // if you get a newline, print the string,
        // then the string's value:
        if (inNum == '\n') {
          inputString = "";
          timerCommand.restartTimer(timerID);
          timerID = timerCommand.setInterval(degreesOff*(rateOfElevationMovement+230), giveRotorMovementCommands);
          timerCommand.restartTimer(timerID);
        }
      }
      break;
    }
  }
}

void giveRotorMovementCommands() {
  // send currentElevation down
  sensorValue = analogRead(A2); 
  currentElevation = map(sensorValue, 0, 962, 0, 18000); // Convert to degrees * 100
  differenceInElevation = satelliteElevation - currentElevation;
  if (differenceInElevation < -degreesOff*100){ // -1 degrees
    digitalWrite(upLed, LOW);
    digitalWrite(downLed, HIGH);
  }
  // send currentElevation up
  else if (differenceInElevation > degreesOff*100){  // 1 degrees
    digitalWrite(downLed, LOW);
    digitalWrite(upLed, HIGH);
  }
  else {
    digitalWrite(upLed, LOW);
    digitalWrite(downLed, LOW);
  }   
}

void PrintToLCD() {
  mySerial.print("El: ");
  mySerial.print(currentElevation/100);
  mySerial.print(".");
  mySerial.print(currentElevation % 100); 
  mySerial.write(13); // carriage return
  mySerial.print("Az: ");
  mySerial.print(currentAzimuth / 100);
  mySerial.print(".");
  mySerial.print(currentAzimuth % 100);
  mySerial.write(13); // carriage return
}





































