/*
   SIM800L SMS RELAY v1.0
   Arduino Hardware (Author): Nano V3 (ATmega328)
   Arduino IDE (Author): 1.6.9
   T.K.Hareendran/2018
*/

#include <SoftwareSerial.h>

SoftwareSerial mySerial(3, 2); // (Rx,Tx  > Tx,Rx)

char incomingByte;
String inputString;
#define Startrelay  10 // Output for Relay Control
#define Stoprelay   9 // Output for Relay Control
#define dryRun      8

#define EXE_INTERVAL 540000
#define DRY_INTERVAL 60000
unsigned long lastExecutedMillis = 0;

unsigned long currentMillis;

void setup()
{
  pinMode(Startrelay, OUTPUT);
  pinMode(Stoprelay, OUTPUT);
  pinMode(dryRun, INPUT);
  digitalWrite(Startrelay, HIGH); // Initial state of the relay
  digitalWrite(Stoprelay, HIGH); // Initial state of the relay
  Serial.begin(9600);
  mySerial.begin(9600);

  delay(30000);

  while (!mySerial.available()) {
    mySerial.println("AT");
    delay(1000);
    Serial.println("Connecting...");
  }
  Serial.println("Connected!");
  mySerial.println("AT+CMGF=1");  //Set SMS to Text Mode
  delay(1000);
  mySerial.println("AT+CNMI=1,2,0,0,0");  //Procedure to handle newly arrived messages(command name in text: new message indications to TE)
  delay(1000);
  mySerial.println("AT+CMGL=\"REC UNREAD\""); // Read Unread Messages
}

void loop()
{
  if (mySerial.available()) {
    delay(100);

    // Serial Buffer
    while (mySerial.available()) {
      incomingByte = mySerial.read();
      inputString += incomingByte;
    }

    delay(10);

    Serial.println(inputString);
    inputString.toUpperCase(); // Uppercase the Received Message

    //turn RELAY ON or OFF
    if (inputString.indexOf("ON") > -1) {
      digitalWrite(Startrelay, LOW);
      delay(1000);
      digitalWrite(Startrelay, HIGH);

      currentMillis = millis();
      lastExecutedMillis = currentMillis;

      if (currentMillis - lastExecutedMillis >= DRY_INTERVAL) {
        if (digitalRead(dryRun) == 0) {
          digitalWrite(Stoprelay, LOW);
          delay(1000);
          digitalWrite(Stoprelay, HIGH);
          delay(1000);
          mySerial.println("AT+CMGS=\"+919526774627\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
          updateSerial();
          mySerial.print("OFF"); //text content
          updateSerial();
          mySerial.write(26);
        }
        else {
          mySerial.println("AT+CMGS=\"+919526774627\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
          updateSerial();
          mySerial.print("ON"); //text content
          updateSerial();
          mySerial.write(26);
        }

      }

    }
    if (inputString.indexOf("OFF") > -1) {
      digitalWrite(Stoprelay, LOW);
      delay(1000);
      digitalWrite(Stoprelay, HIGH);
      mySerial.println("AT+CMGS=\"+919526774627\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
      updateSerial();
      mySerial.print("OFF"); //text content
      updateSerial();
      mySerial.write(26);


    }

    delay(50);

    //Delete Messages & Save Memory
    if (inputString.indexOf("OK") == -1) {
      mySerial.println("AT+CMGDA=\"DEL ALL\"");

      delay(1000);
    }

    inputString = "";
  }
  currentMillis = millis();
  if (currentMillis - lastExecutedMillis >= EXE_INTERVAL) {
    digitalWrite(Stoprelay, LOW);
    delay(1000);
    digitalWrite(Stoprelay, HIGH);
    mySerial.println("AT+CMGS=\"+919526774627\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
    updateSerial();
    mySerial.print("OFF"); //text content
    updateSerial();
    mySerial.write(26);
  }
}

void updateSerial()
{
  delay(500);
  while (Serial.available())
  {
    mySerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while (mySerial.available())
  {
    Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
  }
}
