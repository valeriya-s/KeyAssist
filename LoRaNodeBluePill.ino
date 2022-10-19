/*
  LoRa Simple Gateway/Node Exemple

  This code uses InvertIQ function to create a simple Gateway/Node logic.

  Gateway - Sends messages with enableInvertIQ()
          - Receives messages with disableInvertIQ()

  Node    - Sends messages with disableInvertIQ()
          - Receives messages with enableInvertIQ()

  With this arrangement a Gateway never receive messages from another Gateway
  and a Node never receive message from another Node.
  Only Gateway to Node and vice versa.

  This code receives messages and sends a message every second.

  InvertIQ function basically invert the LoRa I and Q signals.

  See the Semtech datasheet, http://www.semtech.com/images/datasheet/sx1276.pdf
  for more on InvertIQ register 0x33.

  created 05 August 2018
  by Luiz H. Cassettari
*/
// MCU CODE!!

#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <TinyGPS++.h>         //Library for using GPS functions
#include <SoftwareSerial.h>

TinyGPSPlus gps;               //Object gps for class TinyGPSPlus
SoftwareSerial ss(PA3, PA2);

const long frequency = 915E6;  // LoRa Frequency

const int csPin = PA4;  //SCK!!       // LoRa radio chip select
const int resetPin = PB1;        // LoRa radio reset
const int irqPin = PA1; // DIO0          // change for your board; must be a hardware interrupt pin

const int Button1 = PB4; //2;
const int Button2 = PB3;
const int RedLight = PC13;
const int GreenLight = PB6; //10;
const int YellowLight = PB5;
// red light tells user when distress signal is being sent
// blue light is the confirmation (HB and DS)

// BUTTON VARIABLES:
// Constants:
//const int intervalButton = 50;    // Time between two readings of the button state
const unsigned long minDuration = 3000;    // Time we wait before we see the press as a long press, used for both 1 button and 2 button sequences
const unsigned long maxDuration = 10000;    //To end send
// others:
int dLevel = 0;     // level of distress
int buttonState1;
int buttonState2;
int distressHigh = 0;
int distressLow = 0;
int buttonStatePrevious = LOW;       // previous state of the switch
unsigned long currentMillis; 
unsigned long previousButtonMillis;     // Timestamp of the latest reading
bool buttonStateLongPress = false;      // True if it is a long press
unsigned long buttonLongPressMillis;    // Time in ms when we the button was pressed
unsigned long buttonPressDuration;      // Time the button is pressed in ms

//GPS:
unsigned long start;
double lat_val, lng_val;
bool loc_valid;
String latitude;
String longitude;

// for lights:
const unsigned long timeout = 7000;    // checks if a msg was returned!
unsigned long millis4TO;

//////////////////////////////////////////////////////////
// FOR MESH:
byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends
// not sure why we would need the last 2 variables but we'll see
///////////////////////////////////////////////////////////


void setup() {
  Serial.begin(9600);                   // initialize serial
  while (!Serial);

  LoRa.setPins(csPin, resetPin, irqPin);
  
  pinMode(Button1, INPUT); 
  pinMode(Button2, INPUT);
  pinMode(RedLight, OUTPUT);
  pinMode(GreenLight, OUTPUT);
  pinMode(YellowLight, OUTPUT);
  
  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  Serial.println("LoRa init succeeded.");
  Serial.println();
  Serial.println("LoRa Simple Node");
  Serial.println("Only receive messages from gateways");
  Serial.println("Tx: invertIQ disable");
  Serial.println("Rx: invertIQ enable");
  Serial.println();

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();
  
  ss.begin(9600);
  
  millis4TO = millis();

}

void loop() {

  String message; // msg LoRa will send
  
  // for button:
  currentMillis = millis();    // store the current time
  readButtonState();
  
  if (dLevel == 0)
  {
    // HEARTBEAT CODE!!
    message = "HEARTBEAT";
    LoRa_sendMessage(message); // send a message
  
    Serial.println("Heartbeat Sent!");
    delay(3500);
  }
  else // dLevel == 1 or dLevel == 2
  { 
    GPSDelay(1000);

    //gps coordinates:
    lat_val = gps.location.lat();
    lng_val = gps.location.lng();     
      
    latitude = String(lat_val, 6);
    longitude = String(lng_val, 6);

    message = String(dLevel) + "," + latitude + "," + longitude + "," + "01";

    Serial.println("Send Following Message:");
    Serial.println(message);

    digitalWrite(YellowLight, HIGH);
    LoRa_sendMessage(message); // send a message
    
    delay(1000);
  }
}

void LoRa_rxMode(){
  LoRa.enableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  LoRa.disableInvertIQ();               // normal mode
}

void LoRa_sendMessage(String message) {
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
//  LoRa.write(localAddress);
//  LoRa.write(destination);
  LoRa.print(message);                  // add payload
  LoRa.endPacket(true);                 // finish packet and send it
}

void onReceive(int packetSize) {
  String message = "";

  while (LoRa.available()) {
    message += (char)LoRa.read();
  }

//  Serial.print("Node Receive: ");
  Serial.println(message);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));

  /////////////////////////////////
  // HEARTBEAT CODE:
  if(message.compareTo("Received HB") == 0)
  {
    Serial.println("Heartbeat Recieved!");
    digitalWrite(GreenLight, HIGH);
    digitalWrite(RedLight,LOW);
    delay(1000);
    millis4TO = millis();
  }
  /////////////////////////////////
  // DISTRESS SIGNAL CODE:
  else if(message.substring(0,11).compareTo("Received DS") == 0)
  {
//    Serial.println("Distress Signal Recieved!");
    digitalWrite(RedLight,LOW);
    digitalWrite(GreenLight, HIGH);
    if (dLevel == 1)
    {
      digitalWrite(YellowLight, LOW);
      delay(1000);
    }
    digitalWrite(YellowLight, HIGH);
    millis4TO = millis();
  }
  /////////////////////////////////
}

void onTxDone() {
//  Serial.println("TxDone");
  LoRa_rxMode();
}

static void GPSDelay(unsigned long ms)          //Delay for receiving data from GPS
{
  unsigned long start = millis();
  do
  {
    while (ss.available() > 0) 
    gps.encode(ss.read());
  } while (millis() - start < ms);
}

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

void readButtonState()
{

  //before checking buttons, check if it has recieved a signal in a while
  if((currentMillis - millis4TO) > timeout){
    Serial.println("NO SIGNAL");
    digitalWrite(GreenLight,LOW);
    digitalWrite(RedLight,HIGH);
  }
  
  buttonState1 = digitalRead(Button1);
  buttonState2 = digitalRead(Button2);
  
  if( buttonState1 == LOW || buttonState2 == LOW)
  {
    distressLow = 1;
    if( buttonState1 == LOW && buttonState2 == LOW) 
    {
      distressHigh = 1;
    }
    else 
    {
      distressLow = 1;
      distressHigh = 0;
    }
  }
  else
  {
    distressHigh = 0;
    distressLow = 0;
  }

  // INITIAL PRESS //
  if (distressLow == HIGH && buttonStatePrevious == LOW && !buttonStateLongPress) 
  {
    buttonLongPressMillis = currentMillis;
    buttonStatePrevious = HIGH;
    Serial.println("Button pressed, awaiting decision");
  }
    
  buttonPressDuration = currentMillis - buttonLongPressMillis;

  if(dLevel == 0)
  {
    // This section is when the PSD is not in distress!
    
    // BUTTON HITS MIN PRESS LENGTH
    if (distressLow  == HIGH && buttonPressDuration >= minDuration)
    {
      if (distressHigh == HIGH)
      {
        Serial.println("HIGH DISTRESS SIGNAL SENT");
        dLevel = 2;
      }
      else 
      {
        Serial.println("LOW DISTRESS SIGNAL SENT");
        dLevel = 1;
      }
    }

    // distressLow is low when the button is released
    if (distressLow == LOW && buttonStatePrevious == HIGH)
    {
      buttonStatePrevious = LOW;
//      Serial.println("Button released");

      if (buttonPressDuration < minDuration) {
        Serial.println("Button pressed shortly - distress signal not sent");
      }
    }

  }
  else
  {
    // this section is when the PSD is in distress!
    //  - if its lvl 1 change to lvl 2
    //  - if its lvl 2, dont be able to cancel :)
    if (dLevel == 1 && distressHigh  == HIGH && buttonPressDuration >= minDuration)
    {
      Serial.println("HIGH DISTRESS SIGNAL SENT");
      dLevel = 2;
    }
    /////////////////////////////////////////////////////////////////
    
    if (distressLow  == HIGH && buttonPressDuration >= maxDuration && dLevel == 1)
    {
        Serial.println("HIT 10 SECONDS - low distress signal cancelled");
        digitalWrite(YellowLight, LOW);
        dLevel = 0;
    }
    else if (distressLow  == HIGH && buttonPressDuration >= (maxDuration+20000) && dLevel == 2)
    {
        Serial.println("HIT 30 SECONDS - high distress signal cancelled");
        digitalWrite(YellowLight, LOW);
        dLevel = 0;
    } 

    if (distressLow == LOW && buttonStatePrevious == HIGH)
      {
        buttonStatePrevious = LOW;
        buttonStateLongPress = false;
        Serial.println("Button released");
      }

  }
  
  // store the current timestamp in previousButtonMillis
  previousButtonMillis = currentMillis;

}
