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

// THIS IS JUST FOR THE ARDUINO - DONT PUT ON MCU!!! :)

#include <SPI.h>              // include libraries
#include <LoRa.h>

const long frequency = 915E6;  // LoRa Frequency

const int csPin = 10;          // LoRa radio chip select
const int resetPin = 9;        // LoRa radio reset
const int irqPin = 2;          // change for your board; must be a hardware interrupt pin

int check = 0;

////////////////////////////////////////////////////////////
/// MESH VARIABLES !!! ///
bool mode = false; 
// starts @ false
//  0 -> tower recieves from PSD
//    -> tower sends to tower
//    ---------> rx = dis, tx = dis
// after tower sends msg, switch to true
//  1 -> tower recieves from tower
//    -> tower sends to PSD
//    ---------> rx = en, tx = end
// go back to false!
unsigned long timeout;
const unsigned long maxTO = 60000;
///////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);                   // initialize serial
  while (!Serial);

  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  Serial.println("LoRa init succeeded.");
  Serial.println();
  Serial.println("LoRa Simple Gateway");
  Serial.println("Only receive messages from nodes");
  Serial.println("Tx: invertIQ enable");
  Serial.println("Rx: invertIQ disable");
  Serial.println();

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();
}

void loop() {
  if (runEvery(2500)) { // repeat every 5000 millis

    if (timeout > maxTO){
      Serial.println("timed out");
      mode = !mode;
    }
  }
}

void LoRa_rxMode(){
  if (mode == 0) {
    LoRa.disableInvertIQ();               // normal mode
  } else {
    LoRa.enableInvertIQ();
  }
  delay(3500);
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  if (mode == 0) {
    LoRa.disableInvertIQ();                // active invert I and Q signals
  } else {
    LoRa.enableInvertIQ();
  }
  delay(3500);
}

void LoRa_sendMessage(String message) {
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.print(message);                  // add payload
  LoRa.endPacket(true);                 // finish packet and send it
  mode = !mode;
  delay(3500);
}

void onReceive(int packetSize) {
  String message = "";

  while (LoRa.available()) {
    message += (char)LoRa.read();
  }

  
  message += ",N1"; // change if setting diff nodes

  Serial.println(message);

  unsigned long timeout = millis();
  LoRa_sendMessage(message);
}


void onTxDone() {
//  Serial.println("TxDone");
  LoRa_rxMode();
}

boolean runEvery(unsigned long interval)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}
