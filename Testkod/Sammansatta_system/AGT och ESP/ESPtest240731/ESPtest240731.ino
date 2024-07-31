#include <HardwareSerial.h>

#define RX_PIN 36 // RX pin configuration
#define TX_PIN 35 // TX pin configuration
#define POKE_PIN 48 // Pin to check if the device has been poked
#define ACTIVATION_PIN 45 // Activation pin for initializing UART

String incomingString; // Variable to store incoming byte

HardwareSerial MySerial(1); // Initialize UART1

bool uartInitialized = false; // Flag to check if UART is initialized

void setup() {
  Serial.begin(115200);  // Serial for debugging
  Serial.println("ESP32-S3 ready to communicate");

  MySerial.begin(19200, SERIAL_8N1, RX_PIN, TX_PIN); // Initialize UART with specified pins
  
  pinMode(ACTIVATION_PIN, OUTPUT);  // Set the activation pin as input with pull-down resistor
  pinMode(POKE_PIN, INPUT);
}


void loop() {

  while (Serial.available() > 0) {
    incomingString = Serial.readString();
    if (incomingString == "1") {
        Serial.println("Sending message to AGT...");
        Serial.println("Waiting for handshake...");
        digitalWrite(ACTIVATION_PIN, HIGH);

        while (digitalRead(POKE_PIN) == LOW) { // Wait for handshake
          delay(100);
        }

        if (digitalRead(POKE_PIN) == HIGH) {
          Serial.println("AGT is ready!");
          // Send message to AGT
          delay(500);
          MySerial.println("m: Hi AGT, I am ESP32!]");
          delay(100);
          digitalWrite(ACTIVATION_PIN, LOW);
        }
        
    } else if (incomingString == "2") {
        Serial.println("Sending the AGT to sleep...");
        Serial.println("Waiting for handshake...");
        digitalWrite(ACTIVATION_PIN, HIGH);

        while (digitalRead(POKE_PIN) == LOW) { // Wait for handshake
          delay(100);
        }

        if (digitalRead(POKE_PIN) == HIGH) {
          Serial.println("AGT is ready!");
          // Send message to AGT
          delay(500);
          MySerial.println("s]");
          delay(100);
          digitalWrite(ACTIVATION_PIN, LOW);
        }

    } else if (incomingString == "3") {
        Serial.println("Tell the AGT to send position");
        Serial.println("Waiting for handshake...");
        digitalWrite(ACTIVATION_PIN, HIGH);

        while (digitalRead(POKE_PIN) == LOW) { // Wait for handshake
          delay(100);
        }

        if (digitalRead(POKE_PIN) == HIGH) {
          Serial.println("AGT is ready!");
          // Send message to AGT
          delay(500);
          MySerial.println("g]");
          delay(100);
          digitalWrite(ACTIVATION_PIN, LOW);
        }

    } else if (incomingString == "4") {
        Serial.println("Get message from AGT");
        Serial.println("Waiting for handshake...");

        while (digitalRead(POKE_PIN) == LOW) { // Wait for handshake
          delay(100);
        }

        if (digitalRead(POKE_PIN) == HIGH) {
          Serial.println("AGT is ready!");
          // Send message to AGT
          delay(500);
          MySerial.println("r]");
          delay(100);
          digitalWrite(ACTIVATION_PIN, LOW);
        }
    }
  }

  // if (digitalRead(POKE_PIN) == HIGH) {
  //   Serial.println("I've been poked! Repsonding...");
  //   digitalWrite(ACTIVATION_PIN, HIGH);
  //   delay(500);
  //   digitalWrite(ACTIVATION_PIN, LOW);
  // }

  // delay(1000); // Delay between messages
}


//SIMPLE CODE FOR TESTING THE COMMUNICATION BETWEEN THE ARTEMIS GLOBAL TRACKER AND THE ESP32

// void setup() {
//   Serial.begin(115200);  // Serial for debugging
//   Serial.println("ESP32 ready");

//   MySerial.begin(19200, SERIAL_8N1, RX_PIN, TX_PIN); // Initialize UART with specified pins
//   pinMode(ACTIVATION_PIN, INPUT);  // Set the activation pin as input with pull-down resistor
// }

// void loop() {
//   MySerial.print("Hi AGT, I am ESP32 >");
//   Serial.println("Message sent: Hi AGT, I am ESP32>");

//   if (digitalRead(ACTIVATION_PIN) == LOW) {
//     if (MySerial.available()) {
//       Serial.print("Received by AGT");
//     }
//   }
  
//   delay(2000);  // Delay between messages
// }