#include <HardwareSerial.h>

#define RX_PIN 36 // RX pin configuration
#define TX_PIN 35 // TX pin configuration
#define ACTIVATION_PIN 45 // Activation pin for initializing UART

HardwareSerial MySerial(1); // Initialize UART1

bool uartInitialized = false; // Flag to check if UART is initialized

void setup() {
  Serial.begin(115200);  // Serial for debugging
  Serial.println("ESP32-S3 ready to communicate");

  MySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Initialize UART with specified pins
  
  pinMode(ACTIVATION_PIN, INPUT_PULLDOWN);  // Set the activation pin as input with pull-down resistor
}

void initializeUART() {
  if (!uartInitialized) {
    MySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Initialize UART with specified pins
    Serial.println("UART initialized with pins 36 (RX) and 35 (TX)");
    uartInitialized = true; // Set the flag to true
  }
}

void deinitializeUART() {
  if (uartInitialized) {
    MySerial.end(); // Deinitialize UART
    Serial.println("UART deinitialized");
    uartInitialized = false; // Reset the flag
  }
}

void checkActivationPin() {
  if (digitalRead(ACTIVATION_PIN) == HIGH) {
    initializeUART();
  } else {
    deinitializeUART();
  }
}

void checkUARTAndReceive() {
  if (uartInitialized && MySerial.available()) {
    String received = MySerial.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(received);
  }
}

void sendMessageToUART() {
  if (uartInitialized) {
    MySerial.print("Hi AGT, I am ESP32 >");
    Serial.println("Message sent: Hi AGT, I am ESP32>");
  }
}

void loop() {
  checkActivationPin();
  checkUARTAndReceive();
  sendMessageToUART();
  delay(1000); // Delay between messages
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