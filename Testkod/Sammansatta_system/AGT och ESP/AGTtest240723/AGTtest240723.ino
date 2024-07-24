#include <SoftwareSerial.h>
// #include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// Define UART for Artemis Global Tracker
// #define RXD1 43  // RX pin (adjust as needed)
// #define TXD1 42  // TX pin (adjust as needed)
// #define BAUD_RATE 19200

// Activation pin for initializing UART

//REMEMBER TO DISCONNECT AND RECONNECT THE CABLE! OTHERWISE IT WILL NOT WORK

SoftwareSerial mySerial(D5, D6);


// void setup() {
//   // Initialize serial communication for debugging
//   // Start the console serial port
//   Serial.begin(115200);
//   Serial.println("Software Serial");
//   mySerial.begin(19200);
//   pinMode(32, OUTPUT);
//   delay(100);
//   Serial.println();
//   Serial.println();
//   Serial.println(F("Artemis Global Tracker"));
//   Serial.println(F("Example: Serial"));
//   Serial.println();


//   //empty the serial buffer
//   while(Serial.available() > 0)
//     Serial.read();

//   //wait for the user to press any key before beginning
//   Serial.println(F("Please check that the Serial Monitor is set to 115200 Baud"));
//   Serial.println(F("and that the line ending is set to Newline."));
//   Serial.println(F("Then click Send to start the example."));
//   Serial.println();
//   while(Serial.available() == 0)
//     ;

//   Serial.println(F("Starting example..."));
// }

// void loop() {

//   // Wait for a response
//   if (mySerial.available()) {
//     String receivedData = mySerial.readStringUntil('>');
//     // For debugging: print the received data to the serial monitor
//     Serial.println("Received: " + receivedData);
//     digitalWrite(32, LOW);
//     delay(1000);
//   } else {
//     // For debugging: print a message to the serial monitor
//     Serial.println("No data received");
//     digitalWrite(32, HIGH);
//   }
  
//   delay(1000);
//   Serial.println("Waiting for more data...");
  
// }

//SIMPLE CODE FOR TESTING THE COMMUNICATION BETWEEN THE ARTEMIS GLOBAL TRACKER AND THE ESP32

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  Serial.println("Artemis Global Tracker ready");

  mySerial.begin(19200);

  pinMode(35, OUTPUT);
  digitalWrite(35, HIGH);
}

void loop() {
  if (mySerial.available()) {
    String receivedData = mySerial.readStringUntil('>');
    Serial.println("Received: " + receivedData);
    digitalWrite(35, LOW);  // Indicate data received
    delay(1000);
    digitalWrite(35, HIGH);
  } else {
    Serial.println("No data received");
  }

  Serial.println("Waiting for more data...");
  delay(1000);
}

//...................................................................................................
