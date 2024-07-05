#include <Arduino.h>

// unsigned long startMillis;
// unsigned long delayMillis = 10000;
// char receivedChar;
// unsigned long previousMillis = 0;
// unsigned long caseoneMillis;
// unsigned long casetwoMillis;

unsigned long delayMillis = 10000;
unsigned long previousMillis = 0;
unsigned long stateStartMillis = 0;
unsigned long stateDuration = 30000;

bool printflag = false;

enum GliderState {
    caseone,
    casetwo,
    inputone,
    inputtwo
};

GliderState state = caseone;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW); // Start with LED off
  stateStartMillis = millis(); // Initialize state start time
//   startMillis = millis();

}

void loop() {
//   // put your main code here, to run repeatedly:
//   unsigned long currentMillis = millis();
//   unsigned long randomMillis = millis();

//   if (currentMillis - startMillis >= delayMillis) {
//     startMillis = currentMillis;
//     digitalWrite(BUILTIN_LED, !digitalRead(BUILTIN_LED));
//     Serial.println("Toggling LED");
//   }

//   Serial.print(F("startMillis: "));
//   Serial.print(startMillis);
//   Serial.print(F(" | currentMillis: "));
//   Serial.print(currentMillis);
//   Serial.print(F(" | delayMillis: "));
//   Serial.print(delayMillis);
//   Serial.print(F(" | randomMillis: "));
//   Serial.print(randomMillis);
//   Serial.print(F(" | LED state: "));
//   Serial.println(digitalRead(BUILTIN_LED));

//   delay(500); // Add a delay to make the Serial output more readable

    // switch(state) {
    //     case caseone:
    //         previousMillis = 0;
    //         casetwoMillis = 0;
    //         caseoneMillis = millis();
    //         if (caseoneMillis - previousMillis >= delayMillis) {
    //             previousMillis = caseoneMillis;
    //             digitalWrite(BUILTIN_LED, !digitalRead(BUILTIN_LED));
    //             Serial.println("Toggling LED");
    //         }
    //         Serial.print(F("caseoneMillis: "));
    //         Serial.println(caseoneMillis);
    //         Serial.print(F("previousMillis: "));
    //         Serial.print(previousMillis);

    //         if (caseoneMillis >= 30000) {
    //             state = casetwo;
    //             Serial.println("State changed to casetwo");
    //         }

    //         delay(500);
    //         break;

    //     case casetwo:
    //         previousMillis = 0;
    //         caseoneMillis = 0;
    //         casetwoMillis = millis();
    //         if (casetwoMillis - previousMillis >= delayMillis) {
    //             previousMillis = casetwoMillis;
    //             digitalWrite(BUILTIN_LED, !digitalRead(BUILTIN_LED));
    //             Serial.println("Toggling LED");
    //         }
    //         Serial.print(F("casetwoMillis: "));
    //         Serial.println(casetwoMillis);
    //         Serial.print(F("previousMillis: "));
    //         Serial.print(previousMillis);

    //         if (casetwoMillis >= 30000) {
    //             state = caseone;
    //             Serial.println("State changed to caseone");
    //         }

    //         delay(500);
    //             break;
    // }
  if (Serial.available() > 0) {
    char receivedChar = Serial.read(); // Read the input once

    if (receivedChar == '1') {
      state = inputone;
      Serial.println("State changed to inputone");
    } else if (receivedChar == '2') {
      state = inputtwo;
      Serial.println("State changed to inputtwo");
    }
  }
  
  unsigned long currentMillis = millis();
  
  switch (state) {
    case caseone:
      if (currentMillis - stateStartMillis < stateDuration) {
        if (currentMillis - previousMillis >= delayMillis) {
          previousMillis = currentMillis;
          digitalWrite(BUILTIN_LED, !digitalRead(BUILTIN_LED)); // Toggle LED
          Serial.println("Toggling LED in caseone");
        }
      } else {
        state = casetwo;
        stateStartMillis = currentMillis;
        previousMillis = 0;
        digitalWrite(BUILTIN_LED, LOW); // Ensure LED is off when switching state
        Serial.println("State changed to casetwo");
      }
      break;

    case casetwo:
      if (currentMillis - stateStartMillis < stateDuration) {
        if (currentMillis - previousMillis >= delayMillis) {
          previousMillis = currentMillis;
          digitalWrite(BUILTIN_LED, !digitalRead(BUILTIN_LED)); // Toggle LED
          Serial.println("Toggling LED in casetwo");
        }
      } else {
        state = caseone;
        stateStartMillis = currentMillis;
        // previousMillis = 0;
        digitalWrite(BUILTIN_LED, LOW); // Ensure LED is off when switching state
        Serial.println("State changed to caseone");
      }
      break;

    case inputone:
      if (!printflag) {
        Serial.println("Input one, waiting one second");
        printflag = true;
      }

      delay (1000);

      state = caseone;
      stateStartMillis = currentMillis;
      previousMillis = currentMillis;
    //   previousMillis = 0;
      printflag = false;
      Serial.println("State changed to caseone");
        break;

    case inputtwo:
      Serial.println("Input two");
      delay(5000);
        break; 
  }

  // Debug output
  Serial.print(F("currentMillis: "));
  Serial.print(currentMillis);
  Serial.print(F(" | previousMillis: "));
  Serial.print(previousMillis);
  Serial.print(F(" | stateStartMillis: "));
  Serial.print(stateStartMillis);
  Serial.print(F(" | state: "));
  Serial.println(String(state));

  delay(500); // Add a delay to make the Serial output more readable

}