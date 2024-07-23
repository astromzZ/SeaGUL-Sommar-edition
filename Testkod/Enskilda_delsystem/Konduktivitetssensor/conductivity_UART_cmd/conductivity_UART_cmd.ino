#include <Arduino.h>
#include <SoftwareSerial.h>                           // Vi måste inkludera SoftwareSerial-biblioteket för att kunna använda det
#define rx 13                                          // Definiera vilken pin rx är kopplad till
#define tx 14                                          // Definiera vilken pin tx är kopplad till
 
SoftwareSerial myserial(rx, tx);                      // Definiera hur den mjukvaruseriella porten ska fungera
 
String inputstring = "";                              // En sträng för att hålla inkommande data från PC
String sensorstring = "";                             // En sträng för att hålla data från Atlas Scientific-produkten
boolean input_string_complete = false;                // Har vi mottagit all data från PC?
boolean sensor_string_complete = false;               // Har vi mottagit all data från Atlas Scientific-produkten?
float conductivity;                                   // Används för att hålla ett flyttal som är konduktiviteten
 
void setup()
{
  Serial.begin(9600);                                 // Sätt baudrate för hårdvaru-seriella port_0 till 9600
  myserial.begin(9600);                               // Sätt baudrate för mjukvaru-seriella porten till 9600
  inputstring.reserve(10);                            // Reservera några bytes för att ta emot data från PC
  sensorstring.reserve(30);                           // Reservera några bytes för att ta emot data från Atlas Scientific-produkten
}
 
void serialEvent()
{
  inputstring = Serial.readStringUntil(13);           // Läs strängen tills vi ser en <CR>
  input_string_complete = true;                       // Sätt flaggan för att indikera att vi har mottagit en komplett sträng från PC
}
 
void loop()
{
  if (input_string_complete == true)                  // Om en sträng från PC har mottagits i sin helhet
  {
    myserial.print(inputstring);                      // Skicka den strängen till Atlas Scientific-produkten
    myserial.print('\r');                             // Lägg till en <CR> i slutet av strängen
    inputstring = "";                                 // Rensa strängen
    input_string_complete = false;                    // Återställ flaggan för att indikera att vi har mottagit en komplett sträng från PC
  }
 
  if (myserial.available() > 0)                       // Om vi ser att Atlas Scientific-produkten har skickat en tecken
  {
    char inchar = (char)myserial.read();              // Få tecknet vi just mottagit
    sensorstring += inchar;                           // Lägg till tecknet till variabeln som heter sensorstring
    if (inchar == '\r')                               // Om det inkommande tecknet är en <CR>
    {
      sensor_string_complete = true;                  // Sätt flaggan
    }
  }
 
  if (sensor_string_complete == true)                 // Om en sträng från Atlas Scientific-produkten har mottagits i sin helhet
  {
    Serial.println(sensorstring);                     // Skicka den strängen till PC:ns seriella monitor
    sensorstring = "";                                // Rensa strängen
    sensor_string_complete = false;                   // Återställ flaggan för att indikera att vi har mottagit en komplett sträng från Atlas Scientific-produkten
  }
}
