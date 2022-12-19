#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX

#define REPORTING_PERIOD_MS     200

// Create a PulseOximeter object
PulseOximeter pox;

// Time at which the last beat occurred
uint32_t tsLastReport = 0;

// Callback routine is executed when a pulse is detected
void onBeatDetected() {
    Serial.println("Beat!");
}

void setup() {
    Serial.begin(9600);
    mySerial.begin(9600);
    Serial.print("Initializing pulse oximeter..");

    // Initialize sensor
    if (!pox.begin()) {
        Serial.println("FAILED");
        for(;;);
    } else {
        Serial.println("SUCCESS");
    }

  // Configure sensor to use 7.6mA for LED drive
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

    // Register a callback routine
//    pox.setOnBeatDetectedCallback(onBeatDetected);
}

void loop() {
    // Read from the sensor
    pox.update();

    // Grab the updated heart rate and SpO2 levels
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
//        Serial.print("Heart rate:");
//        Serial.print(pox.getHeartRate());
//        Serial.print("bpm / SpO2:");
//        Serial.print(pox.getSpO2());
//        Serial.println("%");
          Serial.print("##");
          Serial.print("H");
          Serial.print(pox.getHeartRate());
          Serial.print("/S");
          Serial.print(pox.getSpO2());
          Serial.print("/\n");

          mySerial.print("##");
          mySerial.print("H");
          mySerial.print(pox.getHeartRate());
          mySerial.print("@S");
          mySerial.print(pox.getSpO2());
          mySerial.print("@\n");

        tsLastReport = millis();
    }
}
