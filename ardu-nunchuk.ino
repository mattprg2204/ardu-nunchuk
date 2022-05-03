#include <Wire.h>
#include "Nunchuk.h"

using namespace communication;
Nunchuk dev{Control::ADDR_NUNCHUK};

void setup() {
  
  // Pegelwandler initialisieren
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  digitalWrite(11, LOW);
  digitalWrite(10, LOW);
  Serial.println("Peripherie: Pegelwandler (Initialisierung erfolgreich)");
  
  dev.libinit();

  digitalWrite(11, HIGH);
  Serial.println();
  Serial.println("Peripherie: Pegelwandler (Aktiviert)");
}

void loop() {

  // ggf. Nunchuk initialisieren
  dev.begin();

  // Messwerte auslesen
  dev.read();
  
  // digitalWrite(11, LOW);
  // Serial.println("Peripherie: Pegelwandler (Deaktiviert)");
  // Serial.println();

  // Messwerte auslesen
  dev.print();
}