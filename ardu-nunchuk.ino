#include <Wire.h>
#include "Nunchuk.h"

using namespace communication;

I2CBus bus{ Control::ADDR_NUNCHUK };

void setup() {
  // Seriellen Monitor für Debug Initialisieren
  Serial.begin(9600);

  // I2C-Bus Initialisieren
  Wire.begin();
}

void loop() {
  // Falls Verbindung Verloren: erneut verbinden
  if (!bus.isConnected())
    bus.begin();

  // Messwerte auslesen
  bus.read();

  // Ausgabe der Messwerte
  // bus.getDevice().print();
}