#include <Wire.h>
#include "Nunchuk.h"

using namespace communication;
Nunchuk dev{Control::ADDR_NUNCHUK};

constexpr const uint8_t PIN_LVLSHFT_DI2C{10};
constexpr const uint8_t PIN_LVLSHFT_NUNCHUK{11};

void setup()
{
  // Pegelwandler initialisieren
  pinMode(PIN_LVLSHFT_NUNCHUK, OUTPUT);
  pinMode(PIN_LVLSHFT_DI2C, OUTPUT);
  digitalWrite(PIN_LVLSHFT_NUNCHUK, HIGH);
  digitalWrite(PIN_LVLSHFT_DI2C, LOW);
  
  dev.libinit();
  serialinfo("Nunchuk-Pegelwandler aktiviert. DI2C-Pegelwandler deaktiviert.");

  // Nunchuk initialisieren
  dev.begin();
}

void loop()
{
  Serial.println();
  digitalWrite(PIN_LVLSHFT_NUNCHUK, HIGH);
  serialinfo("Pegelwandler aktiviert.");

  // Messwerte auslesen
  dev.read();

  digitalWrite(PIN_LVLSHFT_NUNCHUK, LOW);
  serialinfo("Pegelwandler deaktiviert.");

  // Messwerte auslesen
  dev.print();
}