#include <Wire.h>
#include "Nunchuk.h"

using namespace communication;
Nunchuk dev{Control::ADDR_NUNCHUK};

constexpr const uint8_t PIN_LVLSHFT_DI2C{10};
constexpr const uint8_t PIN_LVLSHFT_NUNCHUK{11};

struct State
{
  using T = uint8;

  enum local : T
  {
    LESEND,
    DECODIEREND,

  }
};

uint8_t state {static_cast<uint8_t>(State::START)};

void setup()
{
  // Pegelwandler initialisieren
  pinMode(PIN_LVLSHFT_NUNCHUK, OUTPUT);
  pinMode(PIN_LVLSHFT_DI2C, OUTPUT);
  digitalWrite(PIN_LVLSHFT_NUNCHUK, HIGH);
  digitalWrite(PIN_LVLSHFT_DI2C, LOW);
  
  dev.libinit();
  serialdebug("Pegelwandler erfolgreich initialisiert (Nunchuk aktiv, DI2C inaktiv)");

  // Nunchuk initialisieren
  dev.begin();
}

void loop()
{
  // Messwerte auslesen
  switch(state)
  {
    case static_cast<uint8_t>(State::LESEND):
      dev.read();
      state = static_cast<uint8_t>(State::DECODIEREND);
      break;
    case static_cast<uint8_t>(State::DECODIEREND):
      dev.decode();
      break;      
  }
  
  digitalWrite(11, LOW);
  Serial.println("Peripherie: Pegelwandler (Deaktiviert)");

  // Messwerte auslesen
  dev.print();
  Serial.println();
}