/**
 * Copyright (c) 2022, Mattheo Krümmel
 * SPDX-License-Identifier: LGPL-3.0-or-later
 */

 /**
  * @section LICENSE
  *
  * This library is free software: you can redistribute it and/or modify
  * it under the terms of the GNU Lesser General Public License as published by
  * the Free Software Foundation, version 3 or (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  * Lesser General Public License for more details.
  *
  * You should have received a copy of the GNU Lesser General Public License
  * along with this program. If not, see <http://www.gnu.org/licenses/>.
  */

  /**
   * @file   Nunchuk.cpp
   * 
   * @brief  Klassenimplementierung für Wii Nunchuk und Kommunikation über I2C.
   *         Verwendet die Klasse Wire zur Kommunikation mit dem Nunchuk.
   * 
   * @author Mattheo Krümmel
   * 
   * @date   22-07-2020
   */

#include "Nunchuk.h"

#include <Wire.h>

namespace communication
{
  
  bool Nunchuk::m_isSerialInit;
  bool Nunchuk::m_isWireInit;

	Nunchuk::Nunchuk()
        : Nunchuk(static_cast<uint8_t>(0x00))
    {
    }

    Nunchuk::Nunchuk(uint8_t addr)
        : m_isButtonC{ false },
        m_isButtonZ{ false },
        m_accX{ Acceleration::X_NULL },
        m_accY{ Acceleration::Y_NULL },
        m_accZ{ Acceleration::Z_NULL},
        m_jsPosX{ Joystick::X_NULL },
        m_jsPosY{ Joystick::Y_NULL },
        m_addr{ addr },
        m_clock{ BusControl::I2C_CLOCK_STANDARD_100_kHz },
        m_isConnected{ false },
        m_lastError{ ExitCodes::NO_ERROR }
    {
    }

    const uint8_t Nunchuk::getAddress()
    {
        return m_addr;
    }

    bool Nunchuk::isConnected()
    {
        return m_isConnected;
    }

    void Nunchuk::libinit()
    {
        if (!m_isSerialInit)
        {
            Serial.begin(115200);
            m_isSerialInit = true;
            Serial.println("Bibliothek: Serial (Initialisierung erfolgreich)");
        }
        if (!m_isWireInit)
        {
            Wire.begin();
            Wire.setClock(m_clock);
            m_isWireInit = true;
            Serial.println("Bibliothek: Wire (Initialisierung erfolgreich)");
        }
    }

    uint16_t Nunchuk::begin()
    {
        if (!m_isSerialInit || !m_isWireInit)
            libinit();

        Serial.println("Gerät: Nunchuk (Initialisierung gestartet)");
        delay(1);

        Wire.beginTransmission(m_addr);
        // erstes Initialisierungsregister
        Wire.write((uint8_t) 0xF0);
        // auf ersten Initialisierungswert setzen
        Wire.write((uint8_t) 0x55);
        Wire.endTransmission();

        delay(1);

        Wire.beginTransmission(m_addr);
        // zweites Initialisierungsregister
        Wire.write((uint8_t) 0xFB);
        // auf zweiten Initialisierungswert setzen
        Wire.write((uint8_t) 0x00);

        if (Wire.endTransmission())
        {
            // Fehlerbehandlung falls nicht auf den Bus geschrieben werden kann
            m_isConnected = false;
            Serial.print("Gerät: Nunchuk (Initialisierung fehlgeschlagen) Exitcode: 0x");
            Serial.println(ExitCodes::NOT_CONNECTED, HEX);
            return m_lastError = ExitCodes::NOT_CONNECTED;
        }
        else
        {
            m_isConnected = true;
            Serial.print("Gerät: Nunchuk (Initialisierung erfolgreich)");
        }

        return ExitCodes::NO_ERROR;
    }

    uint16_t Nunchuk::read()
    {
        // Falls das Gerät nicht verbunden/initialisiert ist zweimal versuchen, sonst mit Fehler
        // zurückkehren
        for (int i = 0; i < 2; i++)
        {
            if (m_isConnected)
                break;
            else
            {
                Serial.println("Gerät: Nunchuk (Nicht verbunden)");
                begin();
            }
        }

        if (!m_isConnected)
        {
            Serial.print("Gerät: Nunchuk (Nicht verbunden) Exitcode: 0x");
            Serial.println(ExitCodes::NOT_CONNECTED, HEX);
            return m_lastError = ExitCodes::NOT_CONNECTED;
        }

        // Rohdaten vom Gerät anfordern
        if (!Wire.requestFrom(m_addr, static_cast<uint8_t>(Control::LEN_RAW_DATA)))
        {
            // falls Fehler bei der Kommunikation, das Gerät als getrennt markieren und mit
            // Fehler zurückkehren
            m_isConnected = false;
            Serial.print("Gerät: Nunchuk (Nicht verbunden) Exitcode: 0x");
            Serial.println(ExitCodes::NOT_CONNECTED, HEX);
            return m_lastError = ExitCodes::NOT_CONNECTED;
        }

        // Array mit den Rohdaten, die vom Gerät kommen
        static uint8_t raw[Control::LEN_RAW_DATA]{ 0x00 };
        static uint16_t i = 0;

        delayMicroseconds(10);

        // empfangene Daten auslesen
        for (i = 0; (i < Control::LEN_RAW_DATA) && Wire.available(); i++)
            raw[i] = Wire.read();

        Wire.beginTransmission(m_addr);
        Wire.write(Control::REG_RAW_DATA);
        delayMicroseconds(50);
        Wire.endTransmission(true);

        Serial.println("Rohdaten");
        for (int i = 0; i < Control::LEN_RAW_DATA; i++)
        {
          Serial.print(raw[i], HEX);
          Serial.print(" ");
        }
        
        // Daten formatieren und in Klasse Nunchuk speichern
        decodeJoystickX(raw[0]);
        decodeJoystickY(raw[1]);

        decodeAccelerationX(raw[2], raw[5]);
        decodeAccelerationY(raw[3], raw[5]);
        decodeAccelerationZ(raw[4], raw[5]);

        decodeButtonZ(raw[5]);
        decodeButtonC(raw[5]);

        return ExitCodes::NO_ERROR;
    }

    const bool Nunchuk::decodeButtonZ(const int8_t x)
    {
        return m_isButtonZ = static_cast<bool>(!((x & Bitmasks::BUTTON_Z_STATE) >> 0));
    }

    const bool Nunchuk::decodeButtonC(const int8_t x)
    {
        return m_isButtonC = static_cast<bool>(!((x & Bitmasks::BUTTON_C_STATE) >> 1));
    }

    const int16_t Nunchuk::decodeAccelerationX(const uint16_t x, const uint8_t reg)
    {
        return m_accX = static_cast<int16_t>((x << 2) | (static_cast<uint16_t>((reg & Bitmasks::ACC_X_BIT_0_1) >> 2))
            - Acceleration::X_NULL);
    }

    const int16_t Nunchuk::decodeAccelerationY(const uint16_t x, const uint8_t reg)
    {
        return m_accY = static_cast<int16_t>((x << 2) | (static_cast<uint16_t>((reg & Bitmasks::ACC_Y_BIT_0_1) >> 4))
            - Acceleration::Y_NULL);
    }

    const int16_t Nunchuk::decodeAccelerationZ(const uint16_t x, const uint8_t reg)
    {
        return m_accZ = static_cast<int16_t>((x << 2) | (static_cast<uint16_t>((reg & Bitmasks::ACC_Z_BIT_0_1) >> 6))
            - Acceleration::Z_NULL);
    }

    const int8_t Nunchuk::decodeJoystickX(const uint8_t x)
    {
        return m_jsPosX = x - Joystick::X_NULL;
    }

    const int8_t Nunchuk::decodeJoystickY(const uint8_t x)
    {
        return m_jsPosY = x - Joystick::Y_NULL;
    }

    void Nunchuk::print()
    {
      if (!m_isConnected)
      {
        Serial.println("Error: Keine Daten verfügbar");
        return;
      }
      
      Serial.print("\nDaten (dezimale Werte)\n\n");
      Serial.print("Joystick:\t\t\tX = ");
      Serial.print(m_jsPosX, DEC);
      Serial.print("\tY = ");
      Serial.print(m_jsPosY, DEC);
      Serial.println();
      Serial.print("Beschleunigung:\tX = ");
      Serial.print(m_accX, DEC);
      Serial.print("\tY = ");
      Serial.print(m_accY, DEC);
      Serial.print("\tZ = ");
      Serial.print(m_accZ, DEC);
      Serial.println();
      Serial.print("Buttons:\n\tC = ");
      Serial.print(m_isButtonC ? "gedrückt" : "nicht gedrückt");
      Serial.println();
      Serial.print("\tZ = ");
      Serial.print(m_isButtonZ ? "gedrückt" : "nicht gedrückt");
      Serial.println();
    }
}