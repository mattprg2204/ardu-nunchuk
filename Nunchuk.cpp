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
void serialwrite(const char* mode = nullptr, const char* annotation = nullptr)
{
  if (!mode || !annotation)
    return;

  Serial.print("(");
  Serial.print(mode);
  Serial.print(") ");
  Serial.println(annotation);
}

void serialverbose(const char* annotation = nullptr)
{
  if constexpr (debugmode > 1)
  {
  if (!annotation)
    return;

  serialwrite("verbose", annotation);
  }
}

void serialinfo(const char* annotation = nullptr)
{
  if constexpr (debugmode > 0)
  {
  if (!annotation)
    return;

  serialwrite("info", annotation);
  }
}

void serialerror(const char* annotation = nullptr, const uint16_t code = 0x00)
{
  if constexpr (debugmode >= 0)
  {
  if (!annotation)
    return;

  static const char buffer[270] = {0x00};
  static_assert(sizeof(annotation) <= 255, "Annotation zu lang. Maximal 255 Zeichen zulässig.");

  sprintf(buffer, "%s (Code: %X#02)", annotation, code);
  serialwrite("error", buffer);
  }
}

  bool Nunchuk::m_isSerialInit {false};
  bool Nunchuk::m_isWireInit{false};

	Nunchuk::Nunchuk()
        : Nunchuk(static_cast<uint8_t>(0x00))
    {
    }

    Nunchuk::Nunchuk(uint8_t addr)
        : m_clock{ BusControl::I2C_CLOCK_STANDARD_100_kHz },
        m_raw { 0x00 },
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

    void Nunchuk::setClock(uint32_t newfreq)
    {
      m_clock = newfreq;
    }

    void Nunchuk::libinit()
    {
        if (!Nunchuk::m_isSerialInit)
        {
            Serial.begin(115200);
            m_isSerialInit = true;
            serialinfo("Serial-Bibliothek erfolgreich initialisiert.");
        }
        if (!Nunchuk::m_isWireInit)
        {
            Wire.begin();
            Wire.setClock(m_clock);
            m_isWireInit = true;
            serialinfo("Wire-Bibliothek erfolgreich initialisiert.");
        }
    }

    uint16_t Nunchuk::begin()
    {
        if (!Nunchuk::m_isSerialInit || !Nunchuk::m_isWireInit)
            libinit();

        serialverbose("Nunchuk-Initialisierung gestartet.");
        delay(1);

        Wire.beginTransmission(m_addr);
        // erstes Initialisierungsregister
        Wire.write((uint8_t) 0xF0);
        // auf ersten Initialisierungswert setzen
        Wire.write((uint8_t) 0x55);
        Wire.endTransmission(true);

        delay(1);

        Wire.beginTransmission(m_addr);
        // zweites Initialisierungsregister
        Wire.write((uint8_t) 0xFB);
        // auf zweiten Initialisierungswert setzen
        Wire.write((uint8_t) 0x00);

        if (Wire.endTransmission(true))
        {
            // Fehlerbehandlung falls nicht auf den Bus geschrieben werden kann
            m_isConnected = false;
            serialerror("Nunchukinitialisierung fehlgeschlagen.", ExitCodes::NOT_CONNECTED);
            return m_lastError = ExitCodes::NOT_CONNECTED;
        }
        else
        {
            m_isConnected = true;
            serialinfo("Nunchuk-Initalisierung erfolgreich.");
        }

        return ExitCodes::NO_ERROR;
    }

    uint16_t Nunchuk::read()
    {
        const char formatbuffer[100]{0x00};
        // Falls das Gerät nicht verbunden/initialisiert ist zweimal versuchen, sonst mit Fehler
        // zurückkehren
        for (int i = 0; i < 2; i++)
        {
            if (m_isConnected)
            {
              serialinfo("Nunchuk bereit zur Kommunikation");            
              break;
            }
            else
            {
                sprintf(formatbuffer, "Verbindungsaufbau fehlgeschlagen. Versuch %i1/3", i+1);
                serialinfo(formatbuffer);
                begin();
            }
        }

        if (!m_isConnected)
        {
            serialerror("Verbindungsaufbau fehlgeschlagen. Versuch 3/3.", ExitCodes::NOT_CONNECTED);
            return m_lastError = ExitCodes::NOT_CONNECTED;
        }

        // Rohdaten vom Gerät anfordern
        delayMicroseconds(1);

        if (!Wire.requestFrom(m_addr, static_cast<uint8_t>(Control::LEN_RAW_DATA)))
        {
            // falls Fehler bei der Kommunikation, das Gerät als getrennt markieren und mit
            // Fehler zurückkehren
            m_isConnected = false;
            serialerror("Übertragung fehlgeschlagen.", ExitCodes::NOT_CONNECTED);
            return m_lastError = ExitCodes::NOT_CONNECTED;
        }

        static uint16_t i = 0;

        sprintf(formatbuffer, "Anzahl der verfügbaren Bytes: %i", Wire.available());
        serialverbose(formatbuffer);

        // empfangene Daten auslesen
        for (i = 0; (i < Control::LEN_RAW_DATA) && Wire.available(); i++)
            m_raw[i] = Wire.read();
        
        Wire.beginTransmission(m_addr);
        Wire.write(Control::REG_RAW_DATA);
        Wire.endTransmission(true);

        serialinfo("Rohdaten:");
        if (debugmode > 0)
        {
          for (int i = 0; i < Control::LEN_RAW_DATA; i++)
          {
            Serial.print(m_raw[i], HEX);
            Serial.print(" ");
          }
          Serial.println();
        }

        return ExitCodes::NO_ERROR;
    }

    const bool Nunchuk::decodeButtonZ() const
    {
        return static_cast<bool>(!((m_raw[5] & Bitmasks::BUTTON_Z_STATE) >> 0));
    }

    const bool Nunchuk::decodeButtonC() const
    {
        return static_cast<bool>(!((m_raw[5] & Bitmasks::BUTTON_C_STATE) >> 1));
    }

    const int16_t Nunchuk::decodeAccelerationX() const
    {
        return static_cast<int16_t>((m_raw[2] << 2) | (static_cast<uint16_t>((m_raw[5] & Bitmasks::ACC_X_BIT_0_1) >> 2))
            - Acceleration::X_NULL);
    }

    const int16_t Nunchuk::decodeAccelerationY() const
    {
        return static_cast<int16_t>((m_raw[3] << 2) | (static_cast<uint16_t>((m_raw[5] & Bitmasks::ACC_Y_BIT_0_1) >> 4))
            - Acceleration::Y_NULL);
    }

    const int16_t Nunchuk::decodeAccelerationZ() const
    {
        return static_cast<int16_t>((m_raw[4] << 2) | (static_cast<uint16_t>((m_raw[5] & Bitmasks::ACC_Z_BIT_0_1) >> 6))
            - Acceleration::Z_NULL);
    }

    const int8_t Nunchuk::decodeJoystickX() const
    {
        return m_raw[0] - Joystick::X_NULL;
    }

    const int8_t Nunchuk::decodeJoystickY() const
    {
        return m_raw[1] - Joystick::Y_NULL;
    }

/*    void Nunchuk::print()
    {
      if (!m_isConnected)
      {
        serialerror("Es liegen keine neuen Sensorendaten vor.", ExitCodes::NO_DATA_AVAILABLE);
        m_lastError = ExitCodes::NO_DATA_AVAILABLE;
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
    */
}