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

	Nunchuk::Nunchuk()
        : Nunchuk(0x00)
    {
    }

    Nunchuk::Nunchuk(uint8_t addr)
        : m_isButtonC{ false }, m_isButtonZ{ false },
        m_accX{ Acceleration::X_NULL }, m_accY{ Acceleration::Y_NULL }, m_accZ{ Acceleration::Z_NULL},
        m_jsPosX{ Joystick::X_NULL }, m_jsPosY{ Joystick::Y_NULL },
        m_addr{ addr }
    {
    }

    const uint8_t Nunchuk::getAddress()
    {
        return m_addr;
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


    I2CBus::I2CBus()
        : I2CBus(0x00)
    {
    }

    I2CBus::I2CBus(uint8_t addr)
        : I2CBus(addr, static_cast<int32_t>(BusControl::I2C_CLOCK_STANDARD_100_kHz))
    {
    }

    I2CBus::I2CBus(uint8_t addr, uint32_t clock)
        : m_device{ addr }, m_clock{ clock },
        m_isConnected{ false }, m_lastError{ static_cast<uint16_t>(ExitCodes::NO_ERROR) }
    {
    }

    Nunchuk &I2CBus::getDevice()
    {
        return m_device;
    }

    bool I2CBus::isConnected()
    {
        return m_isConnected;
    }

    uint16_t I2CBus::begin()
    {
        // Debug-Ausgaben
        Serial.println("Nunchuk initialisierung gestartet!\n");

        // I2C-Bus initialisieren
        Wire.begin();
        Wire.setClock(m_clock);
        Wire.beginTransmission(m_device.getAddress());

        // erstes Initialisierungsregister
        Wire.write((uint8_t) 0xF0);
        // auf ersten Initialisierungswert setzen
        Wire.write((uint8_t) 0x55);
        Wire.endTransmission();

        delay(1);

        Wire.beginTransmission(m_device.getAddress());
        // zweites Initialisierungsregister
        Wire.write((uint8_t) 0xFB);
        // auf zweiten Initialisierungswert setzen
        Wire.write((uint8_t) 0x00);

        if (Wire.endTransmission())
        {
            // Fehlerbehandlung falls nicht auf den Bus geschrieben werden kann
            m_isConnected = false;
            Serial.print("Nunchukinitialisierung fehlgeschlagen!\nExitcode: 0x");
            Serial.println(ExitCodes::NOT_CONNECTED, HEX);
            return m_lastError = ExitCodes::NOT_CONNECTED;
        }
        else
        {
            m_isConnected = true;
            Serial.print("Nunchukinitialisierung erfolgreich!");
        }

        return ExitCodes::NO_ERROR;
    }

    uint16_t I2CBus::read()
    {
        // Falls das Gerät nicht verbunden/initialisiert ist zweimal versuchen, sonst mit Fehler
        // zurüclkkehren
        for (int i = 0; i < 2; i++)
        {
            if (m_isConnected)
                break;
            else
                begin();
        }

        if (!m_isConnected)
        {
            Serial.print("Error: Nicht verbunden! Code: 0x");
            Serial.println(ExitCodes::NOT_CONNECTED, HEX);
            return m_lastError = ExitCodes::NOT_CONNECTED;
        }

        // Rohdaten vom Gerät anfordern
        if (!Wire.requestFrom(m_device.getAddress(), Control::LEN_RAW_DATA))
        {
            // falls Fehler bei der Kommunikation, das Gerät als getrennt markieren und mit
            // Fehler zurückkehren
            m_isConnected = false;
            Serial.print("Error: Nicht verbunden! Code: 0x");
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

        Wire.beginTransmission(m_device.getAddress());
        Wire.write(Control::REG_RAW_DATA);
        delayMicroseconds(50);
        Wire.endTransmission(true);

        // Daten formatieren und in Klasse Nunchuk speichern
        m_device.decodeJoystickX(raw[0]);
        m_device.decodeJoystickY(raw[1]);

        m_device.decodeAccelerationX(raw[2], raw[5]);
        m_device.decodeAccelerationY(raw[3], raw[5]);
        m_device.decodeAccelerationZ(raw[4], raw[5]);

        m_device.decodeButtonZ(raw[5]);
        m_device.decodeButtonC(raw[5]);

        return ExitCodes::NO_ERROR;
    }
}