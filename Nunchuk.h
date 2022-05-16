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
     *   @file   Nunchuk.h
     * 
     *   @brief  Klassendefinition für Wii Nunchuk und Kommunikation über I2C.
     * 
     *   @author Mattheo Krümmel
     *
     *   @date   22-07-2020
     */

#ifndef NUNCHUK_H
#define NUNCHUK_H

#include <Arduino.h>

namespace communication
{
    /**************************
     * Definitionen von enums *
     **************************/

    // Exitcodes der Methoden
    struct ExitCodes
    {
        using T = uint16_t;

        enum local : T
        {
            // kein Fehler
            NO_ERROR = 0,

            // allgemeiner Fehler
            ERROR_OCCURED,

            // ungültiger Wert
            BAD_VALUE,

            // keine Verbindung zum Nunchuk möglich
            NOT_CONNECTED,

            // Datentyp nicht initialisiert
            NOT_INITIALIZED,

            // keine Daten vorliegend
            NO_DATA_AVAILABLE
        };
    };

    // Mittenwert des Joysticks in angegebener Richtung
    struct Joystick
    {
        using T = int8_t;

        enum local : T
        {
            // Mittenwert des Joysticks (links <-> rechts)
            X_NULL = 0x7D,

            // Mittenwert des Joysticks (oben <-> unten)
            Y_NULL = 0x7E
        };
    };

    // Neutralwert der Gyrosensoren in angebener Richtung
    struct Acceleration
    {
        using T = int16_t;

        enum local : T
        {
            // Neutralwert des Gyrosensors (links <-> rechts)
            X_NULL = 512,

            // Neutralwert des Gyrosensors (vor <-> zurück)
            Y_NULL = 512,

            // Neutralwert des Gyrosensors (oben <-> unten)
            Z_NULL = 512
        };
    };

    // Allgemeine Standardwerte
    struct Control
    {
        using T = uint8_t;

        enum local : T
        {
            // Länge des Arrays für Sensorendaten
            LEN_RAW_DATA = 6,

            // Länge des Arrays für Kalibrierungsdaten
            LEN_CAL_DATA = 16,

            // I2C-Adresse des Nunchuks
            ADDR_NUNCHUK = 0x52,

            // Registeradresse der Sensorendaten
            REG_RAW_DATA = 0x00,

            // Registeradresse der Kalibrierungsdaten
            REG_CAL_DATA = 0x20,

            // Registeradresse der Nunchuk-ID
            REG_ID = 0xFA,

            // Registeradresse zum Überprüfen des Verschlüsselungsstatus
            REG_IS_ENCR
        };
    };

    // I2C Bus Einstellungen
    struct BusControl
    {
        using T = uint32_t;

        enum local : T
        {
            // I2C-Frequenz im Standardmode
            I2C_CLOCK_STANDARD_100_kHz = 100000,

            // I2C-Frequenz im Fastmode
            I2C_CLOCK_FAST_400_kHz = 400000
        };
    };

    
    // Bitmasken der zusammengesetzten Register, die der Nunchuck ausgibt
    struct Bitmasks
    {
        using T = uint8_t;

        enum local : T
        {
            // Bit 0 des zusammengesetzten Registers
            // entspricht Gedrücktstatus des Buttons Z [1 = pressed/0 = released]
            BUTTON_Z_STATE = 0x01,

            // Bit 1 des zusammengesetzten Regosters
            // entspricht Gedrücktstatus des Buttons C [1 = pressed/0 = released]]
            BUTTON_C_STATE = 0x02,

            // Bits [3:2] des zusammengesetzten Registers;
            // entsprechen Bits [1:0] des Beschleunigungswertes in X-Richtung (rechts - links)
            ACC_X_BIT_0_1 = 0x0C,

            // Bits [5:4] des zusammengesetzten Registers
            // entsprechen Bits [1:0] des Beschleunigungswertes in Y-Richtung (vorne - hinten)
            ACC_Y_BIT_0_1 = 0x30,

            // Bits [7:6] des zusammengesetzten Registers
            // entsprechen Bits [1:0] des Beschleunigungswertes in Z-Richtung (oben - unten)
            ACC_Z_BIT_0_1 = 0xC0
        };
    };

    /************************************
     * Definitionen statischer Methoden *
     ************************************/

 //   int8_t operator&(const int8_t left, const Bitmasks right);
 //
 //   int16_t operator-(const int8_t left, const Acceleration right);

  /**
   * @brief         Sendet eine formatierte Ausgabe an den Seriellen Monitor
   * 
   * @mode          Art der Ausgabe, siehe debugmode
   * @annotation    Inhalt der Meldung
   */
  void serialwrite(const char *mode, const char *annotation);

  /**
   * @brief         Sendet eine formatierte erweitererte Information an den Seriellen Monitor
   * 
   * @annotation    Inhalt der Meldung
   */
  void serialverbose(const char *annotation);

  /**
   * @brief         Sendet eine formatierte informelle Benachrichtigung an den Seriellen Monitor
   * 
   * @annotation    Inhalt der Meldung
   */
  void serialinfo(const char *annotation);

  /**
   * @brief         Sendet eine formatierte Fehlermeldung mit entsprechendem Fehlercode
   *                an den Seriellen Monitor
   * 
   * @annotation    Inhalt der Meldung
   * @code          Code des auslösenden Fehlers
   */
  void serialerror(const char *annotation, const uint16_t code);

    /*************************************
     * Definitionen statischer Variablen *
     *************************************/

  // Ausgabemodus der Bibliothek
  // >= 0   Fehlermedlungen werden ausgegeben
  // > 0    informelle Benachrichtungen werden ausgegeben
  // > 1    erweiterte Informationen (v. a. für Debugging) werden ausgegeben
  constexpr const int8_t debugmode{0};

    /******************************
     * Definition der Hauptklasse *
     ******************************/

    /**
     * @brief   Klasse Nunchuk kommuniziert mit einem Nunchuk und verarbeitet und speichert die
     *          empfangenen Sensorendaten.
     */
    class Nunchuk
    {
    public:
        // Konstruktoren

        /**
         * @brief   Konstruktor der Klasse Nunchuk.
         *          Initialisiert die Memeber mit den Neutralwerten.
         */
        Nunchuk();

        /**
         * @brief   Konstruktor der Klasse Nunchuk.
         *          Initialisiert die Memeber mit den Neutralwerten und definiert die Adresse des Nunchuks
         *
         * @addr    I2C-Adresse des korrespondierenden Nunchuks
         */
        explicit Nunchuk(uint8_t addr);

        // Getter und Setter

        /*
         * @brief   Getter für m_addr
         */
        const uint8_t getAddress();

        /**
         * @brief   Getter für m_isConnected
         */
        bool isConnected();

        /**
         * @brief   Setter für m_clock
         */
        void setClock(uint32_t newfreq);

        // Andere Methoden
        
        /*
         * @brief   Initialisiert die erforderlichen Bibliotheken und Peripherie.
         *          Sollte vor Nunchuk::begin() aufgerufen werden.
         */
        void libinit();

        /**
         * @brief   Initialisierungssequenz für den Nunchuk, um mit ihm kommunizieren zu können.
         *
         * @return  Exitcode der Methode
         */
        uint16_t begin();

        /**
         * @brief   Liest die aktuellen Sensorwerte vom Nunchuk über den I2C-Bus.
         *
         * @return  Exitcode der Mehtode
         */
        uint16_t read();

        void decode(const uint8_t data[6]);

        /**
         * @brief   Extrahiert den Gedrücktstatus des Buttons Z aus dem zusammengesetzten Register
         *          und speichert ihn in m_isButtonZ.
         *
         * @x       Wert des Registers mit den zusammengesetzten Werten
         * @return  Gedrücktstatus des Buttons Z [false = released/true = pressed]
         */
        const bool decodeButtonZ(const int8_t x);

        /**
         * @brief   Extrahiert den Gedrücktstatus des Buttons C aus dem zusammengesetzten Register
         *          und speichert ihn in m_isButtonC.
         *
         * @x       Wert des Registers mit den zusammengesetzten Werten
         * @return  Gedrücktstatus des Buttons C [false = released/true = pressed]
         */
        const bool decodeButtonC(const int8_t x);

        /**
         * @brief   Extrahiert die Bits [0:1] des Beschleunigungswerts in X-Richtung aus dem
         *          zusammengesetzten Register, setzt sie mit den Bits [2:10] zusammen und
         *          speichert ihn in m_accX.
         *
         * @x       Wert des Registers mit den Bits [2:9]
         * @reg     Wert des Registers mit den zusammengesetzten Werten
         * @return  Beschleunigungswert in X-Richtung (-512;512]
         */
        const int16_t decodeAccelerationX(const uint16_t x, const uint8_t reg);

        /**
         * @brief   Extrahiert die Bits [0:1] des Beschleunigungswerts in Y-Richtung aus dem
         *          zusammengesetzten Register, setzt sie mit den Bits [2:10] zusammen und
         *          speichert ihn in m_accY.
         *
         * @x       Wert des Registers mit den Bits [2:9]
         * @reg     Wert des Registers mit den zusammengesetzten Werten
         * @return  Beschleunigungswert in Y-Richtung (-512;512]
         */
        const int16_t decodeAccelerationY(const uint16_t x, const uint8_t reg);

        /**
         * @brief   Extrahiert die Bits [0:1] des Beschleunigungswerts in Z-Richtung aus dem
         *          zusammengesetzten Register, setzt sie mit den Bits [2:10] zusammen und
         *          speichert ihn in m_accZ.
         *
         * @x       Wert des Registers mit den Bits [2:9]
         * @reg     Wert des Registers mit den zusammengesetzten Werten
         * @return  Beschleunigungswert in Z-Richtung (-512;512]
         */
        const int16_t decodeAccelerationZ(const uint16_t x, const uint8_t reg);

        /**
         * @brief   Subtrahiert den Mittenwert vom Registerwert für die Position in X-Richtung
         *          und speichert ihn in m_jsPosX.
         *
         * @x       Wert des Registers mit dem Positionswert
         * @return  Position relativ zur Mitte in X-Richtung (-125;130]
         */
        const int8_t decodeJoystickX(const uint8_t x);

        /**
         * @brief   Subtrahiert den Mittenwert vom Registerwert für die Position in Y-Richtung
         *          und speichert ihn in m_jsPosY.
         *
         * @x       Wert des Registers mit dem Positionswert
         * @return  Position relativ zur Mitte in Y-Richtung (-126;129]
         */
        const int8_t decodeJoystickY(const uint8_t x);

        /**
         * @brief   Gibt die Daten in Textform über die Serielle Schnittstelle aus.
         *
         * @return  Exitcode der Methode
         */
        void print();

    private:
        // Initialisierungsstatus der Wire-Bibliothek [true = initialisiert]
        static bool m_isWireInit;

        // Initialisierungsstatus der Serial-Bibliothek [true = initialisiert]
        static bool m_isSerialInit;

        // Adresse des korrespondierenden Nunchuks
        const uint8_t m_addr;

        // Position des Joysticks in X-Richtung relativ zur Mitte
        int8_t m_jsPosX;

        // Position des Joysticks in Y-Richtung relativ zur Mitte
        int8_t m_jsPosY;

        // Beschleunigungswert in X-Richtung
        int16_t m_accX;

        // Beschleunigungswert in Y-Richtung
        int16_t m_accY;

        // Beschleunigungswert in Z-Richtung 
        int16_t m_accZ;

        // Gedrücktstatus des C-Buttons [true = gedrückt]
        bool m_isButtonC;

        // Gedrücktstatus des Z-Buttons [true = gedrückt]
        bool m_isButtonZ;

        // Taktfrequenz der I2C-Clock
        uint32_t m_clock;

        // Code des letzten Aufgetretenen Fehlers
        uint16_t m_lastError;

        // Verbundenheitsstatus des Geräts
        bool m_isConnected;
    };
}
#endif // !NUNCHUK_H
