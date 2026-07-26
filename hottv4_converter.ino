#include <Arduino.h>

// ============================================================================
// HARDWARE / PIN CONFIGURATION (ESP32-C3)
// ============================================================================
// HOTT ESC (Single-Wire 19200 Baud): RX & TX Pin anpassen
// Für Half-Duplex Single-Wire werden RX und TX extern/intern auf denselben Bus gelegt.
#define HOTT_RX_PIN    4
#define HOTT_TX_PIN    5

// BLHeli32 / KISS Telemetrie Output an den Flight Controller (115200 Baud)
// Falls du USB-Serial zur FC nutzt, verwende Serial. Hier nutzen wir Serial1 für physische Pins:
#define FC_TELEMETRY_SERIAL Serial1
#define FC_TX_PIN           21  // Sendet BLHeli32 Stream an FC RX
#define FC_RX_PIN           20  // Dummy / Unbenutzt

// HoTT ESC Sensor ID
#define HOTT_ESC_REQ_ID   0x8C  // Standard Abfrage-ID für HoTT ESC Modul

// HardwareSerial Objekt für HoTT (UART 0 auf freie Pins gemappt)
HardwareSerial HottSerial(0);

// ============================================================================
// STRUKTUREN & GLOBAL VARS
// ============================================================================
struct ESCTelemetryData {
  uint8_t  temp_c;      // Temperatur in °C
  uint16_t voltage_10mv;// Spannung in 0.01V (10mV Einheiten)
  uint16_t current_10ma;// Strom in 0.01A (10mA Einheiten)
  uint16_t mah_consumed;// Verbrauch in mAh
  uint16_t erpm;        // Elektrische RPM / 100
} telemetry;

uint32_t lastHottPollTime = 0;
const uint32_t HOTT_POLL_INTERVAL_MS = 100; // 10Hz Abfrage-Intervall für HoTT

// ============================================================================
// HELPER: CRC8 BERECHNUNG (DOW-CRC8 für BLHeli32 / KISS)
// ============================================================================
uint8_t updateCRC8(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x80) {
      crc = (crc << 1) ^ 0x07;
    } else {
      crc <<= 1;
    }
  }
  return crc;
}

uint8_t calculateBLHeliCRC(const uint8_t *buf, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc = updateCRC8(crc, buf[i]);
  }
  return crc;
}

// ============================================================================
// BLHELI_32 TELEMETRIE STREAM
// ============================================================================
void sendBLHeli32Frame() {
  uint8_t frame[10];

  frame[0] = telemetry.temp_c;                       // Temp in °C
  frame[1] = (telemetry.voltage_10mv >> 8) & 0xFF;    // Voltage High Byte
  frame[2] = telemetry.voltage_10mv & 0xFF;           // Voltage Low Byte
  frame[3] = (telemetry.current_10ma >> 8) & 0xFF;    // Current High Byte
  frame[4] = telemetry.current_10ma & 0xFF;           // Current Low Byte
  frame[5] = (telemetry.mah_consumed >> 8) & 0xFF;    // mAh High Byte
  frame[6] = telemetry.mah_consumed & 0xFF;           // mAh Low Byte
  frame[7] = (telemetry.erpm >> 8) & 0xFF;            // eRPM/100 High Byte
  frame[8] = telemetry.erpm & 0xFF;                   // eRPM/100 Low Byte
  
  // Byte 9: DOW-CRC8
  frame[9] = calculateBLHeliCRC(frame, 9);

  // Frame an Flight Controller senden
  FC_TELEMETRY_SERIAL.write(frame, 10);
}

// ============================================================================
// HOTT PARSER
// ============================================================================
void pollHoTTESC() {
  // HoTT Master Request
  HottSerial.write(HOTT_ESC_REQ_ID);
}

void parseHoTTResponse() {
  static uint8_t rxBuffer[64];
  static uint8_t rxIndex = 0;

  while (HottSerial.available()) {
    uint8_t b = HottSerial.read();

    // Start-Byte 0x7C suchen
    if (rxIndex == 0 && b != 0x7C) {
      continue;
    }

    rxBuffer[rxIndex++] = b;

    // Standard HoTT Frame Länge: 45 Bytes
    if (rxIndex >= 45) {
      uint8_t checksum = 0;
      for (uint8_t i = 0; i < 44; i++) {
        checksum += rxBuffer[i];
      }

      if (checksum == rxBuffer[44]) {
        // Daten aus HoTT Frame extrahieren
        telemetry.temp_c       = rxBuffer[21] - 20; 
        
        uint16_t raw_voltage   = (rxBuffer[18] << 8) | rxBuffer[17]; 
        telemetry.voltage_10mv = raw_voltage * 10;                  
        
        uint16_t raw_current   = (rxBuffer[20] << 8) | rxBuffer[19]; 
        telemetry.current_10ma = raw_current * 10;                  
        
        telemetry.mah_consumed = (rxBuffer[23] << 8) | rxBuffer[22]; 
        
        uint16_t raw_rpm       = (rxBuffer[16] << 8) | rxBuffer[15]; 
        telemetry.erpm         = raw_rpm / 10;                       
        
        // Weiterleitung an den FC
        sendBLHeli32Frame();
      }

      rxIndex = 0;
    }
  }
}

// ============================================================================
// SETUP & MAIN LOOP
// ============================================================================
void setup() {
  // Wichtig für ESP32-C3: Kurze Pause für stabile Spannungsversorgung beim Booten
  delay(1000);

  // HoTT Serial initialisieren (UART0 auf custom Pins mappen)
  HottSerial.begin(19200, SERIAL_8N1, HOTT_RX_PIN, HOTT_TX_PIN);

  // FC Telemetry Serial initialisieren (UART1 auf custom Pins mappen)
  FC_TELEMETRY_SERIAL.begin(115200, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);
}

void loop() {
  uint32_t now = millis();

  if (now - lastHottPollTime >= HOTT_POLL_INTERVAL_MS) {
    lastHottPollTime = now;
    pollHoTTESC();
  }

  parseHoTTResponse();

  // ESP32 Watchdog / Task Yielding (verhindert WDT Reset auf dem Single-Core ESP32-C3)
  yield();
}
