#include <Arduino.h>

// ============================================================================
// HARDWARE HARDWARE CONFIGURATION
// ============================================================================
// HOTT_SERIAL: Empfang von Graupner HoTT ESC (z.B. Serial1)
// BLHELI_SERIAL: Ausgabe an Flight Controller / iNav / Betaflight (z.B. Serial2 oder Serial)
#define HOTT_SERIAL    Serial1
#define BLHELI_SERIAL  Serial

// HoTT ESC / GAM / EAM Sensor IDs
#define HOTT_ESC_REQ_ID   0x8C  // Standard Abfrage-ID für ESC Modul
#define HOTT_EAM_REQ_ID   0x8E  // Alternativ Electric Air Module (EAM)

// ============================================================================
// STRUKTUREN & GLOBAL VARS
// ============================================================================

// Interne Struktur zur Speicherung der konvertierten Telemetriewerte
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
// Sendet ein standardmäßiges 10-Byte BLHeli32 / KISS Telemetrie Frame
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
  
  // Byte 9 ist der CRC8 über die ersten 9 Bytes
  frame[9] = calculateBLHeliCRC(frame, 9);

  // Frame über UART an FC / iNav senden
  BLHELI_SERIAL.write(frame, 10);
}

// ============================================================================
// HOTT PARSER
// ============================================================================
void pollHoTTESC() {
  // HoTT sendet Telemetrie nur nach explizitem Request (Master/Slave Prinzipt)
  // Request Byte sendet Device ID
  HOTT_SERIAL.write(HOTT_ESC_REQ_ID);
}

void parseHoTTResponse() {
  // HoTT ESC / General Air Frames haben typischerweise 45 Bytes Länge
  static uint8_t rxBuffer[64];
  static uint8_t rxIndex = 0;

  while (HOTT_SERIAL.available()) {
    uint8_t b = HOTT_SERIAL.read();

    // Start-Byte Identifikation (HoTT v4 Data Response startet meist mit Header Byte 0x7C)
    if (rxIndex == 0 && b != 0x7C) {
      continue; // Ignorieren, bis Header gefunden wird
    }

    rxBuffer[rxIndex++] = b;

    // Standard HoTT Frame Länge ist 45 Bytes
    if (rxIndex >= 45) {
      // Prüfsummenvalidierung
      uint8_t checksum = 0;
      for (uint8_t i = 0; i < 44; i++) {
        checksum += rxBuffer[i];
      }

      if (checksum == rxBuffer[44]) {
        // Frame ist valide - Extrahiere Werte aus dem HoTT ESC Paket:
        // (Offset-Indizes basierend auf der Graupner HoTT v4 ESC Spezifikation)
        
        telemetry.temp_c       = rxBuffer[21] - 20; // HoTT Temp Offset ist gewöhnlich +20°C
        
        uint16_t raw_voltage   = (rxBuffer[18] << 8) | rxBuffer[17]; // In 0.1V Einheiten
        telemetry.voltage_10mv = raw_voltage * 10;                  // Konvertieren zu 10mV Einheiten
        
        uint16_t raw_current   = (rxBuffer[20] << 8) | rxBuffer[19]; // In 0.1A Einheiten
        telemetry.current_10ma = raw_current * 10;                  // Konvertieren zu 10mA Einheiten
        
        telemetry.mah_consumed = (rxBuffer[23] << 8) | rxBuffer[22]; // In mAh (10mAh Schritte) * 10
        
        uint16_t raw_rpm       = (rxBuffer[16] << 8) | rxBuffer[15]; // RPM / 10
        telemetry.erpm         = raw_rpm / 10;                       // BLHeli32 speichert eRPM / 100
        
        // Nach erfolgreicher Konvertierung -> Paket direkt im BLHeli32 Format weiterleiten
        sendBLHeli32Frame();
      }

      // Buffer für nächstes Paket zurücksetzen
      rxIndex = 0;
    }
  }
}

// ============================================================================
// SETUP & MAIN LOOP
// ============================================================================
void setup() {
  // HoTT v4 nutzt 19200 Baud, 8N1
  HOTT_SERIAL.begin(19200);

  // BLHeli32 / KISS Telemetrie nutzt 115200 Baud, 8N1
  BLHELI_SERIAL.begin(115200);
}

void loop() {
  uint32_t now = millis();

  // 1. Periodisch Daten vom HoTT ESC anfordern (Polling)
  if (now - lastHottPollTime >= HOTT_POLL_INTERVAL_MS) {
    lastHottPollTime = now;
    pollHoTTESC();
  }

  // 2. Empfangene HoTT Daten verarbeiten und bei Erfolg BLHeli32 Stream ausgeben
  parseHoTTResponse();
}
