#include <Arduino.h>

// ============================================================================
// HARDWARE / PIN CONFIGURATION (ESP32-C3)
// ============================================================================
// HoTT ESC #1 (Hardware UART 0)
#define HOTT1_RX_PIN    4
#define HOTT1_TX_PIN    5

// HoTT ESC #2 (Hardware UART 1)
#define HOTT2_RX_PIN    6
#define HOTT2_TX_PIN    7

// BLHeli_32 Output Stream an FC (Nutzung der Haupt-Serial/USB-CDC oder TX-Pin 21)
#define FC_TELEMETRY_SERIAL Serial

// HoTT Sensor Abfrage-ID
#define HOTT_ESC_REQ_ID 0x8C

// Zwei Hardware-UART Instanzen auf dem ESP32-C3 definieren
HardwareSerial HottSerial1(0);
HardwareSerial HottSerial2(1);

// ============================================================================
// STRUKTUREN & GLOBAL VARS
// ============================================================================
struct ESCTelemetryData {
  uint8_t  temp_c;      // °C
  uint16_t voltage_10mv;// 0.01V
  uint16_t current_10ma;// 0.01A
  uint16_t mah_consumed;// mAh
  uint16_t erpm;        // eRPM / 100
};

// Telemetriedaten für beide Regler
ESCTelemetryData esc1_telemetry;
ESCTelemetryData esc2_telemetry;

// Parser-Buffer für beide Kanäle
uint8_t rxBuffer1[64];
uint8_t rxIndex1 = 0;

uint8_t rxBuffer2[64];
uint8_t rxIndex2 = 0;

uint32_t lastHottPollTime = 0;
const uint32_t HOTT_POLL_INTERVAL_MS = 100; // 10Hz Abfrage-Intervall

// ============================================================================
// HELPER: DOW-CRC8 BERECHNUNG (BLHeli32 / KISS Standard)
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
// BLHELI_32 TELEMETRIE STREAM OUTPUT
// ============================================================================
void sendBLHeli32Frame(const ESCTelemetryData &data) {
  uint8_t frame[10];

  frame[0] = data.temp_c;                       // Temp in °C
  frame[1] = (data.voltage_10mv >> 8) & 0xFF;    // Voltage High
  frame[2] = data.voltage_10mv & 0xFF;           // Voltage Low
  frame[3] = (data.current_10ma >> 8) & 0xFF;    // Current High
  frame[4] = data.current_10ma & 0xFF;           // Current Low
  frame[5] = (data.mah_consumed >> 8) & 0xFF;    // mAh High
  frame[6] = data.mah_consumed & 0xFF;           // mAh Low
  frame[7] = (data.erpm >> 8) & 0xFF;            // eRPM/100 High
  frame[8] = data.erpm & 0xFF;                   // eRPM/100 Low
  
  // Byte 9: CRC8
  frame[9] = calculateBLHeliCRC(frame, 9);

  // In den gemeinsamen Stream an die FC schreiben
  FC_TELEMETRY_SERIAL.write(frame, 10);
}

// ============================================================================
// HOTT PARSER GENERISCH
// ============================================================================
bool parseHoTTStream(HardwareSerial &port, uint8_t *buffer, uint8_t &index, ESCTelemetryData &outData) {
  while (port.available()) {
    uint8_t b = port.read();

    // Start-Byte (0x7C Header) finden
    if (index == 0 && b != 0x7C) {
      continue;
    }

    buffer[index++] = b;

    // 45 Bytes HoTT v4 Frame voll?
    if (index >= 45) {
      uint8_t checksum = 0;
      for (uint8_t i = 0; i < 44; i++) {
        checksum += buffer[i];
      }

      index = 0; // Buffer nach Paket-Prüfung sofort wieder freigeben

      if (checksum == buffer[44]) {
        // Konvertieren aus dem HoTT-Protokoll
        outData.temp_c       = buffer[21] - 20; 
        
        uint16_t raw_voltage = (buffer[18] << 8) | buffer[17]; 
        outData.voltage_10mv = raw_voltage * 10;                  
        
        uint16_t raw_current = (buffer[20] << 8) | buffer[19]; 
        outData.current_10ma = raw_current * 10;                  
        
        outData.mah_consumed = (buffer[23] << 8) | buffer[22]; 
        
        uint16_t raw_rpm     = (buffer[16] << 8) | buffer[15]; 
        outData.erpm         = raw_rpm / 10;

        return true; // Erfolgreich ein valides Frame dekodiert
      }
    }
  }
  return false;
}

// ============================================================================
// SETUP & MAIN LOOP
// ============================================================================
void setup() {
  delay(1000); // Boot-Stabilisierung für ESP32-C3

  // Telemetrie-Ausgang an den FC (115200 Baud)
  FC_TELEMETRY_SERIAL.begin(115200);

  // Beide HoTT UARTs initialisieren (19200 Baud, 8N1)
  HottSerial1.begin(19200, SERIAL_8N1, HOTT1_RX_PIN, HOTT1_TX_PIN);
  HottSerial2.begin(19200, SERIAL_8N1, HOTT2_RX_PIN, HOTT2_TX_PIN);
}

void loop() {
  uint32_t now = millis();

  // 1. Beide ESCs zeitgleich mit der Polling-ID abfragen
  if (now - lastHottPollTime >= HOTT_POLL_INTERVAL_MS) {
    lastHottPollTime = now;
    HottSerial1.write(HOTT_ESC_REQ_ID);
    HottSerial2.write(HOTT_ESC_REQ_ID);
  }

  // 2. Antwort von ESC 1 verarbeiten & bei neuem Paket senden
  if (parseHoTTStream(HottSerial1, rxBuffer1, rxIndex1, esc1_telemetry)) {
    sendBLHeli32Frame(esc1_telemetry);
  }

  // 3. Antwort von ESC 2 verarbeiten & bei neuem Paket senden
  if (parseHoTTStream(HottSerial2, rxBuffer2, rxIndex2, esc2_telemetry)) {
    sendBLHeli32Frame(esc2_telemetry);
  }

  // CPU-Entlastung für den ESP32-C3 Single-Core
  yield();
}
