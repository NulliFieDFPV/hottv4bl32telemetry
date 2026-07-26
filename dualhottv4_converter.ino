#include <Arduino.h>

// ============================================================================
// HARDWARE / PIN CONFIGURATION (RP2040-Zero)
// ============================================================================
// HoTT ESC 1 & 2 nutzen PIO für echten Single-Wire Half-Duplex Betrieb
#define HOTT1_PIN    0  // GP0 für ESC 1
#define HOTT2_PIN    1  // GP1 für ESC 2

// BLHeli32 Output Stream nutzt Hardware UART0
#define FC_TX_PIN    4  // GP4 (TX0)
#define FC_RX_PIN    5  // GP5 (Unbenutzt)

// HoTT Sensor Abfrage-ID
#define HOTT_ESC_REQ_ID 0x8C

// Hardware UART für den Telemetrie-Output an den Flight Controller
#define FC_TELEMETRY_SERIAL Serial1

// SoftwareSerial via PIO für echten Single-Wire Half-Duplex Betrieb
#include <SoftwareSerial.h>
SoftwareSerial Hott1Serial(HOTT1_PIN, HOTT1_PIN); 
SoftwareSerial Hott2Serial(HOTT2_PIN, HOTT2_PIN);

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

ESCTelemetryData esc1_telemetry;
ESCTelemetryData esc2_telemetry;

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
// HOTT PARSER (GENERISCH)
// ============================================================================
bool parseHoTTStream(Stream &port, uint8_t *buffer, uint8_t &index, ESCTelemetryData &outData) {
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

        return true; // Valid dekodiertes Paket
      }
    }
  }
  return false;
}

// ============================================================================
// SETUP & MAIN LOOP
// ============================================================================
void setup() {
  // 1. Hardware UART0 für FC-Telemetrie initialisieren (115200 Baud)
  FC_TELEMETRY_SERIAL.setTX(FC_TX_PIN);
  FC_TELEMETRY_SERIAL.setRX(FC_RX_PIN);
  FC_TELEMETRY_SERIAL.begin(115200);

  // 2. PIO-basierte SoftwareSerial Ports für HoTT initialisieren (19200 Baud)
  // Das Earle Philhower Core nutzt automatisch die RP2040 PIO-State-Machines
  // und schaltet die Pins im Single-Wire Modus ohne externe Beschaltung.
  Hott1Serial.begin(19200);
  Hott2Serial.begin(19200);
  
  // Aktivieren des Half-Duplex Single-Wire Betriebs
  Hott1Serial.enableIntTx(false); 
  Hott2Serial.enableIntTx(false);
}

void loop() {
  uint32_t now = millis();

  // 1. Beide ESCs zeitgleich mit der Request-ID anfragen (Polling)
  if (now - lastHottPollTime >= HOTT_POLL_INTERVAL_MS) {
    lastHottPollTime = now;
    
    Hott1Serial.write(HOTT_ESC_REQ_ID);
    Hott2Serial.write(HOTT_ESC_REQ_ID);
  }

  // 2. Antwort von ESC 1 verarbeiten & bei neuem Paket senden
  if (parseHoTTStream(Hott1Serial, rxBuffer1, rxIndex1, esc1_telemetry)) {
    sendBLHeli32Frame(esc1_telemetry);
  }

  // 3. Antwort von ESC 2 verarbeiten & bei neuem Paket senden
  if (parseHoTTStream(Hott2Serial, rxBuffer2, rxIndex2, esc2_telemetry)) {
    sendBLHeli32Frame(esc2_telemetry);
  }
}
