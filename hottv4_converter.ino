#include <Arduino.h>
#include <hardware/gpio.h>
#include <Adafruit_NeoPixel.h>

#define HOTT_TX_PIN         0
#define HOTT_RX_PIN         1
#define HOTT_BAUD           19200
#define HOTT_ESC_REQ_1      0x80
#define HOTT_ESC_REQ_2      0x8C
#define HOTT_FRAME_SIZE     45

// Neuer Port für BLHeli_32 Telemetrie-Ausgabe (z.B. zu Flight Controller UART)
#define BLHELI_TX_PIN       4
#define BLHELI_RX_PIN       5
#define BLHELI_BAUD         115200 // Standard für BLHeli_32 Telemetrie in Betaflight/INav

// WS2812 RGB LED Konfiguration
#define NUMPIXELS           1

Adafruit_NeoPixel strip(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

uint8_t rxBuffer[HOTT_FRAME_SIZE];
uint8_t bufferIdx = 0;
unsigned long lastRequestTime = 0;
unsigned long lastLedUpdate = 0;

enum LedState {
    LED_STATE_STANDBY,
    LED_STATE_POLL,
    LED_STATE_SUCCESS,
    LED_STATE_ERROR
};

LedState currentLedState = LED_STATE_STANDBY;
unsigned long stateTriggerTime = 0;
const unsigned long flashDuration = 60;

void setLedColor(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness = 30) {
    strip.setBrightness(brightness);
    strip.setPixelColor(0, strip.Color(r, g, b));
    strip.show();
}

void updateLedPattern() {
    unsigned long now = millis();

    if (currentLedState != LED_STATE_STANDBY) {
        if (now - stateTriggerTime > flashDuration) {
            currentLedState = LED_STATE_STANDBY;
        } else {
            switch (currentLedState) {
                case LED_STATE_POLL:
                    setLedColor(0, 0, 255, 40);
                    break;
                case LED_STATE_SUCCESS:
                    setLedColor(0, 255, 0, 40);
                    break;
                case LED_STATE_ERROR:
                    setLedColor(255, 80, 0, 40);
                    break;
                default:
                    break;
            }
            return;
        }
    }

    if (now - lastLedUpdate >= 15) {
        lastLedUpdate = now;
        float breath = (sin(now / 400.0 * 3.14159) + 1.0) / 2.0;
        uint8_t brightness = (uint8_t)(2.0 + (breath * 25.0));
        setLedColor(0, 150, 150, brightness);
    }
}

// Funktion zum Senden von BLHeli_32 Telemetrie
void sendBlheliTelemetry(uint16_t voltageRaw, uint16_t currentRaw, uint16_t rpmRaw, uint8_t temperature) {
    uint8_t blheliPacket[8];
    
    blheliPacket[0] = temperature;                 // Temperatur in °C
    blheliPacket[1] = (voltageRaw >> 8) & 0xFF;    // Spannung MSB
    blheliPacket[2] = voltageRaw & 0xFF;           // Spannung LSB
    blheliPacket[3] = (currentRaw >> 8) & 0xFF;    // Strom MSB
    blheliPacket[4] = currentRaw & 0xFF;           // Strom LSB
    blheliPacket[5] = (rpmRaw >> 8) & 0xFF;        // RPM MSB
    blheliPacket[6] = rpmRaw & 0xFF;               // RPM LSB

    // Einfache XOR/Add-Checksumme (Beispielhaft für BLHeli-Stream)
    uint8_t crc = 0;
    for (int i = 0; i < 7; i++) {
        crc += blheliPacket[i];
    }
    blheliPacket[7] = crc;

    // Ausgabe über Serial2 an den Flight Controller
    Serial2.write(blheliPacket, 8);
    Serial2.flush();
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    strip.begin();
    setLedColor(0, 150, 150, 10);

    // YGE HoTT UART (Serial1)
    Serial1.setTX(HOTT_TX_PIN);
    Serial1.setRX(HOTT_RX_PIN);
    Serial1.begin(HOTT_BAUD, SERIAL_8N1);

    // BLHeli_32 Ausgangs-UART (Serial2)
    Serial2.setTX(BLHELI_TX_PIN);
    Serial2.setRX(BLHELI_RX_PIN);
    Serial2.begin(BLHELI_BAUD, SERIAL_8N1);

    Serial.println("\n--- YGE HoTT zu BLHeli_32 Bridge gestartet ---");
}

void sendHottRequestOpenDrain() {
    gpio_set_function(HOTT_TX_PIN, GPIO_FUNC_UART);
    uint8_t request[2] = { HOTT_ESC_REQ_1, HOTT_ESC_REQ_2 };
    Serial1.write(request, 2);
    Serial1.flush();

    unsigned long t0 = millis();
    int readEcho = 0;
    while (readEcho < 2 && (millis() - t0 < 5)) {
        if (Serial1.available()) {
            Serial1.read();
            readEcho++;
        }
    }

    gpio_set_function(HOTT_TX_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(HOTT_TX_PIN, GPIO_IN);

    currentLedState = LED_STATE_POLL;
    stateTriggerTime = millis();
}

void analyzeFrame(uint8_t *data) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < HOTT_FRAME_SIZE - 1; i++) crc += data[i];

    if (crc != data[HOTT_FRAME_SIZE - 1]) {
        Serial.println("[CRC ERROR]");
        currentLedState = LED_STATE_ERROR;
        stateTriggerTime = millis();
        return;
    }

    currentLedState = LED_STATE_SUCCESS;
    stateTriggerTime = millis();

    uint16_t currentRaw  = data[14] | (data[15] << 8);
    uint16_t voltageRaw  = data[6]  | (data[7]  << 8);
    uint16_t capacityRaw = data[10] | (data[11] << 8);
    uint16_t rpmRaw      = data[18] | (data[19] << 8);
    uint16_t escTempRaw  = data[12] << 8;

    float current  = currentRaw / 10.0f;
    float voltage  = voltageRaw / 10.0f;
    uint32_t capacity = capacityRaw * 10;
    uint32_t rpm  = rpmRaw * 10;
    uint8_t escTemp = (uint8_t)(escTempRaw / 1000.0f + 20);

    // BLHeli-kompatible Telemetrie an Port 2 ausgeben
    // Hinweis: BLHeli erwartet oft RPM als Erpmm (Electrical RPM) oder direkt, 
    // hier übergeben wir die rohen/skalierten Werte passend aufbereitet.
    sendBlheliTelemetry(voltageRaw, currentRaw, rpmRaw, escTemp);

    Serial.println("\n================ YGE HoTT -> BLHeli Bridge ================");
    Serial.printf("Spannung: %5.1f V | Strom: %5.1f A | RPM: %u | Temp: %u C\n", voltage, current, rpm, escTemp);
}

void loop() {
    unsigned long now = millis();
    
    updateLedPattern();

    if (now - lastRequestTime >= 500) {
        lastRequestTime = now;
        bufferIdx = 0;
        while (Serial1.available()) Serial1.read();
        sendHottRequestOpenDrain();
    }

    while (Serial1.available()) {
        uint8_t byteIn = Serial1.read();
        if (bufferIdx == 0 && byteIn != 0x7C) continue;
        if (bufferIdx == 1 && byteIn != 0x8C) { bufferIdx = 0; continue; }

        rxBuffer[bufferIdx++] = byteIn;

        if (bufferIdx == HOTT_FRAME_SIZE) {
            analyzeFrame(rxBuffer);
            bufferIdx = 0;
        }
    }
}
