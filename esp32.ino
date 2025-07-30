
// ===================== ESP32 SIDE (SPI MASTER with GPIO Interrupt) ===================== #include <Arduino.h> #include <SPI.h>

#define PIN_MISO  19 #define PIN_MOSI  23 #define PIN_SCK   18 #define PIN_CS    5 #define PIN_EVENT 21

SPIClass spi(VSPI);

uint8_t outs[8] = {0x55,0xAA,0,0,0,0,0,0}; uint8_t dirs[8] = {0xFF,0xFF,0,0,0,0,0,0}; uint8_t ins[8];

volatile bool event_flag = false;

void IRAM_ATTR onEventInterrupt() { event_flag = true; }

void spi_io_exchange(uint8_t *output, uint8_t *dir, uint8_t *input) { uint8_t tx_buf[24]; uint8_t rx_buf[24]; memcpy(tx_buf, output, 8); memcpy(tx_buf + 8, dir, 8); memset(tx_buf + 16, 0, 8);

digitalWrite(PIN_CS, LOW); for (int i = 0; i < 24; ++i) { rx_buf[i] = spi.transfer(tx_buf[i]); } digitalWrite(PIN_CS, HIGH);

memcpy(input, rx_buf + 16, 8); }

void ioTask(void *pvParameters) { for (;;) { if (event_flag) { event_flag = false; spi_io_exchange(outs, dirs, ins); Serial.print("Received: "); for (int i = 0; i < 8; i++) { Serial.print(ins[i], HEX); Serial.print(" "); } Serial.println(); } vTaskDelay(1); } }

void setup() { pinMode(PIN_CS, OUTPUT); pinMode(PIN_EVENT, INPUT); attachInterrupt(digitalPinToInterrupt(PIN_EVENT), onEventInterrupt, RISING);

digitalWrite(PIN_CS, HIGH); spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS); Serial.begin(115200);

xTaskCreatePinnedToCore(ioTask, "IO Task", 4096, NULL, 1, NULL, 1); }

void loop() {}

