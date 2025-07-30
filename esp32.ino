// ===================== ESP32 SIDE (SPI MASTER with GPIO Interrupt) =====================
#include <Arduino.h>
#include <SPI.h>

#define PIN_MISO  19
#define PIN_MOSI  23
#define PIN_SCK   18
#define PIN_CS    5
#define PIN_EVENT 21

// SPI Configuration
#define SPI_FREQUENCY 1000000  // 1MHz - adjust based on requirements
#define SPI_TIMEOUT_MS 100

SPIClass spi(VSPI);

// GPIO state arrays
uint8_t gpio_outputs[8] = {0x55, 0xAA, 0, 0, 0, 0, 0, 0};  // Output values
uint8_t gpio_directions[8] = {0xFF, 0xFF, 0, 0, 0, 0, 0, 0}; // Direction control (1=output, 0=input)
uint8_t gpio_inputs[8] = {0};  // Input readings
uint8_t prev_inputs[8] = {0};  // Previous input state for change detection

// Event handling
volatile bool event_flag = false;
volatile bool spi_busy = false;
volatile unsigned long last_exchange = 0;
volatile unsigned long event_count = 0;

// Statistics
struct {
  unsigned long total_exchanges;
  unsigned long successful_exchanges;  
  unsigned long timeout_errors;
  unsigned long change_events;
} stats = {0};

void IRAM_ATTR onEventInterrupt() {
  if (!spi_busy) {  // Prevent interrupt storms
    event_flag = true;
    event_count++;
  }
}

bool spi_io_exchange(uint8_t *output_vals, uint8_t *directions, uint8_t *input_vals) {
  if (spi_busy) return false;
  
  spi_busy = true;
  stats.total_exchanges++;
  
  uint8_t tx_buf[16];  // Fixed: 16 bytes total (8 outputs + 8 directions)
  uint8_t rx_buf[16];  // Fixed: 16 bytes total (8 inputs + 8 unused)
  
  // Prepare transmission buffer
  memcpy(tx_buf, output_vals, 8);      // Bytes 0-7: GPIO output values
  memcpy(tx_buf + 8, directions, 8);   // Bytes 8-15: GPIO direction control
  
  // Perform SPI transaction
  spi.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(1);  // Small setup time
  
  bool success = true;
  unsigned long start_time = millis();
  
  // Exchange all 16 bytes
  for (int i = 0; i < 16; i++) {
    if (millis() - start_time > SPI_TIMEOUT_MS) {
      success = false;
      stats.timeout_errors++;
      break;
    }
    rx_buf[i] = spi.transfer(tx_buf[i]);
  }
  
  delayMicroseconds(1);  // Small hold time
  digitalWrite(PIN_CS, HIGH);
  spi.endTransaction();
  
  if (success) {
    // Copy received input data (first 8 bytes of rx_buf contain the GPIO inputs)
    memcpy(input_vals, rx_buf, 8);
    stats.successful_exchanges++;
    last_exchange = millis();
  }
  
  spi_busy = false;
  return success;
}

void printGPIOState() {
  Serial.println("=== GPIO State ===");
  
  // Print outputs and directions
  Serial.print("Outputs:    ");
  for (int i = 0; i < 8; i++) {
    Serial.printf("0x%02X ", gpio_outputs[i]);
  }
  Serial.println();
  
  Serial.print("Directions: ");
  for (int i = 0; i < 8; i++) {
    Serial.printf("0x%02X ", gpio_directions[i]);
  }
  Serial.println();
  
  // Print inputs
  Serial.print("Inputs:     ");
  for (int i = 0; i < 8; i++) {
    Serial.printf("0x%02X ", gpio_inputs[i]);
  }
  Serial.println();
  
  // Print changed pins
  Serial.print("Changes:    ");
  bool any_changes = false;
  for (int i = 0; i < 8; i++) {
    uint8_t changed_bits = gpio_inputs[i] ^ prev_inputs[i];
    if (changed_bits) {
      Serial.printf("Port%d:0x%02X ", i, changed_bits);
      any_changes = true;
    }
  }
  if (!any_changes) Serial.print("None");
  Serial.println();
  Serial.println();
}

void printStatistics() {
  Serial.println("=== Statistics ===");
  Serial.printf("Total Exchanges: %lu\n", stats.total_exchanges);
  Serial.printf("Successful: %lu\n", stats.successful_exchanges);
  Serial.printf("Timeouts: %lu\n", stats.timeout_errors);
  Serial.printf("Events: %lu\n", event_count);
  Serial.printf("Changes Detected: %lu\n", stats.change_events);
  Serial.printf("Success Rate: %.1f%%\n", 
    stats.total_exchanges > 0 ? (100.0 * stats.successful_exchanges / stats.total_exchanges) : 0.0);
  Serial.println();
}

void ioTask(void *pvParameters) {
  unsigned long last_periodic_exchange = 0;
  unsigned long last_stats_print = 0;
  const unsigned long PERIODIC_INTERVAL = 1000;  // Periodic exchange every 1s
  const unsigned long STATS_INTERVAL = 10000;    // Print stats every 10s
  
  for (;;) {
    bool should_exchange = false;
    bool is_event_driven = false;
    
    // Check for event-driven exchange
    if (event_flag) {
      event_flag = false;
      should_exchange = true;
      is_event_driven = true;
    }
    
    // Check for periodic exchange (to maintain communication)
    unsigned long now = millis();
    if (now - last_periodic_exchange >= PERIODIC_INTERVAL) {
      should_exchange = true;
      last_periodic_exchange = now;
    }
    
    // Perform SPI exchange if needed
    if (should_exchange) {
      if (spi_io_exchange(gpio_outputs, gpio_directions, gpio_inputs)) {
        // Check for input changes
        bool changes_detected = false;
        for (int i = 0; i < 8; i++) {
          if (gpio_inputs[i] != prev_inputs[i]) {
            changes_detected = true;
            break;
          }
        }
        
        if (changes_detected) {
          stats.change_events++;
          Serial.printf("[%s] GPIO Input Changes Detected:\n", 
            is_event_driven ? "EVENT" : "PERIODIC");
          printGPIOState();
          memcpy(prev_inputs, gpio_inputs, 8);
        } else if (is_event_driven) {
          // Event triggered but no changes detected - possible noise or timing issue
          Serial.println("[EVENT] Interrupt triggered but no changes detected");
        }
      } else {
        Serial.println("[ERROR] SPI exchange failed");
      }
    }
    
    // Print statistics periodically
    if (now - last_stats_print >= STATS_INTERVAL) {
      printStatistics();
      last_stats_print = now;
    }
    
    // Small delay to prevent overwhelming the system
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// Function to update GPIO outputs (can be called from other parts of code)
void setGPIOOutput(uint8_t port, uint8_t value) {
  if (port < 8) {
    gpio_outputs[port] = value;
    Serial.printf("Set GPIO Port %d output to 0x%02X\n", port, value);
  }
}

// Function to update GPIO directions (can be called from other parts of code)  
void setGPIODirection(uint8_t port, uint8_t direction) {
  if (port < 8) {
    gpio_directions[port] = direction;
    Serial.printf("Set GPIO Port %d direction to 0x%02X\n", port, direction);
  }
}

// Function to get current input state
uint8_t getGPIOInput(uint8_t port) {
  return (port < 8) ? gpio_inputs[port] : 0;
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 GPIO Expander Master Starting...");
  
  // Configure pins
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_EVENT, INPUT_PULLDOWN);  // Use pulldown to avoid floating
  digitalWrite(PIN_CS, HIGH);
  
  // Setup SPI
  spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  Serial.println("SPI initialized");
  
  // Setup interrupt
  attachInterrupt(digitalPinToInterrupt(PIN_EVENT), onEventInterrupt, RISING);
  Serial.println("Event interrupt attached");
  
  // Create IO task with higher stack size and priority
  BaseType_t result = xTaskCreatePinnedToCore(
    ioTask,           // Task function
    "IO_Task",        // Task name
    8192,             // Stack size (increased for debugging output)
    NULL,             // Parameters
    2,                // Priority (increased from 1)
    NULL,             // Task handle
    1                 // Core (run on core 1, leaving core 0 for other tasks)
  );
  
  if (result == pdPASS) {
    Serial.println("IO Task created successfully");
  } else {
    Serial.println("Failed to create IO Task");
  }
  
  // Perform initial exchange
  Serial.println("Performing initial SPI exchange...");
  if (spi_io_exchange(gpio_outputs, gpio_directions, gpio_inputs)) {
    Serial.println("Initial exchange successful");
    printGPIOState();
    memcpy(prev_inputs, gpio_inputs, 8);
  } else {
    Serial.println("Initial exchange failed");
  }
  
  Serial.println("Setup complete. Monitoring for GPIO changes...");
  Serial.println("Commands: 's' - show status, 'r' - reset stats");
}

void loop() {
  // Handle serial commands for debugging/control
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 's':
      case 'S':
        printGPIOState();
        printStatistics();
        break;
        
      case 'r':
      case 'R':
        memset(&stats, 0, sizeof(stats));
        event_count = 0;
        Serial.println("Statistics reset");
        break;
        
      case 't':
      case 'T':
        // Test pattern - toggle some outputs
        gpio_outputs[0] = ~gpio_outputs[0];
        gpio_outputs[1] = ~gpio_outputs[1];
        Serial.println("Test pattern applied");
        break;
        
      default:
        Serial.println("Commands: s=status, r=reset stats, t=test pattern");
        break;
    }
  }
  
  delay(100);  // Keep main loop responsive but not busy
}
