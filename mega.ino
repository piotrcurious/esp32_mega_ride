// ===================== ATmega2560 SIDE (SPI SLAVE, INLINE ASM) ===================== // Bidirectional GPIO expander with 64 pins, direction control, and interrupt event reporting

#include <Arduino.h> #include <avr/interrupt.h> #include <util/atomic.h>

volatile uint8_t spi_rx_buf[24]; // 8 bytes output, 8 bytes direction, 8 unused volatile uint8_t spi_tx_buf[16]; // 8 bytes of input pins returned volatile uint8_t spi_index = 0; volatile uint8_t last_input[8]; volatile uint8_t new_event = 0;

#define EVENT_PIN PJ0

void setup_event_pin() { DDRJ |= (1 << EVENT_PIN); PORTJ &= ~(1 << EVENT_PIN); }

void setup_spi_slave() { DDRB |= (1 << PB3); // MISO SPCR = (1 << SPE) | (1 << SPIE); SPDR = spi_tx_buf[0]; }

//ISR(SPI_STC_vect, ISR_NAKED) { asm volatile ( "push r0                \n\t" "in r0, SREG        \n\t" "push r0                \n\t" "push r24               \n\t" "in r24, %[spdr]        \n\t" "lds r0, spi_index      \n\t" "sts spi_rx_buf + r0, r24 \n\t" "lds r24, spi_tx_buf + r0 \n\t" "out %[spdr], r24       \n\t" "inc r0                 \n\t" "cpi r0, 24             \n\t" "brlo 1f                \n\t" "ldi r0, 0              \n\t" "1:                     \n\t" "sts spi_index, r0      \n\t" "pop r24                \n\t" "pop r0                 \n\t" "out SREG, r0       \n\t" "pop r0                 \n\t" "reti                   \n\t" : : [spdr] "I" (_SFR_IO_ADDR(SPDR)) ); }
// Fixed SPI ISR with proper inline assembly
ISR(SPI_STC_vect, ISR_NAKED) {
  asm volatile (
    "push r0                \n\t"
    "in r0, __SREG__        \n\t"
    "push r0                \n\t"
    "push r24               \n\t"
    "push r30               \n\t"
    "push r31               \n\t"
    
    // Read incoming byte
    "in r24, %[spdr]        \n\t"
    
    // Load index and set up pointer to rx_buf
    "lds r30, spi_index     \n\t"
    "ldi r31, hi8(spi_rx_buf)\n\t"
    "ldi r30, lo8(spi_rx_buf)\n\t"
    "lds r0, spi_index      \n\t"
    "add r30, r0            \n\t"
    "adc r31, __zero_reg__  \n\t"
    
    // Store received byte
    "st Z, r24              \n\t"
    
    // Set up pointer to tx_buf
    "ldi r31, hi8(spi_tx_buf)\n\t"
    "ldi r30, lo8(spi_tx_buf)\n\t"
    "add r30, r0            \n\t"
    "adc r31, __zero_reg__  \n\t"
    
    // Load and send next byte
    "ld r24, Z              \n\t"
    "out %[spdr], r24       \n\t"
    
    // Increment and wrap index (0-15, not 24)
    "inc r0                 \n\t"
    "cpi r0, 16             \n\t"
    "brlo 1f                \n\t"
    "ldi r0, 0              \n\t"
    "1:                     \n\t"
    "sts spi_index, r0      \n\t"
    
    "pop r31                \n\t"
    "pop r30                \n\t"
    "pop r24                \n\t"
    "pop r0                 \n\t"
    "out __SREG__, r0       \n\t"
    "pop r0                 \n\t"
    "reti                   \n\t"
    :
    : [spdr] "I" (_SFR_IO_ADDR(SPDR))
    : "memory"
  );
}


// Improved buffer layout and synchronization
volatile uint8_t spi_rx_buf[16];  // 8 bytes output + 8 bytes direction
volatile uint8_t spi_tx_buf[16];  // 8 bytes input + 8 bytes status/unused
volatile uint8_t spi_index = 0;
volatile uint8_t last_input[8];
volatile uint8_t new_event = 0;
volatile uint8_t transaction_complete = 0;

// Fixed apply_io_logic with proper synchronization
void apply_io_logic() {
  uint8_t local_rx_buf[16];
  uint8_t new_input[8];
  
  // Copy SPI data atomically
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (transaction_complete) {
      memcpy(local_rx_buf, (const uint8_t*)spi_rx_buf, 16);
      transaction_complete = 0;
    } else {
      return; // No new data
    }
  }
  
  // Preserve SPI pins in DDRB
  uint8_t spi_ddrb_bits = DDRB & ((1 << PB2) | (1 << PB3) | (1 << PB1) | (1 << PB0));
  
  // Apply direction settings (preserve SPI bits)
  DDRA = local_rx_buf[8];
  DDRB = local_rx_buf[9] | spi_ddrb_bits;  // Keep SPI pins configured
  DDRC = local_rx_buf[10];
  DDRD = local_rx_buf[11];
  DDRE = local_rx_buf[12];
  DDRF = local_rx_buf[13];
  DDRG = local_rx_buf[14];
  DDRH = local_rx_buf[15];
  
  // Apply output values
  PORTA = local_rx_buf[0];
  PORTB = (PORTB & spi_ddrb_bits) | (local_rx_buf[1] & ~spi_ddrb_bits);
  PORTC = local_rx_buf[2];
  PORTD = local_rx_buf[3];
  PORTE = local_rx_buf[4];
  PORTF = local_rx_buf[5];
  PORTG = local_rx_buf[6];
  PORTH = local_rx_buf[7];
  
  // Read inputs
  new_input[0] = PINA;
  new_input[1] = PINB;
  new_input[2] = PINC;
  new_input[3] = PIND;
  new_input[4] = PINE;
  new_input[5] = PINF;
  new_input[6] = PING;
  new_input[7] = PINH;
  
  // Check for changes and update TX buffer
  uint8_t changed = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (uint8_t i = 0; i < 8; i++) {
      spi_tx_buf[i] = new_input[i];
      if (new_input[i] != last_input[i]) {
        changed = 1;
      }
      last_input[i] = new_input[i];
    }
  }
  
  // Signal event if changes detected
  if (changed && !new_event) {
    PORTJ |= (1 << EVENT_PIN);
    new_event = 1;
  }
}

void clear_event_if_needed() { if (new_event) { PORTJ &= ~(1 << EVENT_PIN); new_event = 0; } }

void setup() { cli(); setup_event_pin(); setup_spi_slave(); sei(); }

void loop() { apply_io_logic(); clear_event_if_needed(); }

