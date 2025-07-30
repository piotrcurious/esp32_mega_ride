// ===================== ATmega2560 SIDE (SPI SLAVE, INLINE ASM) ===================== // Bidirectional GPIO expander with 64 pins, direction control, and interrupt event reporting

#include <Arduino.h> #include <avr/interrupt.h> #include <util/atomic.h>

volatile uint8_t spi_rx_buf[24]; // 8 bytes output, 8 bytes direction, 8 unused volatile uint8_t spi_tx_buf[16]; // 8 bytes of input pins returned volatile uint8_t spi_index = 0; volatile uint8_t last_input[8]; volatile uint8_t new_event = 0;

#define EVENT_PIN PJ0

void setup_event_pin() { DDRJ |= (1 << EVENT_PIN); PORTJ &= ~(1 << EVENT_PIN); }

void setup_spi_slave() { DDRB |= (1 << PB3); // MISO SPCR = (1 << SPE) | (1 << SPIE); SPDR = spi_tx_buf[0]; }

ISR(SPI_STC_vect, ISR_NAKED) { asm volatile ( "push r0                \n\t" "in r0, SREG        \n\t" "push r0                \n\t" "push r24               \n\t" "in r24, %[spdr]        \n\t" "lds r0, spi_index      \n\t" "sts spi_rx_buf + r0, r24 \n\t" "lds r24, spi_tx_buf + r0 \n\t" "out %[spdr], r24       \n\t" "inc r0                 \n\t" "cpi r0, 24             \n\t" "brlo 1f                \n\t" "ldi r0, 0              \n\t" "1:                     \n\t" "sts spi_index, r0      \n\t" "pop r24                \n\t" "pop r0                 \n\t" "out SREG, r0       \n\t" "pop r0                 \n\t" "reti                   \n\t" : : [spdr] "I" (_SFR_IO_ADDR(SPDR)) ); }

void apply_io_logic() { ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { DDRA = spi_rx_buf[8]; DDRB = spi_rx_buf[9]; DDRC = spi_rx_buf[10]; DDRD = spi_rx_buf[11]; DDRE = spi_rx_buf[12]; DDRF = spi_rx_buf[13]; DDRG = spi_rx_buf[14]; DDRH = spi_rx_buf[15];

PORTA = spi_rx_buf[0]; PORTB = spi_rx_buf[1]; PORTC = spi_rx_buf[2]; PORTD = spi_rx_buf[3];
PORTE = spi_rx_buf[4]; PORTF = spi_rx_buf[5]; PORTG = spi_rx_buf[6]; PORTH = spi_rx_buf[7];

uint8_t new_input[8];
new_input[0] = PINA; new_input[1] = PINB; new_input[2] = PINC; new_input[3] = PIND;
new_input[4] = PINE; new_input[5] = PINF; new_input[6] = PING; new_input[7] = PINH;

uint8_t changed = 0;
for (uint8_t i = 0; i < 8; i++) {
  spi_tx_buf[i] = new_input[i];
  if (new_input[i] != last_input[i]) changed = 1;
  last_input[i] = new_input[i];
}

if (changed) {
  PORTJ |= (1 << EVENT_PIN);
  new_event = 1;
}

} }

void clear_event_if_needed() { if (new_event) { PORTJ &= ~(1 << EVENT_PIN); new_event = 0; } }

void setup() { cli(); setup_event_pin(); setup_spi_slave(); sei(); }

void loop() { apply_io_logic(); clear_event_if_needed(); }

