Invert leds.

@x
  DDRD |= 1 << PD5;
  DDRB |= 1 << PB0;
@y
  PORTD |= 1 << PD5, DDRD |= 1 << PD5;
  PORTB |= 1 << PB0, DDRB |= 1 << PB0;
@z

@x
          if (rx_counter == 0) PORTD |= 1 << PD5; /* this cannot happen */
@y
          if (rx_counter == 0) PORTD &= ~(1 << PD5); /* this cannot happen */
@z
