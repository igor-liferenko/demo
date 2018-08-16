Invert leds.

@x
  DDRD |= 1 << PD5;
  DDRB |= 1 << PB0;
@y
  DDRD |= 1 << PD5, PORTD |= 1 << PD5;
  DDRB |= 1 << PB0, PORTB |= 1 << PB0;
@z

@x
            PORTD |= 1 << PD5; /* check if this ever happens */
@y
            PORTD &= ~(1 << PD5); /* check if this ever happens */
@z
