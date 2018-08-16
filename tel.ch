Make it work with tel.w + invert leds.

@x
  DDRF &= ~(1 << PF4), PORTF |= 1 << PF4; /* input */
  DDRF &= ~(1 << PF5), PORTF |= 1 << PF5; /* input */
  DDRF &= ~(1 << PF6), PORTF |= 1 << PF6; /* input */
  DDRD |= 1 << PD7; /* ground */
@y
  PORTD |= 1 << PD5; /* led off */
  DDRE |= 1 << PE6;
  PORTE |= 1 << PE6; /* |DTR| pin high */
@z

@x
            PORTD |= 1 << PD5; /* check if this ever happens */
@y
            PORTD &= ~(1 << PD5); /* check if this ever happens */
@z

@x
      if (cpt_sof >= 100) { /* debounce (FIXME: how is this even supposed to work?) */
        unsigned char data;
        if (!(PINF & 1 << PF4)) {
          data = '*'; @+ uart_usb_send_buffer(&data, 1);
          serial_state.bDCD = 1;
        }
        else
          serial_state.bDCD = 0;
        if (!(PINF & 1 << PF5)) {
          data = '0'; @+ uart_usb_send_buffer(&data, 1);
        }
        if (!(PINF & 1 << PF6)) {
          data = '#'; @+ uart_usb_send_buffer(&data, 1);
          serial_state.bDSR = 1;
        }
        else
          serial_state.bDSR = 0;
        @<Notify host if |serial_state| changed@>@;
      }
@y
@z

@x
        PORTB ^= 1 << PB0;
@y
@z

@x
  line_status.all = UEDATX | UEDATX << 8;
@y
line_status.all = UEDATX | UEDATX << 8;
if (line_status.DTR) {
  PORTE &= ~(1 << PE6); /* |DTR| pin low */
  PORTB |= 1 << PB0; /* led off */
}
else {
  PORTE |= 1 << PE6; /* |DTR| pin high */
  PORTB &= ~(1 << PB0); /* led on */
}
@z