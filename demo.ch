@x
@* Program.
DTR is used by \.{tel} to switch the phone off (on timeout and for
special commands) by switching off/on
base station for one second (the phone looses connection to base
station and automatically powers itself off).

\.{tel} uses DTR to switch on base station when it starts;
and when TTY is closed, DTR switches off base station.

The following phone model is used: Panasonic KX-TCD245.
The main requirement is that base station
must have led indicator for on-hook / off-hook on base station (to be able
to reset to initial state in state machine in \.{tel}; note, that
measuring voltage drop in phone line to determine hook state does not work
reliably, because it
falsely triggers when dtmf signal is produced ---~the dtmf signal is alternating
below the trigger level and multiple on-hook/off-hook events occur in high
succession).

Note, that relay switches off output from base station's power supply, not input
because transition processes from 220v could damage power supply because it
is switched on/off multiple times.

Also note that when device is not plugged in,
base station must be powered off, and it must be powered on by \.{tel} (this
is why non-inverted relay must be used (and from such kind of relay the
only suitable I know of is mechanical relay; and such relay gives an advantage
that power supply with AC and DC output may be used)).

%Note, that we can not use simple cordless phone---a DECT phone is needed, because
%resetting base station to put the phone on-hook will not work
%(FIXME: check if it is really so).
@y
@* Program. This is cleaned-up Atmel's demo. All functionality of original is preserved.

To test this, run `\.{stty raw </dev/ttyACM\char2}' and `\.{cat /dev/ttyACM\char2}' and
press keys `\.*', `\.0' and `\.\#' on numpad (`\.{\char2}' can be seen in
output of `\.{tail /var/log/kern.log}').
@z

@x
typedef unsigned short U16;
@y
typedef unsigned short U16;
typedef unsigned long U32;
@z

@x
@<Type \null definitions@>@;
@y
@<Predeclarations of procedures@>@;
@<Type \null definitions@>@;
@z

@x
volatile int keydetect = 0;
ISR(INT1_vect)
{
  keydetect = 1;
}

volatile int connected = 0;
void main(void)
@y
U8 endpoint_status[7];
volatile U8 usb_configuration_nb;
volatile U8 usb_request_break_generation = 0;
volatile U8 rs2usb[10];
volatile U8 cpt_sof;
U8 usb_suspended = 0;
U16 rx_counter;

void main(void)
@z

@x
  UHWCON |= 1 << UVREGE;
@y
  UHWCON |= 1 << UVREGE;
  clock_prescale_set(0);
@z

@x
  UDIEN |= 1 << EORSTE;
@y
  UDIEN |= 1 << EORSTE;
  UDIEN |= 1 << SOFE;
@z

@x
  while (!connected)
    if (UEINTX & 1 << RXSTPI)
      @<Process SETUP request@>@;
  UENUM = EP1;

  PORTD |= 1 << PD5; /* led off (before enabling output, because this led is inverted) */
  DDRD |= 1 << PD5; /* on-line/off-line indicator; also |PORTD & 1 << PD5| is used to get current
                       state to determine if transition happened (to save extra variable) */
  @<Set |PD2| to pullup mode@>@;
  EICRA |= 1 << ISC11 | 1 << ISC10; /* set INT1 to trigger on rising edge */
  EIMSK |= 1 << INT1; /* turn on INT1 */
  DDRB |= 1 << PB0; /* DTR indicator; also |PORTB & 1 << PB0| is used to get current DTR state
                       to determine if transition happened (to save extra variable) */
  DDRE |= 1 << PE6;

  if (line_status.DTR != 0) { /* are unions automatically zeroed? (may be removed if yes) */
    PORTB &= ~(1 << PB0);
    PORTD &= ~(1 << PD5);
    return;
  }
  char digit;
  while (1) {
    @<Get |line_status|@>@;
    if (line_status.DTR) {
      PORTE |= 1 << PE6; /* base station on */
      PORTB |= 1 << PB0; /* led off */
    }
    else {
      if (PORTB & 1 << PB0) { /* transition happened */
        PORTE &= ~(1 << PE6); /* base station off */
        keydetect = 0; /* in case key was detected right before base station was
                          switched off, which means that nothing must come from it */
      }
      PORTB &= ~(1 << PB0); /* led on */
    }
    @<Indicate phone line state and notify \.{tel} if state changed@>@;
    if (keydetect) {
      keydetect = 0;
      switch (PINB & (1 << PB4 | 1 << PB5 | 1 << PB6) | PIND & 1 << PD7) {
      case (0x10): digit = '1'; @+ break;
      case (0x20): digit = '2'; @+ break;
      case (0x30): digit = '3'; @+ break;
      case (0x40): digit = '4'; @+ break;
      case (0x50): digit = '5'; @+ break;
      case (0x60): digit = '6'; @+ break;
      case (0x70): digit = '7'; @+ break;
      case (0x80): digit = '8'; @+ break;
      case (0x90): digit = '9'; @+ break;
      case (0xA0): digit = '0'; @+ break;
      case (0xB0): digit = '*'; @+ break;
      case (0xC0): digit = '#'; @+ break;
      default: digit = '?';
      }
      while (!(UEINTX & 1 << TXINI)) ;
      UEINTX &= ~(1 << TXINI);
      UEDATX = digit;
      UEINTX &= ~(1 << FIFOCON);
    }
  }
}

@ For on-line indication we send `\.{@@}' character to \.{tel}---to put
it to initial state.
For off-line indication we send `\.{\%}' character to \.{tel}---to disable
power reset on base station after timeout.

TODO: insert PC817C.png

@<Indicate phone line state and notify \.{tel} if state changed@>=
if (PIND & 1 << PD2) { /* off-line */
  if (!(PORTD & 1 << PD5)) { /* transition happened */
    while (!(UEINTX & 1 << TXINI)) ;
    UEINTX &= ~(1 << TXINI);
    UEDATX = '%';
    UEINTX &= ~(1 << FIFOCON);
  }
  PORTD |= 1 << PD5;
}
else { /* on-line */
  if (PORTD & 1 << PD5) { /* transition happened */
    while (!(UEINTX & 1 << TXINI)) ;
    UEINTX &= ~(1 << TXINI);
    UEDATX = '@@';
    UEINTX &= ~(1 << FIFOCON);
  }
  PORTD &= ~(1 << PD5);
}

@ The pull-up resistor is connected to the high voltage (this is usually 3.3V or 5V and is
often refereed to as VCC).

Pull-ups are often used with buttons and switches.

With a pull-up resistor, the input pin will read a high state when the photo-transistor
is not opened. In other words, a small amount of current is flowing between VCC and the input
pin (not to ground), thus the input pin reads close to VCC. When the photo-transistor is
opened, it connects the input pin directly to ground. The current flows through the resistor
to ground, thus the input pin reads a low state.

Since pull-up resistors are so commonly needed, many MCUs, like the ATmega328 microcontroller
on the Arduino platform, have internal pull-ups that can be enabled and disabled.

TODO: insert pullup.svg

@<Set |PD2| to pullup mode@>=
PORTD |= 1 << PD2;

@ No other requests except {\caps set control line state} come
after connection is established (speed is not set in \.{tel}).

@<Get |line_status|@>=
UENUM = EP0;
if (UEINTX & 1 << RXSTPI) {
  (void) UEDATX; @+ (void) UEDATX;
  @<Handle {\caps set control line state}@>@;
}
UENUM = EP1; /* restore */

@ @<Type \null definitions@>=
typedef union {
  U16 all;
  struct {
    U16 DTR:1;
    U16 RTS:1;
    U16 unused:14;
  };
} S_line_status;

@ @<Global variables@>=
S_line_status line_status;

@ This request generates RS-232/V.24 style control signals.

Only first two bits of the first byte are used. First bit indicates to DCE if DTE is
present or not. This signal corresponds to V.24 signal 108/2 and RS-232 signal DTR.
@^DTR@>
Second bit activates or deactivates carrier. This signal corresponds to V.24 signal
105 and RS-232 signal RTS\footnote*{For some reason on linux DTR and RTS signals
are tied to each other.}. Carrier control is used for half duplex modems.
The device ignores the value of this bit when operating in full duplex mode.

\S6.2.14 in CDC spec.

TODO: manage here hardware flow control (this TODO taken from original example, not
sure what it means)
@^TODO@>

@<Handle {\caps set control line state}@>=
wValue = UEDATX | UEDATX << 8;
UEINTX &= ~(1 << RXSTPI);
UEINTX &= ~(1 << TXINI); /* STATUS stage */
line_status.all = wValue;
@y
  DDRD |= 1 << PD5;
  DDRB |= 1 << PB0;
  DDRF &= ~(1 << PF4), PORTF |= 1 << PF4; /* input */
  DDRF &= ~(1 << PF5), PORTF |= 1 << PF5; /* input */
  DDRF &= ~(1 << PF6), PORTF |= 1 << PF6; /* input */
  DDRD |= 1 << PD7; /* ground */

  while (1) {
    if (!(USBSTA & 1 << VBUS)) usb_configuration_nb = 0;
    UENUM = EP0;
    if (UEINTX & 1 << RXSTPI)
      @<Process SETUP request@>@;
    if (usb_configuration_nb != 0) { /* do not allow to receive data before
                                        {\caps set configuration} */
      if (UCSR1A & 1 << UDRE1) {
        UENUM = EP2;
        if (UEINTX & 1 << RXOUTI) {
          UEINTX &= ~(1 << RXOUTI);
          rx_counter = UEBCLX | UEBCHX << 8;
          if (rx_counter == 0) PORTD |= 1 << PD5; /* this cannot happen */
          while (rx_counter) {
            while (!(UCSR1A & 1 << UDRE1)) ;
            UDR1 = UEDATX;
            rx_counter--;
            if (rx_counter == 0)
              UEINTX &= ~(1 << FIFOCON);
          }
        }
      }
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
      if (usb_request_break_generation == 1) {
        usb_request_break_generation = 0;
        PORTB ^= 1 << PB0;
        @<Reset MCU@>@;
      }
    }
  }
}

char __low_level_init(void) __attribute__ ((section(".init3"), naked));
char __low_level_init()
{
  clock_prescale_set(0);
  return 1;
}

@ @<Type \null definitions@>=
typedef union {
  U16 all;
  struct {
    U16 bDCD:1;
    U16 bDSR:1;
    U16 bBreak:1;
    U16 bRing:1;
    U16 bFraming:1;
    U16 bParity:1;
    U16 bOverRun:1;
    U16 reserved:9;
  };
} S_serial_state;

@ We buffer the old state in ordrer to send this interrupt message only if
|serial_state| has changed.

@<Global variables@>=
S_serial_state serial_state; // actual state
S_serial_state serial_state_saved; // buffered previously sent state

@ Check if serial state has changed and update host with that information.
Necessary for hardware handshake support.

|serial_state| is used like a real interrupt status register. Once a notification has been sent,
the device will reset and re-evaluate the different signals. For the consistent signals like
carrier detect or transmission carrier, this will mean another notification will not be generated
until there is a state change. For the irregular signals like break, the incoming ring signal,
or the overrun error state, this will reset their values to zero and again will not send another
notification until their state changes.

\S6.3.5 in CDC spec.

Here we have to wait (before or after sending notification, but
`after' is not efficient\footnote*{It is not efficient
to wait right after clearing |FIFOCON|.
We may do other things --- meanwhile the data will be transmitted.
It is only necessary to wait right before sending next data.})
if we want to make sure that notification was delivered
to host. Incidentally, on control endpoint it is never necessary
to wait |TXINI| to become `1', because on control endpoint if |RXSTPI| is `1',
|TXINI| is guaranteed to be `1'.

CDC\S6.3 (see also CDC\S3.6.2.1).

@<Notify host if |serial_state| changed@>=
if (serial_state_saved.all != serial_state.all) {
  serial_state_saved.all = serial_state.all;
  UENUM = EP3;
  while (!(UEINTX & 1 << TXINI)) ; /* wait until previous packet is sent */
  UEINTX &= ~(1 << TXINI);
  UEDATX = 0xA1;
  UEDATX = 0x20;
  UEDATX = 0x00; @+ UEDATX = 0x00;
  UEDATX = 0x00; @+ UEDATX = 0x00;
  UEDATX = 0x02; @+ UEDATX = 0x00;
  UEDATX = ((U8 *) &serial_state.all)[0];
  UEDATX = ((U8 *) &serial_state.all)[1];
  UEINTX &= ~(1 << FIFOCON);
}
@z

@x
ISR(USB_GEN_vect)
{
  UDINT &= ~(1 << EORSTI); /* for the interrupt handler to be called for next USB\_RESET */
  if (!connected) {
    UECONX |= 1 << EPEN;
    UECFG1X = 1 << EPSIZE1; /* 32 bytes\footnote\ddag{Must correspond to |EP0_SIZE|.} */
    UECFG1X |= 1 << ALLOC;
  }
  else {
    @<Reset MCU@>@;
  }
}
@y
@ TODO: |if (!line_status.DTR) discard byte|
see also usb/README.DTR

BUG: do not use |uart_usb_send_buffer| outside interrupt, because |empty_packet| is calculated
for one situation, and bank may be filled with data behind the scenes

@<Predeclarations of procedures@>=
void uart_usb_send_buffer(U8 *buffer, U8 nb_data);
@ @c
void uart_usb_send_buffer(U8 *buffer, U8 nb_data)
{
  U8 empty_packet = 0;

  if (nb_data % EP0_SIZE == 0)
    empty_packet = 1; /* indicate to the host that no more data will follow (USB\S5.8.3) */
  UENUM = EP1;
  while (nb_data) {
    while (!(UEINTX & 1 << RWAL)) ;
    while (UEINTX & 1 << RWAL && nb_data) {
      UEDATX = (U8) *buffer;
      buffer++;
      nb_data--;
    }
    UEINTX &= ~(1 << TXINI), UEINTX &= ~(1 << FIFOCON);
  }
  if (empty_packet) {
    while (!(UEINTX & 1 << RWAL)) ;
    UEINTX &= ~(1 << TXINI), UEINTX &= ~(1 << FIFOCON);
  }
}

ISR(USB_GEN_vect)
{
  if (UDINT & 1 << SOFI) { /* was it |SOFI| that caused this interrupt handler to be called? */
    UDINT &= ~(1 << SOFI); /* for the interrupt handler to be called when
                                next USB ``Start Of Frame'' PID will be detected
                                and avoid misdetecting an event
                                that will cause this interrupt handler to be called */
    cpt_sof++;
  }
  if (UDINT & 1 << EORSTI) {
    UDINT &= ~(1 << EORSTI);
    UENUM = EP0;
    UECONX |= 1 << EPEN;
    UECFG1X = 1 << EPSIZE1; /* 32 bytes\footnote\ddag{Must correspond to |EP0_SIZE|.} */
    UECFG1X |= 1 << ALLOC;
    UERST = 1 << EP0, UERST = 0; /* FIXME: is it necessary? */
    usb_configuration_nb = 0;
  }
}

@ TODO: to be able to use rx and tx in parallel, just put byte to buffer
here and in main loop alternate between rx and tx byte by byte;
for now, make via a simple buffer and use a led to indicate if it is full,
and later do via ring buffer
@^TODO@>

TODO: send bank if timer expired and restart timer (check timer in the same
loop where you alternate between rx and tx as said in above TODO)
see avr/C.c how to use timer

@c
ISR(USART1_RX_vect)
{
  U8 i = 0;
  U8 save_ep;

  if (usb_configuration_nb != 0) {
    save_ep = UENUM;
    UENUM = EP1;
    do {
      if (UCSR1A & 1 << RXC1) { // if there is unread data in the receive buffer
        rs2usb[i] = UDR1;
        i++;
      }
    } while (!(UEINTX & 1 << RWAL)); // while write not allowed
    uart_usb_send_buffer((U8 *) &rs2usb, i);
    UENUM = (U8) save_ep;
  }
}
@z

@x
@<Process SETUP request@>=
switch (UEDATX | UEDATX << 8) {
@y
@<Process SETUP request@>=
switch (UEDATX | UEDATX << 8) {
case 0x0080: @/
  @<Handle {\caps get status device}@>@;
  break;
case 0x0081: @/
  @<Handle {\caps get status interface}@>@;
  break;
case 0x0082: @/
  @<Handle {\caps get status endpoint}@>@;
  break;
case 0x0100: @/
  @<Handle {\caps clear feature device}@>@;
  break;
case 0x0101: @/
  @<Handle {\caps clear feature interface}@>@;
  break;
case 0x0102: @/
  @<Handle {\caps clear feature endpoint}@>@;
  break;
case 0x0300: @/
  @<Handle {\caps set feature device}@>@;
  break;
case 0x0301: @/
  @<Handle {\caps set feature interface}@>@;
  break;
case 0x0302: @/
  @<Handle {\caps set feature endpoint}@>@;
  break;
@z

@x
    @<Handle {\caps get descriptor device}\null@>@;
    break;
  case 0x0200: @/
    @<Handle {\caps get descriptor configuration}@>@;
    break;
  case 0x0300: @/
    @<Handle {\caps get descriptor string} (language)@>@;
    break;
  case 0x03 << 8 | MANUFACTURER: @/
    @<Handle {\caps get descriptor string} (manufacturer)@>@;
    break;
  case 0x03 << 8 | PRODUCT: @/
    @<Handle {\caps get descriptor string} (product)@>@;
    break;
  case 0x03 << 8 | SERIAL_NUMBER: @/
    @<Handle {\caps get descriptor string} (serial)@>@;
    break;
  case 0x0600: @/
    @<Handle {\caps get descriptor device qualifier}@>@;
    break;
@y
    @<Handle {\caps get descriptor device}@>@;
    break;
  case 0x0200: @/
    @<Handle {\caps get descriptor configuration}@>@;
    break;
  default: /* in code derived from this example change this to ``Handle
              {\caps get descriptor device qualifier}'' */
    UECONX |= 1 << STALLRQ; /* prepare to send STALL handshake in response to next token from
      host (USB\S8.5.3.4) */
    UEINTX &= ~(1 << RXSTPI);
@z

@x
case 0x0900: @/
  @<Handle {\caps set configuration}@>@;
  break;
@y
case 0x0880: @/
  @<Handle {\caps get configuration}@>@;
  break;
case 0x0900: @/
  @<Handle {\caps set configuration}@>@;
  break;
case 0x0A81: @/
  @<Handle {\caps get interface}@>@;
  break;
case 0x0B01: @/
  @<Handle {\caps set interface}@>@;
  break;
@z

@x
case 0x2021: @/
  @<Handle {\caps set line coding}@>@;
  break;
@y
case 0x2021: @/
  @<Handle {\caps set line coding}@>@;
  break;
case 0x21A1: @/
  @<Handle {\caps get line coding}@>@;
  break;
case 0x2221: @/
  @<Handle {\caps set control line state}@>@;
  break;
case 0x2321: @/
  @<Handle {\caps send break}@>@;
  break;
default: /* in code derived from this example remove this and all unused "Handle" sections */
  UECONX |= 1 << STALLRQ; /* prepare to send STALL handshake in response to next token from host
    (USB\S8.5.3.4) */
  UEINTX &= ~(1 << RXSTPI);
@z

@x
@<Handle {\caps get descriptor device}\null@>=
@y
@<Handle {\caps get descriptor device}@>=
@z

@x
@ A high-speed capable device that has different device information for full-speed and high-speed
must have a Device Qualifier Descriptor. For example, if the device is currently operating at
full-speed, the Device Qualifier returns information about how it would operate at high-speed and
vice-versa. So as this device is full-speed, it tells the host not to request
device information for high-speed by using ``protocol stall'' (such stall
does not indicate an error with the device ---~it serves as a means of
extending USB requests).

The host sends an IN token to the control pipe to initiate the DATA stage.

$$\hbox to10.93cm{\vbox to5.15055555555556cm{\vfil\special{%
  psfile=../usb/stall-control-read-with-data-stage.eps
  clip llx=0 lly=0 urx=310 ury=146 rwi=3100}}\hfil}$$

Note, that next token comes after \.{RXSTPI} is cleared, so we set \.{STALLRQ} before
clearing \.{RXSTPI}, to make sure that \.{STALLRQ} is already set when next token arrives.

This STALL condition is automatically cleared on the receipt of the
next SETUP token.

USB\S8.5.3.4, datasheet\S22.11.

@<Handle {\caps get descriptor device qualifier}@>=
UECONX |= 1 << STALLRQ; /* prepare to send STALL handshake in response to IN token of the DATA
  stage */
UEINTX &= ~(1 << RXSTPI);
@y
@z

@x
@ @<Handle {\caps get descriptor string} (language)@>=
(void) UEDATX; @+ (void) UEDATX;
wLength = UEDATX | UEDATX << 8;
UEINTX &= ~(1 << RXSTPI);
size = sizeof lang_desc;
buf = lang_desc;
@<Send descriptor@>@;

@ @<Handle {\caps get descriptor string} (manufacturer)@>=
(void) UEDATX; @+ (void) UEDATX;
wLength = UEDATX | UEDATX << 8;
UEINTX &= ~(1 << RXSTPI);
size = pgm_read_byte(&mfr_desc.bLength);
buf = &mfr_desc;
@<Send descriptor@>@;

@ @<Handle {\caps get descriptor string} (product)@>=
(void) UEDATX; @+ (void) UEDATX;
wLength = UEDATX | UEDATX << 8;
UEINTX &= ~(1 << RXSTPI);
size = pgm_read_byte(&prod_desc.bLength);
buf = &prod_desc;
@<Send descriptor@>@;

@ Here we handle one case when data (serial number) needs to be transmitted from memory,
not from program.

@<Handle {\caps get descriptor string} (serial)@>=
(void) UEDATX; @+ (void) UEDATX;
wLength = UEDATX | UEDATX << 8;
UEINTX &= ~(1 << RXSTPI);
size = 1 + 1 + SN_LENGTH * 2; /* multiply because Unicode */
@<Get serial number@>@;
buf = &sn_desc;
from_program = 0;
@<Send descriptor@>@;

@ @<Handle {\caps set configuration}@>=
UEINTX &= ~(1 << RXSTPI);

UENUM = EP3;
UECONX |= 1 << EPEN;
UECFG0X = 1 << EPTYPE1 | 1 << EPTYPE0 | 1 << EPDIR; /* interrupt\footnote\dag{Must
  correspond to |@<Initialize element 6 ...@>|.}, IN */
UECFG1X = 1 << EPSIZE1; /* 32 bytes\footnote\ddag{Must
  correspond to |@<Initialize element 6 ...@>|.} */
UECFG1X |= 1 << ALLOC;

UENUM = EP1;
UECONX |= 1 << EPEN;
UECFG0X = 1 << EPTYPE1 | 1 << EPDIR; /* bulk\footnote\dag{Must
  correspond to |@<Initialize element 8 ...@>|.}, IN */
UECFG1X = 1 << EPSIZE1; /* 32 bytes\footnote\ddag{Must
  correspond to |@<Initialize element 8 ...@>|.} */
UECFG1X |= 1 << ALLOC;

UENUM = EP2;
UECONX |= 1 << EPEN;
UECFG0X = 1 << EPTYPE1; /* bulk\footnote\dag{Must
  correspond to |@<Initialize element 9 ...@>|.}, OUT */
UECFG1X = 1 << EPSIZE1; /* 32 bytes\footnote\ddag{Must
  correspond to |@<Initialize element 9 ...@>|.} */
UECFG1X |= 1 << ALLOC;

UENUM = EP0; /* restore for further setup requests */
UEINTX &= ~(1 << TXINI); /* STATUS stage */
@y
@ The host sends an IN token to the control pipe to initiate the STATUS stage.

$$\hbox to12.06cm{\vbox to4.26861111111111cm{\vfil\special{%
  psfile=stall-control-write-without-data-stage.eps
  clip llx=0 lly=0 urx=342 ury=121 rwi=3420}}\hfil}$$

Note, that next token comes after \.{RXSTPI} is cleared, so we set \.{STALLRQ} before
clearing \.{RXSTPI}, to make sure that \.{STALLRQ} is already set when next token arrives.

Also note, that such STALL (which is called ``protocol stall'') does not indicate an error with
the device ---~it serves as a means of extending USB requests.

This STALL condition is automatically cleared on the receipt of the
next SETUP token.

USB\S8.5.3.4, datasheet\S22.11.

\xdef\stallinstatus{\secno}

@<Handle {\caps set configuration}@>=
wValue = UEDATX | UEDATX << 8;
if (wValue <= 1) { /* FIXME: this is nonsense, because host cannot send configuration
    which was not specified in |S_configuration_descriptor|, and because host cannot
    set configuration with zero value (?) */
  UEINTX &= ~(1 << RXSTPI);
  usb_configuration_nb = wValue & 0xFF;

  UENUM = EP3;
  UECONX |= 1 << EPEN;
  UECFG0X = 1 << EPTYPE1 | 1 << EPTYPE0 | 1 << EPDIR; /* interrupt\footnote\dag{Must
    correspond to |@<Initialize element 6 ...@>|.}, IN */
  UECFG1X = 1 << EPSIZE1; /* 32 bytes\footnote\ddag{Must
    correspond to |@<Initialize element 6 ...@>|.} */
  UECFG1X |= 1 << ALLOC;

  UENUM = EP1;
  UECONX |= 1 << EPEN;
  UECFG0X = 1 << EPTYPE1 | 1 << EPDIR; /* bulk\footnote\dag{Must
    correspond to |@<Initialize element 8 ...@>|.}, IN */
  UECFG1X = 1 << EPSIZE1; /* 32 bytes\footnote\ddag{Must
    correspond to |@<Initialize element 8 ...@>|.} */
  UECFG1X |= 1 << ALLOC;

  UENUM = EP2;
  UECONX |= 1 << EPEN;
  UECFG0X = 1 << EPTYPE1; /* bulk\footnote\dag{Must
    correspond to |@<Initialize element 9 ...@>|.}, OUT */
  UECFG1X = 1 << EPSIZE1; /* 32 bytes\footnote\ddag{Must
    correspond to |@<Initialize element 9 ...@>|.} */
  UECFG1X |= 1 << ALLOC;

  UERST = 1 << EP3, UERST = 0; /* FIXME: why is it done? */
  UERST = 1 << EP1, UERST = 0;
  UERST = 1 << EP2, UERST = 0;

  UENUM = EP0; /* restore for further setup requests */
  UEINTX &= ~(1 << TXINI); /* STATUS stage */
}
else {
  UECONX |= 1 << STALLRQ; /* prepare to send STALL handshake in response to IN token of STATUS
    stage */
  UEINTX &= ~(1 << RXSTPI);
}
@z

@x
@ Just discard the data.
This is the last request after attachment to host.

@<Handle {\caps set line coding}@>=
UEINTX &= ~(1 << RXSTPI);
while (!(UEINTX & 1 << RXOUTI)) ; /* wait for DATA stage */
UEINTX &= ~(1 << RXOUTI);
UEINTX &= ~(1 << TXINI); /* STATUS stage */
connected = 1;
@y
@ @<Type \null definitions@>=
typedef struct {
  U32 dwDTERate;
  U8 bCharFormat;
  U8 bParityType;
  U8 bDataBits;
} S_line_coding;

@ @<Global variables@>=
S_line_coding line_coding;

@ This request allows the host to specify typical asynchronous line-character formatting
properties.

\S6.2.12 in CDC spec.

Note, that 32bit is (LSW,MSW) or (LSB,...,MSB).

@<Handle {\caps set line coding}@>=
UEINTX &= ~(1 << RXSTPI);
while (!(UEINTX & 1 << RXOUTI)) ; /* wait for DATA stage */
((U8 *) &line_coding.dwDTERate)[0] = UEDATX;
((U8 *) &line_coding.dwDTERate)[1] = UEDATX;
((U8 *) &line_coding.dwDTERate)[2] = UEDATX;
((U8 *) &line_coding.dwDTERate)[3] = UEDATX;
line_coding.bCharFormat = UEDATX;
line_coding.bParityType = UEDATX;
line_coding.bDataBits = UEDATX;
UEINTX &= ~(1 << RXOUTI);
UEINTX &= ~(1 << TXINI); /* STATUS stage */
@<Configure UART@>@;

@ This will send two bytes during the DATA stage. Only first two bits of the first byte are used.
If first bit is set, then this indicates the device is self powered. If clear, the device
is bus powered. If second bit is set, the device has remote wakeup enabled and can wake the
host up during suspend. The remote wakeup bit can be by the {\caps set feature} and
{\caps clear feature} requests with a feature selector of |0x01| (\.{DEVICE\_REMOTE\_WAKEUP}).

@<Handle {\caps get status device}@>=
UEINTX &= ~(1 << RXSTPI);
UEDATX = 0x01;
UEDATX = 0x00;
UEINTX &= ~(1 << TXINI); /* DATA stage */
while (!(UEINTX & 1 << RXOUTI)) ; /* wait for STATUS stage */
UEINTX &= ~(1 << RXOUTI);

@ Sends two bytes of |0x00|, |0x00|. (Both bytes are reserved for future use.)

@<Handle {\caps get status interface}@>=
UEINTX &= ~(1 << RXSTPI);
UEDATX = 0x00;
UEDATX = 0x00;
UEINTX &= ~(1 << TXINI); /* DATA stage */
while (!(UEINTX & 1 << RXOUTI)) ; /* wait for STATUS stage */
UEINTX &= ~(1 << RXOUTI);

@ Sends two bytes indicating the status (Halted/Stalled) of an endpoint.
Only first bit of the first byte is used.

@<Handle {\caps get status endpoint}@>=
(void) UEDATX; @+ (void) UEDATX;
wIndex = UEDATX | UEDATX << 8;
UEINTX &= ~(1 << RXSTPI);
UEDATX = endpoint_status[wIndex & 0x7F];
UEDATX = 0x00;
UEINTX &= ~(1 << TXINI); /* DATA stage */
while (!(UEINTX & 1 << RXOUTI)) ; /* wait for STATUS stage */
UEINTX &= ~(1 << RXOUTI);

@ This gets the Alternative Interface setting.

@<Handle {\caps get interface}@>=
UEINTX &= ~(1 << RXSTPI);
UEDATX = 0x00;
UEINTX &= ~(1 << TXINI); /* DATA stage */
while (!(UEINTX & 1 << RXOUTI)) ; /* wait for STATUS stage */
UEINTX &= ~(1 << RXOUTI);

@ Used to set boolean features. The only two feature selectors available are
\.{DEVICE\_REMOTE\_WAKEUP} and \.{TEST\_MODE}.

@<Handle {\caps set feature device}@>=
UECONX |= 1 << STALLRQ; /* see \S\stallinstatus\ */
UEINTX &= ~(1 << RXSTPI);

@ @<Handle {\caps clear feature device}@>=
UECONX |= 1 << STALLRQ; /* see \S\stallinstatus\ */
UEINTX &= ~(1 << RXSTPI);

@ Used to set boolean features. The current USB Specification Revision 2 specifies no
interface features.

@<Handle {\caps set feature interface}@>=
UECONX |= 1 << STALLRQ; /* see \S\stallinstatus\ */
UEINTX &= ~(1 << RXSTPI);

@ @<Handle {\caps clear feature interface}@>=
UECONX |= 1 << STALLRQ; /* see \S\stallinstatus\ */
UEINTX &= ~(1 << RXSTPI);

@ Used to set endpoint features. The standard currently defines one endpoint feature
selector |0x00|, which allows the host to stall and clear an endpoint.
Only endpoints other than the default endpoint are recommended to have this functionality.

@<Handle {\caps set feature endpoint}@>=
wValue = UEDATX | UEDATX << 8;
if (wValue == 0) {
  wIndex = UEDATX | UEDATX << 8;
  if (wIndex == 0) {
    UECONX |= 1 << STALLRQ; /* see \S\stallinstatus\ */
    UEINTX &= ~(1 << RXSTPI);
  }
  UENUM = (U8) wIndex;
  if (UECONX & 1 << EPEN) {
    UECONX |= 1 << STALLRQ; /* TODO: determine if it is a ``functional stall'' or
      ``commanded stall'' and compare this code with USB\S8.4 or USB\S9 respectively
      and see datasheet\S22.11 */
    endpoint_status[wIndex & 0x7F] = 0x01;
    UENUM = EP0;
    UEINTX &= ~(1 << RXSTPI);
    UEINTX &= ~(1 << TXINI); /* STATUS stage */
  }
  else {
    UENUM = EP0;
    UECONX |= 1 << STALLRQ; /* see \S\stallinstatus\ */
    UEINTX &= ~(1 << RXSTPI);
  }
}
else {
  UECONX |= 1 << STALLRQ; /* see \S\stallinstatus\ */
  UEINTX &= ~(1 << RXSTPI);
}

@ @<Handle {\caps clear feature endpoint}@>=
wValue = UEDATX | UEDATX << 8;
if (wValue == 0) {
  wIndex = UEDATX | UEDATX << 8;
  UENUM = (U8) wIndex;
  if (UECONX & 1 << EPEN) {
    if (wIndex != 0) {
      UECONX |= 1 << STALLRQC; /* TODO: determine if it is a ``functional stall'' or
      ``commanded stall'' and compare this code with USB\S8.4 or USB\S9 respectively
      and see datasheet\S22.11 */
      UERST = 1 << (U8) wIndex, UERST = 0;
      UECONX |= 1 << RSTDT;
    }
    endpoint_status[wIndex & 0x7F] = 0x00;
    UENUM = EP0;
    UEINTX &= ~(1 << RXSTPI);
    UEINTX &= ~(1 << TXINI); /* STATUS stage */
  }
  else {
    UENUM = EP0;
    UECONX |= 1 << STALLRQ; /* see \S\stallinstatus\ */
    UEINTX &= ~(1 << RXSTPI);
  }
}
else {
  UECONX |= 1 << STALLRQ; /* see \S\stallinstatus\ */
  UEINTX &= ~(1 << RXSTPI);
}

@ Used to request the current device configuration. A byte will be sent during the DATA stage
indicating the devices status. A zero value means the device is not configured and a non-zero
value indicates the device is configured.

@<Handle {\caps get configuration}@>=
UEINTX &= ~(1 << RXSTPI);
UEDATX = usb_configuration_nb;
UEINTX &= ~(1 << TXINI); /* DATA stage */
while (!(UEINTX & 1 << RXOUTI)) ; /* wait for STATUS stage */
UEINTX &= ~(1 << RXOUTI);

@ This request allows the host to find out the currently configured line coding.

\S6.2.13 in CDC spec.

@<Handle {\caps get line coding}@>=
UEINTX &= ~(1 << RXSTPI);
UEDATX = ((U8 *) &line_coding.dwDTERate)[0];
UEDATX = ((U8 *) &line_coding.dwDTERate)[1];
UEDATX = ((U8 *) &line_coding.dwDTERate)[2];
UEDATX = ((U8 *) &line_coding.dwDTERate)[3];
UEDATX = line_coding.bCharFormat;
UEDATX = line_coding.bParityType;
UEDATX = line_coding.bDataBits;
UEINTX &= ~(1 << TXINI); /* DATA stage */
while (!(UEINTX & 1 << RXOUTI)) ; /* wait for STATUS stage */
UEINTX &= ~(1 << RXOUTI);

@ This request sends special carrier modulation that generates an RS-232 style break.

\S6.2.15 in CDC spec.

TODO: manage here hardware flow control (this TODO taken from original example, not
sure what it means)
@^TODO@>

@<Handle {\caps send break}@>=
(void) UEDATX; @+ (void) UEDATX; /* break length */
UEINTX &= ~(1 << RXSTPI);
UEINTX &= ~(1 << TXINI);
usb_request_break_generation = 1;

@ @<Type \null definitions@>=
typedef union {
  U16 all;
  struct {
    U16 DTR:1;
    U16 RTS:1;
    U16 unused:14;
  };
} S_line_status;

@ @<Global variables@>=
S_line_status line_status;

@ This request generates RS-232/V.24 style control signals.

Only first two bits of the first byte are used. First bit indicates to DCE if DTE is
present or not. This signal corresponds to V.24 signal 108/2 and RS-232 signal DTR.
@^DTR@>
Second bit activates or deactivates carrier. This signal corresponds to V.24 signal
105 and RS-232 signal RTS\footnote*{For some reason on linux DTR and RTS signals
are tied to each other.}. Carrier control is used for half duplex modems.
The device ignores the value of this bit when operating in full duplex mode.

\S6.2.14 in CDC spec.

TODO: manage here hardware flow control (this TODO taken from original example, not
sure what it means)
@^TODO@>

@<Handle {\caps set control line state}@>=
wValue = UEDATX | UEDATX << 8;
UEINTX &= ~(1 << RXSTPI);
UEINTX &= ~(1 << TXINI); /* STATUS stage */
line_status.all = wValue;

@ This request allows the host to select an alternate setting for the specified interface.

\S9.4.10 in USB spec.

@<Handle {\caps set interface}@>=
UEINTX &= ~(1 << RXSTPI);
UEINTX &= ~(1 << TXINI); /* STATUS stage */
@z

@x
U8 from_program = 1; /* serial number is transmitted last, so this can be set only once */
@y
@z

@x
    UEDATX = from_program ? pgm_read_byte(buf++) : *(U8 *) buf++;
@y
    UEDATX = pgm_read_byte(buf++);
@z

@x
@i ../usb/CONTROL-endpoint-management.w

@i ../usb/IN-endpoint-management.w

@* USB stack.
@y
@ In application we set baud only once, before setting |DTR|;
if you change baud in middle of application, think if something
should be considered here.

Note, that 115200 baud is not supported (due to 16MHz).

@<Configure UART@>=
UCSR1B &= ~(1 << RXCIE1); /* disable interrupt first */
UCSR1B &= ~(1 << RXEN1); /* disable receiver to flush receive buffer */

switch (line_coding.dwDTERate) {
case 9600: @/
  UBRR1 = 103;
  UCSR1A &= ~(1 << U2X1);
  UCSR1B |= 1 << RXEN1 | 1 << TXEN1 | 1 << RXCIE1;
  break;
case 57600: @/
  UBRR1 = 34;
  UCSR1A |= 1 << U2X1;
  UCSR1B |= 1 << RXEN1 | 1 << TXEN1 | 1 << RXCIE1;
  break;
default:
  UCSR1B &= ~(1 << TXEN1);
}

@* USB stack.
@z

@x
@d MANUFACTURER 1
@d PRODUCT 2
@d SERIAL_NUMBER 3
@y
@z

@x
  MANUFACTURER, /* (\.{Mfr} in \.{kern.log}) */
  PRODUCT, /* (\.{Product} in \.{kern.log}) */
  SERIAL_NUMBER, /* (\.{SerialNumber} in \.{kern.log}) */
@y
  0, /* set it to non-zero in code derived from this example */
  0, /* set it to non-zero in code derived from this example */
  0, /* set it to non-zero in code derived from this example */
@z

@x
  @<Class-specific interface descriptor 2@> @,@,@! el5;
  @<Class-specific interface descriptor 3@> @,@,@! el6;
@y
  @<Class-specific interface descriptor 2@> @,@,@! el4;
  @<Class-specific interface descriptor 3@> @,@,@! el5;
  @<Class-specific interface descriptor 4@> @,@,@! el6;
@z

@x
@t\2@> @<Initialize element 9 ...@> @/
@y
  @<Initialize element 9 ...@>, @/
@t\2@> @<Initialize element \null 10 ...@> @/
@z

@x
@ @<Initialize element 7 in configuration descriptor@>= { @t\1@> @/
@y
@ @<Initialize element 8 in configuration descriptor@>= { @t\1@> @/
@z

@x
@<Initialize element 6 in configuration descriptor@>= { @t\1@> @/
@y
@<Initialize element 7 in configuration descriptor@>= { @t\1@> @/
@z

@x
@ @<Initialize element 8 in configuration descriptor@>= { @t\1@> @/
@y
@ @<Initialize element 9 in configuration descriptor@>= { @t\1@> @/
@z

@x
@<Initialize element 9 in configuration descriptor@>= { @t\1@> @/
@y
@<Initialize element \null 10 in configuration descriptor@>= { @t\1@> @/
@z

@x
@*3 Abstract control management functional descriptor.
@y
@*3 Call management functional descriptor.

The Call Management functional descriptor describes
the processing of calls for the Communication Class interface.

\S5.2.3.2 in CDC spec.

@<Class-specific interface descriptor 2@>=
struct {
  U8 bFunctionLength;
  U8 bDescriptorType;
  U8 bDescriptorSubtype;
  U8 bmCapabilities;
  U8 bDataInterface;
}

@ |bmCapabilities|:
Only first two bits are used.
If first bit is set, then this indicates the device handles call
management itself. If clear, the device
does not handle call management itself. If second bit is set,
the device can send/receive call management information over a
Data Class interface. If clear, the device sends/receives call
management information only over the Communication Class
interface. The previous bits, in combination, identify
which call management scenario is used. If first bit
is reset to 0, then the value of second bit is
ignored. In this case, second bit is reset to zero.

@<Initialize element 4 in configuration descriptor@>= { @t\1@> @/
  5, /* size of this structure */
  0x24, /* interface */
  0x01, /* call management */
  1 << 1 | 1, @/
@t\2@> 1 /* number of CDC data interface */
}

@*3 Abstract control management functional descriptor.
@z

@x
@<Class-specific interface descriptor 2@>=
@y
@<Class-specific interface descriptor 3@>=
@z

@x
@<Initialize element 4 in configuration descriptor@>= { @t\1@> @/
@y
@<Initialize element 5 in configuration descriptor@>= { @t\1@> @/
@z

@x
@<Class-specific interface descriptor 3@>=
@y
@<Class-specific interface descriptor 4@>=
@z

@x
@<Initialize element 5 in configuration descriptor@>= { @t\1@> @/
@y
@<Initialize element 6 in configuration descriptor@>= { @t\1@> @/
@z

@x
#include <avr/boot.h> /* |boot_signature_byte_get| */
@y
#include <avr/power.h> /* |clock_prescale_set| */
@z

