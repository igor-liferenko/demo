\let\lheader\rheader
%\datethis
\secpagedepth=2 % begin new page only on *
\font\caps=cmcsc10 at 9pt

@* Program. This is cleaned-up Atmel's demo. All functionality of original is preserved.

@c
@<Header files@>@;

typedef unsigned char U8;
typedef unsigned short U16;
typedef unsigned long U32;
typedef unsigned char Bool;
typedef unsigned char Uchar;

#define EVT_USB_POWERED               1         // USB plugged
#define EVT_USB_UNPOWERED             2         // USB un-plugged
#define EVT_USB_SUSPEND               5
#define EVT_USB_WAKE_UP               6
#define EVT_USB_RESUME                7
#define EVT_USB_RESET                 8

@<Predeclarations of procedures@>@;
@<Type \null definitions@>@;

typedef union {
  U8 all;
  struct {
    U8 DTR:1;
    U8 RTS:1;
    U8 unused:6;
  };
} S_line_status;

typedef struct {
  U32 dwDTERate;
  U8 bCharFormat;
  U8 bParityType;
  U8 bDataBits;
} S_line_coding;

@* USB stack.

@*1 Device descriptor.

TODO: find what prefixes mean in names of variables (i.e., `b', `bcd', ...)

@<Type \null definitions@>=
typedef struct {
  uint8_t      bLength;
  uint8_t      bDescriptorType;
  uint16_t     bcdUSB; /* version */
  uint8_t      bDeviceClass; /* class code assigned by the USB */
  uint8_t      bDeviceSubClass; /* sub-class code assigned by the USB */
  uint8_t      bDeviceProtocol; /* protocol code assigned by the USB */
  uint8_t      bMaxPacketSize0; /* max packet size for EP0 */
  uint16_t     idVendor;
  uint16_t     idProduct;
  uint16_t     bcdDevice; /* device release number */
  uint8_t      iManufacturer; /* index of manu. string descriptor */
  uint8_t      iProduct; /* index of prod. string descriptor */
  uint8_t      iSerialNumber; /* index of S.N. string descriptor */
  uint8_t      bNumConfigurations;
} S_device_descriptor;

@ @<Global variables@>=
const S_device_descriptor dev_desc
@t\hskip2.5pt@> @=PROGMEM@> = { @t\1@> @/
  sizeof (S_device_descriptor), @/
  0x01, /* device */
  0x0200, /* USB 2.0 */
  0x02, /* CDC (\S4.1 in CDC spec) */
  0, /* no subclass */
  0, @/
  EP0_SIZE, @/
  0x03EB, /* VID (Atmel) */
  0x2018, /* PID (CDC ACM) */
  0x1000, /* device revision */
  0x00, /* set it to non-zero in code derived from this example */
  0x00, /* set it to non-zero in code derived from this example */
  0x00, /* set it to non-zero in code derived from this example */
@t\2@> 1 /* one configuration for this device */
};

@*1 USB configuration descriptor.

Abstract Control Model consists of two interfaces: Data Class interface
and Communication Class interface.

The Communication Class interface uses two endpoints, one to implement
a notification element and theh other to implement
a management element. The management element uses the default endpoint
for all standard and Communication Class-specific requests.

Theh Data Class interface consists of two endpoints to implement
channels over which to carry data.

$$\hbox to7.5cm{\vbox to7.88cm{\vfil\special{psfile=cdc-structure.eps
  clip llx=0 lly=0 urx=274 ury=288 rwi=2125}}\hfil}$$

@<Type \null definitions@>=
@<Type definitions used in USB configuration descriptor@>@;
typedef struct {
  S_configuration_descriptor el1;
  S_interface_descriptor el2;
  S_cdc_descriptor el3;
  S_call_management_descriptor el4;
  S_acm_descriptor el5;
  S_union_descriptor el6;
  S_endpoint_descriptor el7;
  S_interface_descriptor el8;
  S_endpoint_descriptor el9;
  S_endpoint_descriptor el10;
} S_usb_configuration_descriptor;

@ @<Global variables@>=
const S_usb_configuration_descriptor usb_conf_desc
@t\hskip2.5pt@> @=PROGMEM@> = { @t\1@> @/
  @<Initialize element 1 ...@>, @/
  @<Initialize element 2 ...@>, @/
  @<Initialize element 3 ...@>, @/
  @<Initialize element 4 ...@>, @/
  @<Initialize element 5 ...@>, @/
  @<Initialize element 6 ...@>, @/
  @<Initialize element 7 ...@>, @/
  @<Initialize element 8 ...@>, @/
  @<Initialize element 9 ...@>, @/
@t\2@> @<Initialize element 10 ...@> @/
};

@*2 Configuration descriptor.

@s S_configuration_descriptor int

@<Type definitions ...@>=
typedef struct {
   uint8_t      bLength;
   uint8_t      bDescriptorType;
   uint16_t     wTotalLength;
   uint8_t      bNumInterfaces;
   uint8_t      bConfigurationValue; /* number between 1 and |bNumConfigurations|, for
     each configuration\footnote\dag{For some reason
     configurations start numbering with `1', and interfaces and altsettings with `0'.} */
   uint8_t      iConfiguration; /* index of string descriptor */
   uint8_t      bmAttibutes;
   uint8_t      MaxPower;
} S_configuration_descriptor;

@ @<Initialize element 1 in USB configuration descriptor@>= { @t\1@> @/
  sizeof (S_configuration_descriptor), @/
  0x02, /* configuration descriptor */
  sizeof (S_usb_configuration_descriptor), @/
  2, /* two interfaces in this configuration */
  1, /* this corresponds to `1' in `cfg1' on picture */
  0, /* no string descriptor */
  0x80, /* device is powered from bus */
@t\2@> 0x32 /* device uses 100mA */
}

@*2 Interface descriptor.

@s S_interface_descriptor int

@<Type definitions ...@>=
typedef struct {
   uint8_t      bLength;
   uint8_t      bDescriptorType;
   uint8_t      bInterfaceNumber; /* number between 0 and |bNumInterfaces-1|, for
                                     each interface */
   uint8_t      bAlternativeSetting; /* number starting from 0, for each interface */
   uint8_t      bNumEndpoints; /* number of EP except EP 0 */
   uint8_t      bInterfaceClass; /* class code assigned by the USB */
   uint8_t      bInterfaceSubClass; /* sub-class code assigned by the USB */
   uint8_t      bInterfaceProtocol; /* protocol code assigned by the USB */
   uint8_t      iInterface; /* index of string descriptor */
}  S_interface_descriptor;

@ @<Initialize element 2 in USB configuration descriptor@>= { @t\1@> @/
  sizeof (S_interface_descriptor), @/
  0x04, /* interface descriptor */
  0, /* this corresponds to `0' in `if0' on picture */
  0, /* this corresponds to `0' in `alt0' on picture */
  1, /* one endpoint is used */
  0x02, /* CDC (\S4.2 in CDC spec) */
  0x02, /* ACM (\S4.3 in CDC spec) */
  0x01, /* AT command (\S4.4 in CDC spec) */
@t\2@> 0 /* not used */
}

@ @<Initialize element 8 in USB configuration descriptor@>= { @t\1@> @/
  sizeof (S_interface_descriptor), @/
  0x04, /* interface descriptor */
  1, /* this corresponds to `1' in `if1' on picture */
  0, /* this corresponds to `0' in `alt0' on picture */
  2, /* two endpoints are used */
  0x0A, /* CDC data (\S4.5 in CDC spec) */
  0x00, /* unused */
  0x00, /* no protocol */
@t\2@> 0 /* not used */
}

@*2 Endpoint descriptor.

@s S_endpoint_descriptor int

@<Type definitions ...@>=
typedef struct {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bEndpointAddress;
  uint8_t bmAttributes;
  uint16_t wMaxPacketSize;
  uint8_t bInterval; /* interval for polling EP by host to determine if data is available (ms-1) */
} S_endpoint_descriptor;

@ @d IN (1 << 7)

@<Initialize element 7 in USB configuration descriptor@>= { @t\1@> @/
  sizeof (S_endpoint_descriptor), @/
  0x05, /* endpoint */
  IN | 3, /* this corresponds to `3' in `ep3' on picture */
  0x03, /* transfers via interrupts\footnote\dag{Must correspond to
    |UECFG0X| of |EP3|.} */
  0x0020, /* 32 bytes */
@t\2@> 0xFF /* 256 */
}

@ @<Initialize element 9 in USB configuration descriptor@>= { @t\1@> @/
  sizeof (S_endpoint_descriptor), @/
  0x05, /* endpoint */
  IN | 1, /* this corresponds to `1' in `ep1' on picture */
  0x02, /* bulk transfers\footnote\dag{Must correspond to
    |UECFG0X| of |EP1|.} */
  0x0020, /* 32 bytes */
@t\2@> 0x00 /* not applicable */
}

@ @d OUT (0 << 7)

@<Initialize element 10 in USB configuration descriptor@>= { @t\1@> @/
  sizeof (S_endpoint_descriptor), @/
  0x05, /* endpoint */
  OUT | 2, /* this corresponds to `2' in `ep2' on picture */
  0x02, /* bulk transfers\footnote\dag{Must correspond to
    |UECFG0X| of |EP2|.} */
  0x0020, /* 32 bytes */
@t\2@> 0x00 /* not applicable */
}

@*2 CDC descriptors.

@*3 CDC descriptor.

The class-specific descriptor shall start with a header.
It identifies the release of the USB Class Definitions for
Communication Devices Specification with which this
interface and its descriptors comply.

\S5.2.3.1 in CDC spec.

@s S_cdc_descriptor int

@<Type definitions ...@>=
typedef struct {
  uint8_t bFunctionLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubtype;
  uint16_t bcdCDC;
} S_cdc_descriptor;

@ @<Initialize element 3 in USB configuration descriptor@>= { @t\1@> @/
  sizeof (S_cdc_descriptor), @/
  0x24, /* interface */
  0x00, /* header */
@t\2@> 0x0110 /* CDC 1.1 */
}

@*3 Call management descriptor.

FIXME: remove it?
@^FIXME@>

The Call Management functional descriptor describes
the processing of calls for the Communication Class interface.

\S5.2.3.2 in CDC spec.

@s S_call_management_descriptor int

@<Type definitions ...@>=
typedef struct {
  uint8_t bFunctionLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubtype;
  uint8_t bmCapabilities;
  uint8_t bDataInterface;
} S_call_management_descriptor;

@ @<Initialize element 4 in USB configuration descriptor@>= { @t\1@> @/
  sizeof (S_call_management_descriptor), @/
  0x24, /* interface */
  0x01, /* call management */
  0x03, @/
@t\2@> 1 /* number of CDC data interface */
}

@*3 Abstract control management descriptor.

The Abstract Control Management functional descriptor
describes the commands supported by the Communication
Class interface, as defined in \S3.6.2 in CDC spec, with the
SubClass code of Abstract Control Model.

\S5.2.3.3 in CDC spec.

@s S_acm_descriptor int

@<Type definitions ...@>=
typedef struct {
  uint8_t bFunctionLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubtype;
  uint8_t bmCapabilities;
} S_acm_descriptor;

@ @<Initialize element 5 in USB configuration descriptor@>= { @t\1@> @/
  sizeof (S_acm_descriptor), @/
  0x24, /* interface */
  0x02, /* ACM */
@t\2@> 0x06 @/
}

@*3 Union descriptor.

The Union functional descriptor describes the relationship between
a group of interfaces that can be considered to form
a functional unit. One of the interfaces in
the group is designated as a master or controlling interface for
the group, and certain class-specific messages can be
sent to this interface to act upon the group as a whole. Similarly,
notifications for the entire group can be sent from this
interface but apply to the entire group of interfaces.

\S5.2.3.8 in CDC spec.

@s S_union_descriptor int

@<Type definitions ...@>=
typedef struct {
  uint8_t bFunctionLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubtype;
  uint8_t bMasterInterface;
  uint8_t bSlaveInterface0;
} S_union_descriptor;

@ @<Initialize element 6 in USB configuration descriptor@>= { @t\1@> @/
  sizeof (S_union_descriptor), @/
  0x24, /* interface */
  0x06, /* union */
  0, /* number of CDC control interface */
@t\2@> 1 /* number of CDC data interface */
}

@ @c
@<Global variables@>@;
U8 endpoint_status[7];
U8 usb_configuration_nb;
volatile U8 usb_request_break_generation = 0;
volatile U8 rs2usb[10];
volatile U8 cpt_sof;
volatile U16 g_usb_event = 0;
S_line_coding line_coding;
U8 usb_suspended = 0;
U8 usb_connected = 0;
Uchar rx_counter;
S_line_status line_status;

#define EP0 0
#define EP1 1
#define EP2 2
#define EP3 3

@ @d EP0_SIZE 32 /* 32 bytes\footnote\dag{Must correspond to |UECFG1X| of |EP0|.} */

@c
int main(void)
{
  @<Disable WDT@>@;

  UHWCON |= 1 << UVREGE;
  clock_prescale_set(0);
  USBCON &= ~(1 << USBE);
  USBCON |= 1 << USBE;
  USBCON |= 1 << OTGPADE;
  USBCON |= 1 << VBUSTE;
  sei();
  UBRR1 = (U16) (((U32) 16000 * 1000L) / ((U32) 57600 / 2 * 16) - 1);
  UCSR1A |= 1 << U2X1;
  UCSR1C = 1 << UCSZ11 | 1 << UCSZ10;
  UCSR1B |= 1 << RXEN1 | 1 << TXEN1;
  UCSR1B |= 1 << RXCIE1;
  DDRD |= 1 << PD5;
  DDRB |= 1 << PB0;
  DDRF &= ~(1 << PF4), PORTF |= 1 << PF4; /* input */
  DDRF &= ~(1 << PF5), PORTF |= 1 << PF5; /* input */
  DDRF &= ~(1 << PF6), PORTF |= 1 << PF6; /* input */
  DDRD |= 1 << PD7; /* ground */

  UDIEN |= 1 << SOFE;

  while (1) {
    if (!usb_connected) {
      if (USBSTA & 1 << VBUS) {
        USBCON |= 1 << USBE;
        usb_connected = 1;
        USBCON |= 1 << FRZCLK;

        PLLFRQ &=
          ~(1 << PDIV3 | 1 << PDIV2 | 1 << PDIV1 | 1 << PDIV0),
          PLLFRQ |= 1 << PDIV2, PLLCSR = 1 << PINDIV | 1 << PLLE;
        while (!(PLLCSR & 1 << PLOCK)) ;
        USBCON &= ~(1 << FRZCLK);
        UDCON &= ~(1 << DETACH);
        UDCON &= ~(1 << RSTCPU);
        UDIEN |= 1 << SUSPE;
        UDIEN |= 1 << EORSTE;
        sei();
      }
    }
    if (g_usb_event & 1 << EVT_USB_RESET) {
      g_usb_event &= ~(1 << EVT_USB_RESET);
      UERST = 1 << EP0, UERST = 0;
      usb_configuration_nb = 0;
    }
    UENUM = EP0;
    if (UEINTX & 1 << RXSTPI) {
      @<Process SETUP request@>@;
    }
    if (usb_configuration_nb != 0 && line_status.DTR) { /* do not allow to send data before
                                                 end of enumeration AND open port detection */
      if (UCSR1A & 1 << UDRE1) {
        if (!rx_counter) {
          UENUM = EP2;
          if (UEINTX & 1 << RXOUTI) {
            rx_counter = (U8) (UEBCLX); /* TODO: remove parens + cast and chk via objdump */
            if (!rx_counter) {
              UEINTX &= ~(1 << RXOUTI), UEINTX &= ~(1 << FIFOCON);
            }
          }
        }
        if (rx_counter) {
          while (rx_counter) {
            while (!(UCSR1A & 1 << UDRE1)) ;
            UENUM = EP2; // FIXME: this seems not to be necessary
            UDR1 = UEDATX;
            rx_counter--;
            if (!rx_counter)
              UEINTX &= ~(1 << RXOUTI), UEINTX &= ~(1 << FIFOCON);
            PORTB ^= 1 << PB0;
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
        PORTD ^= 1 << PD5;
        @<Reset MCU@>@;
      }
    }
  }
  return 0;
}

char __low_level_init(void) __attribute__ ((section(".init3"), naked));
char __low_level_init()
{
  clock_prescale_set(0);
  return 1;
}

@ Used in \.{USB\_RESET} interrupt handler.

@<Reset MCU@>=
WDTCSR |= 1 << WDCE | 1 << WDE; /* allow to enable WDT */
WDTCSR = 1 << WDE; /* enable WDT */
while (1) ;

@ When reset is done via watchdog, WDRF (WatchDog Reset Flag) is set in MCUSR register.
WDE (WatchDog system reset Enable) is always set in WDTCSR when WDRF is set. It
is necessary to clear WDE to stop MCU from eternal resetting:
on MCU start we always clear |WDRF| and WDE
(nothing will change if they are not set).
To avoid unintentional changes of WDE, a special write procedure must be followed
to change the WDE bit. To clear WDE, WDRF must be cleared first.

This should be done right at the beginning of |main|, in order to be in
time before WDT is triggered.
No need to call \\{wdt\_reset} because initialization code,
that \.{avr-gcc} adds, has enough time to execute before watchdog
timer (16ms in this program) expires:

$$\vbox{\halign{\tt#\cr
  eor r1, r1 \cr
  out 0x3f, r1 \cr
  ldi r28, 0xFF	\cr
  ldi r29, 0x0A	\cr
  out 0x3e, r29	\cr
  out 0x3d, r28	\cr
  call <main> \cr
}}$$

At 16MHz each cycle is 62.5 nanoseconds, so it is 7 instructions,
taking FIXME cycles, multiplied by 62.5 is ????.

@<Disable WDT@>=
MCUSR = 0x00; /* clear WDRF */
WDTCSR |= 1 << WDCE | 1 << WDE; /* allow to disable WDT */
WDTCSR = 0x00; /* disable WDT */

@ @<Predeclarations of procedures@>=
void uart_usb_send_buffer(U8 *buffer, U8 nb_data);
@ @c
void uart_usb_send_buffer(U8 *buffer, U8 nb_data)
{
  U8 zlp;

  if (nb_data % 0x20) {
    zlp = 0;
  }
  else {
    zlp = 1;
  }

  UENUM = EP1;
  while (nb_data) {
    while (!(UEINTX & 1 << RWAL)) ;
    while (UEINTX & 1 << RWAL && nb_data) {
      UEDATX = (U8) * buffer;
      buffer++;
      nb_data--;
    }
    UEINTX &= ~(1 << TXINI), UEINTX &= ~(1 << FIFOCON);
  }
  if (zlp) {
    while (!(UEINTX & 1 << RWAL)) ;
    UEINTX &= ~(1 << TXINI), UEINTX &= ~(1 << FIFOCON);
  }
}

ISR(USB_GEN_vect)
{
  if (USBINT & 1 << VBUSTI && USBCON & 1 << VBUSTE) {
    USBINT = ~(1 << VBUSTI);
    if (USBSTA & 1 << VBUS) {
      usb_connected = 1;
      g_usb_event |= 1 << EVT_USB_POWERED;
      UDIEN |= 1 << EORSTE;
      USBCON |= 1 << FRZCLK;

      PLLFRQ &= ~(1 << PDIV3 | 1 << PDIV2 | 1 << PDIV1 | 1 << PDIV0),
        PLLFRQ |= 1 << PDIV2, PLLCSR = 1 << PINDIV | 1 << PLLE;
      while (!(PLLCSR & 1 << PLOCK)) ;
      USBCON &= ~(1 << FRZCLK);
      UDCON &= ~(1 << DETACH);

      UDCON &= ~(1 << RSTCPU);

      UDIEN |= 1 << SUSPE;
      UDIEN |= 1 << EORSTE;
      sei();
      UDCON &= ~(1 << DETACH);
    }
    else {
      usb_connected = 0;
      usb_configuration_nb = 0;
      g_usb_event |= 1 << EVT_USB_UNPOWERED;
    }
  }
  if (UDINT & 1 << SOFI && UDIEN & 1 << SOFE) {
    UDINT = ~(1 << SOFI);
    cpt_sof++;
  }
  if (UDINT & 1 << SUSPI && UDIEN & 1 << SUSPE) {
    usb_suspended = 1;
    UDINT = ~(1 << WAKEUPI);
    g_usb_event |= 1 << EVT_USB_SUSPEND;
    UDINT = ~(1 << SUSPI);
    UDIEN |= 1 << WAKEUPE;
    UDIEN &= ~(1 << EORSME);
    USBCON |= 1 << FRZCLK;
    PLLCSR &= ~(1 << PLLE), PLLCSR = 0;
  }
  if (UDINT & 1 << WAKEUPI && UDIEN & 1 << WAKEUPE) {
    if (!(PLLCSR & 1 << PLOCK)) {
      PLLFRQ &=
        ~(1 << PDIV3 | 1 << PDIV2 | 1 << PDIV1 | 1 << PDIV0),
        PLLFRQ |= 1 << PDIV2, PLLCSR = 1 << PINDIV | 1 << PLLE;
      while (!(PLLCSR & (1 << PLOCK))) ;
    }
    USBCON &= ~(1 << FRZCLK);
    UDINT = ~(1 << WAKEUPI);
    if (usb_suspended) {
      UDIEN |= 1 << EORSME;
      UDIEN |= 1 << EORSTE;
      UDINT = ~(1 << WAKEUPI);
      UDIEN &= ~(1 << WAKEUPE);
      g_usb_event |= 1 << EVT_USB_WAKE_UP;
      UDIEN |= 1 << SUSPE;
      UDIEN |= 1 << EORSME;
      UDIEN |= 1 << EORSTE;
    }
  }
  if (UDINT & 1 << EORSMI && UDIEN & 1 << EORSME) {
    usb_suspended = 0;
    UDIEN &= ~(1 << WAKEUPE);
    UDINT = ~(1 << EORSMI);
    UDIEN &= ~(1 << EORSME);
    g_usb_event |= 1 << EVT_USB_RESUME;
  }
  if (UDINT & 1 << EORSTI && UDIEN & 1 << EORSTE) {
    UDINT = ~(1 << EORSTI);
    UENUM = EP0;
    if (!(UECONX & 1 << EPEN)) {
      UENUM = EP0;
      UECONX |= 1 << EPEN;
      UECFG1X = 1 << EPSIZE1;
      UECFG1X |= 1 << ALLOC;
    }
    g_usb_event |= 1 << EVT_USB_RESET;
  }
}

@ TODO: to be able to use rx and tx in parallel, just put byte to buffer
here and in main loop alternate between rx and tx byte by byte;
for now, make via a simple buffer and use a led to indicate if it is full,
and later do via ring buffer
@^TODO@>

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
    } while (!(UEINTX & 1 << RWAL));
    uart_usb_send_buffer((U8 *) &rs2usb, i);
    UENUM = (U8) save_ep;
  }
}

@ The following big switch just dispatches SETUP request.

@<Process SETUP request@>=
U8 addr;
U16 wValue;
U8 wIndex;
U8 configuration_number;
UEINTX &= ~(1 << RXOUTI); /* TODO: ??? - check if it is non-zero here */
U16 wLength;
U8 nb_byte;
U8 zlp = 0;
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
case 0x0500: @/
  @<Handle {\caps set address}@>@;
  break;
case 0x0680: @/
  switch (UEDATX | UEDATX << 8) {
  case 0x0100: @/
    @<Handle {\caps get descriptor device}\null@>@;
    break;
  case 0x0200: @/
    @<Handle {\caps get descriptor configuration}@>@;
    break;
  default: /* in code derived from this example change this to ``Handle
              {\caps get descriptor device qualifier}'' */
    UECONX |= 1 << STALLRQ;
    UEINTX &= ~(1 << RXSTPI);
  }
  break;
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
  UECONX |= 1 << STALLRQ;
  UEINTX &= ~(1 << RXSTPI);
}

@ @<Global variables@>=
U8 data_to_transfer;
const void *pbuffer;

@ @<Handle {\caps get descriptor device}\null@>=
data_to_transfer = sizeof dev_desc;
pbuffer = &dev_desc;
@<Code which is executed in |0x0680| for both |0x0100| and |0x0200|@>@;

@ @<Handle {\caps get descriptor configuration}@>=
data_to_transfer = sizeof usb_conf_desc;
pbuffer = &usb_conf_desc;
@<Code which is executed in |0x0680| for both |0x0100| and |0x0200|@>@;

@ @<Code which is executed in |0x0680| for both |0x0100| and |0x0200|@>=
    (void) UEDATX;
    (void) UEDATX;
    wLength = UEDATX | UEDATX << 8;
    UEINTX &= ~(1 << RXSTPI);
    if (wLength > data_to_transfer) {
      if (data_to_transfer % 32 == 0) {
        zlp = 1;
      }
      else {
        zlp = 0;
      }
    }
    else {
      data_to_transfer = (U8) wLength;
    }
    UEINTX &= ~(1 << NAKOUTI);
    while (data_to_transfer != 0 && !(UEINTX & 1 << NAKOUTI)) {
      while (!(UEINTX & 1 << TXINI)) {
        if (UEINTX & 1 << NAKOUTI)
          break;
      }
      nb_byte = 0;
      while (data_to_transfer != 0) {
        if (nb_byte++ == 32) {
          break;
        }
        UEDATX = (U8) pgm_read_byte_near((unsigned int) pbuffer++);
        data_to_transfer--;
      }
      if (UEINTX & 1 << NAKOUTI)
        break;
      else
        UEINTX &= ~(1 << TXINI);
    }
    if (zlp == 1 && !(UEINTX & 1 << NAKOUTI)) {
      while (!(UEINTX & 1 << TXINI)) ;
      UEINTX &= ~(1 << TXINI);
    }
    while (!(UEINTX & 1 << NAKOUTI)) ;
    UEINTX &= ~(1 << NAKOUTI);
    UEINTX &= ~(1 << RXOUTI);

@ @<Handle {\caps set address}@>=
  addr = UEDATX;
  UDADDR = (UDADDR & 1 << ADDEN) | ((U8) addr & 0x7F);
  UEINTX &= ~(1 << RXSTPI);
  UEINTX &= ~(1 << TXINI);
  while (!(UEINTX & 1 << TXINI)) ; /* TODO: put here explanation like in kbd.w */
  UDADDR |= 1 << ADDEN;

@ @<Handle {\caps set configuration}@>=
  configuration_number = UEDATX;
  if (configuration_number <= 1) {
    UEINTX &= ~(1 << RXSTPI);
    usb_configuration_nb = configuration_number;
    UEINTX &= ~(1 << TXINI); /* STATUS stage */

    UENUM = EP3;
    UECONX |= 1 << EPEN;
    UECFG0X = 1 << EPTYPE1 | 1 << EPTYPE0 | 1 << EPDIR;       /* interrupt, IN */
    UECFG1X = 1 << EPSIZE1;   /* 32 bytes */
    UECFG1X |= 1 << ALLOC;

    UENUM = EP1;
    UECONX |= 1 << EPEN;
    UECFG0X = 1 << EPTYPE1 | 1 << EPDIR;      /* bulk, IN */
    UECFG1X = 1 << EPSIZE1;   /* 32 bytes */
    UECFG1X |= 1 << ALLOC;

    UENUM = EP2;
    UECONX |= 1 << EPEN;
    UECFG0X = 1 << EPTYPE1;   /* bulk, OUT */
    UECFG1X = 1 << EPSIZE1;   /* 32 bytes */
    UECFG1X |= 1 << ALLOC;

    UERST = 1 << EP3, UERST = 0;
    UERST = 1 << EP1, UERST = 0;
    UERST = 1 << EP2, UERST = 0;
  }
  else {
    UECONX |= 1 << STALLRQ;
    UEINTX &= ~(1 << RXSTPI);
  }

@ This will send two bytes during the DATA stage. Only first two bits of the first byte are used.
If first bit is set, then this indicates the device is self powered. If clear, the device
is bus powered. If second bit is set, the device has remote wakeup enabled and can wake the
host up during suspend. The remote wakeup bit can be by the {\caps set feature} and
{\caps clear feature} requests with a feature selector of |0x01| (\.{DEVICE\_REMOTE\_WAKEUP}).

@<Handle {\caps get status device}@>=
  UEINTX &= ~(1 << RXSTPI);
  UEDATX = 0x01;
  UEDATX = 0x00;
  UEINTX &= ~(1 << TXINI);
  while (!(UEINTX & 1 << RXOUTI)) ;
  UEINTX &= ~(1 << RXOUTI);

@ Sends two bytes of |0x00|, |0x00|. (Both bytes are reserved for future use.)

@<Handle {\caps get status interface}@>=
  UEINTX &= ~(1 << RXSTPI);
  UEDATX = 0x00;
  UEDATX = 0x00;
  UEINTX &= ~(1 << TXINI);
  while (!(UEINTX & 1 << RXOUTI)) ;
  UEINTX &= ~(1 << RXOUTI);

@ Sends two bytes indicating the status (Halted/Stalled) of an endpoint.
Only first bit of the first byte is used.

@<Handle {\caps get status endpoint}@>=
  (void) UEDATX;
  (void) UEDATX;
  wIndex = UEDATX;
  UEINTX &= ~(1 << RXSTPI);
  wIndex = wIndex & 0x7F;
  UEDATX = (U8) endpoint_status[wIndex];
  UEDATX = 0x00;
  UEINTX &= ~(1 << TXINI);
  while (!(UEINTX & 1 << RXOUTI)) ;
  UEINTX &= ~(1 << RXOUTI);

@ This gets the Alternative Interface setting.

@<Handle {\caps get interface}@>=
  UEINTX &= ~(1 << RXSTPI);
  UEINTX &= ~(1 << TXINI);
  while (!(UEINTX & 1 << RXOUTI)) ;
  UEINTX &= ~(1 << RXOUTI);

@ Used to set boolean features. The only two feature selectors available are
\.{DEVICE\_REMOTE\_WAKEUP} and \.{TEST\_MODE}.

@<Handle {\caps set feature device}@>=
  UECONX |= 1 << STALLRQ;
  UEINTX &= ~(1 << RXSTPI);

@ @<Handle {\caps clear feature device}@>=
  UECONX |= 1 << STALLRQ;
  UEINTX &= ~(1 << RXSTPI);

@ Used to set boolean features. The current USB Specification Revision 2 specifies no
interface features.

@<Handle {\caps set feature interface}@>=
  UECONX |= 1 << STALLRQ;
  UEINTX &= ~(1 << RXSTPI);

@ @<Handle {\caps clear feature interface}@>=
  UECONX |= 1 << STALLRQ;
  UEINTX &= ~(1 << RXSTPI);

@ Used to set endpoint features. The standard currently defines one endpoint feature
selector |0x00|, which allows the host to stall and clear an endpoint.
Only endpoints other than the default endpoint are recommended to have this functionality.

@<Handle {\caps set feature endpoint}@>=
  wValue = UEDATX;
  (void) UEDATX;
  if (wValue == 0x00) {
    wIndex = UEDATX & 0x7F;
    if (wIndex == 0) {
      UECONX |= 1 << STALLRQ;
      UEINTX &= ~(1 << RXSTPI);
    }
    UENUM = (U8) wIndex;
    if (UECONX & 1 << EPEN) {
      UECONX |= 1 << STALLRQ;
      UENUM = EP0;
      endpoint_status[wIndex] = 0x01;
      UEINTX &= ~(1 << RXSTPI);
      UEINTX &= ~(1 << TXINI); /* STATUS stage */
    }
    else {
      UENUM = EP0;
      UECONX |= 1 << STALLRQ;
      UEINTX &= ~(1 << RXSTPI);
    }
  }
  else {
    UECONX |= 1 << STALLRQ;
    UEINTX &= ~(1 << RXSTPI);
  }

@ @<Handle {\caps clear feature endpoint}@>=
  wValue = UEDATX;
  (void) UEDATX;
  if (wValue == 0x00) {
    wIndex = UEDATX & 0x7F;
    UENUM = (U8) wIndex;
    if (UECONX & 1 << EPEN) {
      if (wIndex != 0) {
        UECONX |= 1 << STALLRQC;
        UERST = 1 << (U8) wIndex, UERST = 0;
        UECONX |= 1 << RSTDT;
      }
      UENUM = EP0;
      endpoint_status[wIndex] = 0x00;
      UEINTX &= ~(1 << RXSTPI);
      UEINTX &= ~(1 << TXINI); /* STATUS stage */
    }
    else {
      UENUM = EP0;
      UECONX |= 1 << STALLRQ;
      UEINTX &= ~(1 << RXSTPI);
    }
  }
  else {
    UECONX |= 1 << STALLRQ;
    UEINTX &= ~(1 << RXSTPI);
  }

@ Used to request the current device configuration. A byte will be sent during the DATA stage
indicating the devices status. A zero value means the device is not configured and a non-zero
value indicates the device is configured.

@<Handle {\caps get configuration}@>=
  UEINTX &= ~(1 << RXSTPI);
  UEDATX = (U8) usb_configuration_nb;
  UEINTX &= ~(1 << TXINI); /* DATA stage */
  while (!(UEINTX & 1 << RXOUTI)) ; /* wait for STATUS stage */
  UEINTX &= ~(1 << RXOUTI);

@ This request allows the host to find out the currently configured line coding. (\S6.2.13 in
CDC spec.)

@<Handle {\caps get line coding}@>=
  UEINTX &= ~(1 << RXSTPI);
  UEDATX = (U8) ((U8 *) &line_coding.dwDTERate)[0];
  UEDATX = (U8) ((U8 *) &line_coding.dwDTERate)[1];
  UEDATX = (U8) ((U8 *) &line_coding.dwDTERate)[2];
  UEDATX = (U8) ((U8 *) &line_coding.dwDTERate)[3];
  UEDATX = (U8) line_coding.bCharFormat;
  UEDATX = (U8) line_coding.bParityType;
  UEDATX = (U8) line_coding.bDataBits;
  UEINTX &= ~(1 << TXINI); /* DATA stage */
  while (!(UEINTX & 1 << RXOUTI)) ; /* wait for STATUS stage */
  UEINTX &= ~(1 << RXOUTI);

@ This request sends special carrier modulation that generates an RS-232 style break.
(\S6.2.15 in CDC spec.)

TODO: manage here hardware flow control
@^TODO@>

@<Handle {\caps send break}@>=
  UEINTX &= ~(1 << RXSTPI);
  UEINTX &= ~(1 << TXINI);
  usb_request_break_generation = 1;

@ This request generates RS-232/V.24 style control signals.
(\S6.2.14 and in CDC spec.)

Especially, first bit of first byte indicates to DCE if DTE is present or not.
This signal corresponds to RS-232 signal DTR (0 --- Not Present, 1 --- Present).
@^DTR@>

TODO: manage here hardware flow control
@^TODO@>

@<Handle {\caps set control line state}@>=
  line_status.all = UEDATX | UEDATX << 8;
  UEINTX &= ~(1 << RXSTPI);
  UEINTX &= ~(1 << TXINI); /* STATUS stage */

@ This request allows the host to specify typical asynchronous line-character formatting
properties. (\S6.2.12 in CDC spec.)

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
  /*|while (!(UEINTX & 1 << TXINI)) ;|*/ /* FIXME: is it necessary? */
/*  |UBRR1 = (U16) (((U32) 16000 * 1000L) /
                ((U32) line_coding.dwDTERate / 2 * 16) - 1);|*/
/*see commit 1325440b633fd639ec158b17b5afaf76b3aa998e in usb/ */

@ This request allows the host to select an alternate setting for the specified interface.
(\S9.4.10 in USB spec.)

@<Handle {\caps set interface}@>=
  UEINTX &= ~(1 << RXSTPI);
  UEINTX &= ~(1 << TXINI); /* STATUS stage */

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

@ @<Global variables@>=
S_serial_state serial_state;
S_serial_state serial_state_saved;

@ Check if serial state has changed and update host with that information.
Necessary for hardware handshake support.
(\S6.3.5 in CDC spec.)

|serial_state| is used like a real interrupt status register. Once a notification has been sent,
the device will reset and re-evaluate the different signals. For the consistent signals like
carrier detect or transmission carrier, this will mean another notification will not be generated
until there is a state change. For the irregular signals like break, the incoming ring signal,
or the overrun error state, this will reset their values to zero and again will not send another
notification until their state changes.

Here we have to wait (before or after sending notification, but
`after' is not efficient\footnote*{It is not efficient
to wait right after clearing |FIFOCON|.
We may do other things --- meanwhile the data will be transmitted.
It is only necessary to wait right before sending next data.})
if we want to make sure that notification was delivered
to host. Incidentally, on control endpoint it is never necessary
to wait TXINI to become `1', because on control endpoint if RXSTPI is `1',
TXINI is guaranteed to be `1'.

@<Notify host if |serial_state| changed@>=
if (serial_state_saved.all != serial_state.all) {
  serial_state_saved.all = serial_state.all;
  UENUM = EP3;
  while (!(UEINTX & 1 << TXINI)) ; /* wait until previous packet was sent */
  UEDATX = 0xA1;
  UEDATX = 0x20;
  UEDATX = 0x00; @+ UEDATX = 0x00;
  UEDATX = 0x00; @+ UEDATX = 0x00;
  UEDATX = 0x02; @+ UEDATX = 0x00;
  UEDATX = (U8) ((U8 *) &serial_state.all)[0];
  UEDATX = (U8) ((U8 *) &serial_state.all)[1];
  UEINTX &= ~(1 << TXINI), UEINTX &= ~(1 << FIFOCON);
}

@* Headers.
\secpagedepth=1 % index on current page

@<Header files@>=
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/power.h> /* |clock_prescale_set| */

@* Index.
