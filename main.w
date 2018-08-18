%incompatible changes started in commit 0abb132676c5c072b87414e01b306804df913c8b
\let\lheader\rheader
%\datethis
\secpagedepth=2 % begin new page only on *
\font\caps=cmcsc10 at 9pt

@* Program. This is cleaned-up Atmel's demo.

@c
@<Header files@>@;

typedef unsigned char U8;
typedef unsigned short U16;
typedef unsigned long U32;
typedef unsigned char Bool;

#define EVT_USB_POWERED               1         // USB plugged
#define EVT_USB_UNPOWERED             2         // USB un-plugged
#define EVT_USB_SUSPEND               5
#define EVT_USB_WAKE_UP               6
#define EVT_USB_RESUME                7
#define EVT_USB_RESET                 8

@<Type \null definitions@>@;

@* USB stack.

@*1 Device descriptor.

TODO: find what prefixes mean in names of variables (i.e., `b', `bcd', ...)

@<Global variables@>=
struct {
  U8 bLength;
  U8 bDescriptorType;
  U16 bcdUSB; /* version */
  U8 bDeviceClass; /* class code assigned by the USB */
  U8 bDeviceSubClass; /* sub-class code assigned by the USB */
  U8 bDeviceProtocol; /* protocol code assigned by the USB */
  U8 bMaxPacketSize0; /* max packet size for EP0 */
  U16 idVendor;
  U16 idProduct;
  U16 bcdDevice; /* device release number */
  U8 iManufacturer; /* index of manu. string descriptor */
  U8 iProduct; /* index of prod. string descriptor */
  U8 iSerialNumber; /* index of S.N. string descriptor */
  U8 bNumConfigurations;
} const dev_desc
@t\hskip2.5pt@> @=PROGMEM@> = { @t\1@> @/
  18, /* size of this structure */
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

@*1 Configuration descriptor.

Abstract Control Model consists of two interfaces: Data Class interface
and Communication Class interface.

The Communication Class interface uses two endpoints\footnote*{Although
CDC spec says that notification endpoint is optional, but in Linux host
driver refuses to work without it.}, one to implement
a notification element and theh other to implement
a management element. The management element uses the default endpoint
for all standard and Communication Class-specific requests.

Theh Data Class interface consists of two endpoints to implement
channels over which to carry data.

\S3.4 in CDC spec.

$$\hbox to7.5cm{\vbox to7.88cm{\vfil\special{psfile=cdc-structure.eps
  clip llx=0 lly=0 urx=274 ury=288 rwi=2125}}\hfil}$$

@<Type \null definitions@>=
@<Type definitions used in configuration descriptor@>@;
typedef struct {
  @<Configuration header descriptor@> @,@,@! el1;
  S_interface_descriptor el2;
  @<Class-specific interface descriptor 1@> @,@,@! el3;
  @<Class-specific interface descriptor 2@> @,@,@! el4;
  @<Class-specific interface descriptor 3@> @,@,@! el5;
  @<Class-specific interface descriptor 4@> @,@,@! el6;
  S_endpoint_descriptor el7;
  S_interface_descriptor el8;
  S_endpoint_descriptor el9;
  S_endpoint_descriptor el10;
} S_configuration_descriptor;

@ @<Global variables@>=
const S_configuration_descriptor conf_desc
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
@t\2@> @<Initialize element \null 10 ...@> @/
};

@*2 Configuration header descriptor.

@ @<Configuration header descriptor@>=
struct {
   U8 bLength;
   U8 bDescriptorType;
   U16 wTotalLength;
   U8 bNumInterfaces;
   U8 bConfigurationValue; /* number between 1 and |bNumConfigurations|, for
     each configuration\footnote\dag{For some reason
     configurations start numbering with `1', and interfaces and altsettings with `0'.} */
   U8 iConfiguration; /* index of string descriptor */
   U8 bmAttibutes;
   U8 MaxPower;
}

@ @<Initialize element 1 in configuration descriptor@>= { @t\1@> @/
  9, /* size of this structure */
  0x02, /* configuration descriptor */
  sizeof (S_configuration_descriptor), @/
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
   U8 bLength;
   U8 bDescriptorType;
   U8 bInterfaceNumber; /* number between 0 and |bNumInterfaces-1|, for
                                     each interface */
   U8 bAlternativeSetting; /* number starting from 0, for each interface */
   U8 bNumEndpoints; /* number of EP except EP 0 */
   U8 bInterfaceClass; /* class code assigned by the USB */
   U8 bInterfaceSubClass; /* sub-class code assigned by the USB */
   U8 bInterfaceProtocol; /* protocol code assigned by the USB */
   U8 iInterface; /* index of string descriptor */
}  S_interface_descriptor;

@ @<Initialize element 2 in configuration descriptor@>= { @t\1@> @/
  9, /* size of this structure */
  0x04, /* interface descriptor */
  0, /* this corresponds to `0' in `if0' on picture */
  0, /* this corresponds to `0' in `alt0' on picture */
  1, /* one endpoint is used */
  0x02, /* CDC (\S4.2 in CDC spec) */
  0x02, /* ACM (\S4.3 in CDC spec) */
  0x01, /* AT command (\S4.4 in CDC spec) */
@t\2@> 0 /* not used */
}

@ @<Initialize element 8 in configuration descriptor@>= { @t\1@> @/
  9, /* size of this structure */
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
  U8 bLength;
  U8 bDescriptorType;
  U8 bEndpointAddress;
  U8 bmAttributes;
  U16 wMaxPacketSize;
  U8 bInterval; /* interval for polling EP by host to determine if data is available (ms-1) */
} S_endpoint_descriptor;

@ @d IN (1 << 7)

@<Initialize element 7 in configuration descriptor@>= { @t\1@> @/
  7, /* size of this structure */
  0x05, /* endpoint */
  IN | 3, /* this corresponds to `3' in `ep3' on picture */
  0x03, /* transfers via interrupts\footnote\dag{Must correspond to
    |UECFG0X| of |EP3|.} */
  0x0020, /* 32 bytes */
@t\2@> 0xFF /* 256 */
}

@ @<Initialize element 9 in configuration descriptor@>= { @t\1@> @/
  7, /* size of this structure */
  0x05, /* endpoint */
  IN | 1, /* this corresponds to `1' in `ep1' on picture */
  0x02, /* bulk transfers\footnote\dag{Must correspond to
    |UECFG0X| of |EP1|.} */
  0x0020, /* 32 bytes */
@t\2@> 0x00 /* not applicable */
}

@ @d OUT (0 << 7)

@<Initialize element \null 10 in configuration descriptor@>= { @t\1@> @/
  7, /* size of this structure */
  0x05, /* endpoint */
  OUT | 2, /* this corresponds to `2' in `ep2' on picture */
  0x02, /* bulk transfers\footnote\dag{Must correspond to
    |UECFG0X| of |EP2|.} */
  0x0020, /* 32 bytes */
@t\2@> 0x00 /* not applicable */
}

@*2 Functional descriptors.

These descriptors describe the content of the class-specific information
within an Interface descriptor. They all start with a common header
descriptor, which allows host software to easily parse the contents of
class-specific descriptors. Although the
Communication Class currently defines class specific interface descriptor
information, the Data Class does not.

\S5.2.3 in CDC spec.

@*3 Header functional descriptor.

The class-specific descriptor shall start with a header.
It identifies the release of the USB Class Definitions for
Communication Devices Specification with which this
interface and its descriptors comply.

\S5.2.3.1 in CDC spec.

@<Class-specific interface descriptor 1@>=
struct {
  U8 bFunctionLength;
  U8 bDescriptorType;
  U8 bDescriptorSubtype;
  U16 bcdCDC;
}

@ @<Initialize element 3 in configuration descriptor@>= { @t\1@> @/
  5, /* size of this structure */
  0x24, /* interface */
  0x00, /* header */
@t\2@> 0x0110 /* CDC 1.1 */
}

@*3 Call management functional descriptor.

FIXME: remove it?
@^FIXME@>

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

The Abstract Control Management functional descriptor
describes the commands supported by the Communication
Class interface, as defined in \S3.6.2 in CDC spec, with the
SubClass code of Abstract Control Model.

\S5.2.3.3 in CDC spec.

@<Class-specific interface descriptor 3@>=
struct {
  U8 bFunctionLength;
  U8 bDescriptorType;
  U8 bDescriptorSubtype;
  U8 bmCapabilities;
}

@ |bmCapabilities|: Only first four bits are used.
If first bit is set, then this indicates the device
supports the request combination of \.{Set\_Comm\_Feature},
\.{Clear\_Comm\_Feature}, and \.{Get\_Comm\_Feature}.
If second bit is set, then the device supports the request
combination of \.{Set\_Line\_Coding}, \.{Set\_Control\_Line\_State},
\.{Get\_Line\_Coding}, and the notification \.{Serial\_State}.
If the third bit is set, then the device supports the request
\.{Send\_Break}. If fourth bit is set, then the device
supports the notification \.{Network\_Connection}.
A bit value of zero means that the request is not supported.

@<Initialize element 5 in configuration descriptor@>= { @t\1@> @/
  4, /* size of this structure */
  0x24, /* interface */
  0x02, /* ACM */
@t\2@> 1 << 2 | 1 << 1 @/
}

@*3 Union functional descriptor.

The Union functional descriptor describes the relationship between
a group of interfaces that can be considered to form
a functional unit. One of the interfaces in
the group is designated as a master or controlling interface for
the group, and certain class-specific messages can be
sent to this interface to act upon the group as a whole. Similarly,
notifications for the entire group can be sent from this
interface but apply to the entire group of interfaces.

\S5.2.3.8 in CDC spec.

@<Class-specific interface descriptor 4@>=
struct {
  U8 bFunctionLength;
  U8 bDescriptorType;
  U8 bDescriptorSubtype;
  U8 bMasterInterface;
  U8 bSlaveInterface[SLAVE_INTERFACE_NUM];
}

@ @d SLAVE_INTERFACE_NUM 1

@<Initialize element 6 in configuration descriptor@>= { @t\1@> @/
  4 + SLAVE_INTERFACE_NUM, /* size of this structure */
  0x24, /* interface */
  0x06, /* union */
  0, /* number of CDC control interface */
  { @t\1@> @/
@t\2@> 1 /* number of CDC data interface */
@t\2@> } @/
}

@ @c
@<Global variables@>@;
U8 endpoint_status[7];
U8 usb_configuration_nb;
volatile U8 usb_request_break_generation = 0;
volatile U8 rs2usb[10];
volatile U16 g_usb_event = 0;
U8 usb_suspended = 0;
U8 usb_connected = 0;
U16 rx_counter;

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
  DDRD |= 1 << PD5;
  DDRB |= 1 << PB0;
#if 0
  DDRF &= ~(1 << PF4), PORTF |= 1 << PF4; /* input */
  DDRF &= ~(1 << PF5), PORTF |= 1 << PF5; /* input */
  DDRD |= 1 << PD7; /* ground */
#endif

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
    if (usb_configuration_nb != 0) { /* do not allow to receive data before
                                        end of enumeration FIXME: does this make any sense? */
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
#if 0
        if (!(PINF & 1 << PF4))
          serial_state.bDCD = 1;
        else
          serial_state.bDCD = 0;
        if (!(PINF & 1 << PF5))
          serial_state.bDSR = 1;
        else
          serial_state.bDSR = 0;
        @<Notify host if |serial_state| changed@>@;
#endif
      if (usb_request_break_generation == 1) {
        usb_request_break_generation = 0;
        PORTB ^= 1 << PB0;
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
We don't call \\{wdt\_reset} because initialization code,
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

(What the above code does: zero r1 register, clear SREG, initialize program stack
(to the stack processor writes addresses for returning from subroutines and interrupt
handlers). To the stack pointer is written address of last cell of RAM.)

@<Disable WDT@>=
MCUSR = 0x00; /* clear WDRF */
WDTCSR |= 1 << WDCE | 1 << WDE; /* allow to disable WDT */
WDTCSR = 0x00; /* disable WDT */

@ @c
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
    UECONX |= 1 << EPEN;
    UECFG1X = 1 << EPSIZE1; /* 32 bytes\footnote\ddag{Must correspond to |EP0_SIZE|.} */
    UECFG1X |= 1 << ALLOC;
    g_usb_event |= 1 << EVT_USB_RESET;
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

TODO: see usb/main.w

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
    } while (!(UEINTX & 1 << TXINI)); // while write not allowed
    if (line_status.DTR) {
      volatile U8 *buffer = rs2usb;
      U8 empty_packet = 0;
      if (i % EP0_SIZE == 0)
        empty_packet = 1; /* indicate to the host that no more data will follow (USB\S5.8.3) */
      while (i) {
        while (!(UEINTX & 1 << TXINI)) ;
        UEINTX &= ~(1 << TXINI);
        while (UEINTX & 1 << RWAL && i) {
          UEDATX = *buffer;
          buffer++;
          i--;
        }
        UEINTX &= ~(1 << FIFOCON);
      }
      if (empty_packet) {
        while (!(UEINTX & 1 << TXINI)) ;
        UEINTX &= ~(1 << TXINI);
        UEINTX &= ~(1 << FIFOCON);
      }
    }
    UENUM = save_ep;
  }
}

@ The following big switch just dispatches SETUP request.

@<Process SETUP request@>=
U16 wValue;
U16 wIndex;
U16 wLength;
U8 configuration_number;
UEINTX &= ~(1 << RXOUTI); /* TODO: ??? - check if it is non-zero here */
U8 nb_byte;
U8 empty_packet;
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
    UEINTX &= ~(1 << RXSTPI);
    UECONX |= 1 << STALLRQ; /* return STALL in response to IN or OUT token of
      DATA or SETUP stage */
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
  UEINTX &= ~(1 << RXSTPI);
  UECONX |= 1 << STALLRQ; /* return STALL in response to IN or OUT token of
    DATA or SETUP stage */
}

@ @<Global variables@>=
U16 data_to_transfer;
const void *pbuffer;

@ When host is booting, BIOS asks 8 bytes in first request of device descriptor (8 bytes is
sufficient for first request of device descriptor). OS asks
64 bytes in first request of device descriptor.
It is OK if we transfer less than the requested amount. But if we try to
transfer more, host does not send OUT packet to initiate STATUS stage.

@<Handle {\caps get descriptor device}\null@>=
(void) UEDATX; @+ (void) UEDATX;
wLength = UEDATX | UEDATX << 8;
UEINTX &= ~(1 << RXSTPI);
data_to_transfer = sizeof dev_desc;
pbuffer = &dev_desc;
@<Send descriptor@>@;

@ First request is 9 bytes, second is according to length given in response to first request.

@<Handle {\caps get descriptor configuration}@>=
(void) UEDATX; @+ (void) UEDATX;
wLength = UEDATX | UEDATX << 8;
UEINTX &= ~(1 << RXSTPI);
data_to_transfer = sizeof conf_desc;
pbuffer = &conf_desc;
@<Send descriptor@>@;

@ @<Send descriptor@>=
    empty_packet = 0;
    if (data_to_transfer < wLength && data_to_transfer % EP0_SIZE == 0)
      empty_packet = 1; /* indicate to the host that no more data will follow (USB\S5.5.3) */
    if (data_to_transfer > wLength)
      data_to_transfer = wLength; /* never send more than requested */
    UEINTX &= ~(1 << NAKOUTI); /* TODO: ??? - check if it is non-zero here */
    while (data_to_transfer != 0 && !(UEINTX & 1 << NAKOUTI)) {
      while (!(UEINTX & 1 << TXINI)) {
        if (UEINTX & 1 << NAKOUTI)
          break;
      }
      nb_byte = 0;
      while (data_to_transfer != 0) {
        if (nb_byte++ == EP0_SIZE) {
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
    if (empty_packet && !(UEINTX & 1 << NAKOUTI)) {
      while (!(UEINTX & 1 << TXINI)) ;
      UEINTX &= ~(1 << TXINI);
    }
    while (!(UEINTX & 1 << NAKOUTI)) ;
    UEINTX &= ~(1 << NAKOUTI);
    UEINTX &= ~(1 << RXOUTI);

@ @<Handle {\caps set address}@>=
  wValue = UEDATX | UEDATX << 8;
  UDADDR = wValue & 0x7F;
  UEINTX &= ~(1 << RXSTPI);
  UEINTX &= ~(1 << TXINI);
  while (!(UEINTX & 1 << TXINI)) ; /* wait until ZLP, prepared by previous command, is
            sent to host\footnote{$\sharp$}{According to \S22.7 of the datasheet,
            firmware must send ZLP in the STATUS stage before enabling the new address.
            The reason is that the request started by using zero address, and all the stages of the
            request must use the same address.
            Otherwise STATUS stage will not complete, and thus set address request will not
            succeed. We can determine when ZLP is sent by receiving the ACK, which sets TXINI to 1.
            See ``Control write (by host)'' in table of contents for the picture (note that DATA
            stage is absent).} */
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
    UEINTX &= ~(1 << RXSTPI);
    UECONX |= 1 << STALLRQ; /* return STALL in response to IN token of STATUS stage */
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
UEINTX &= ~(1 << RXSTPI);
UECONX |= 1 << STALLRQ; /* return STALL in response to IN token of STATUS stage */

@ @<Handle {\caps clear feature device}@>=
UEINTX &= ~(1 << RXSTPI);
UECONX |= 1 << STALLRQ; /* return STALL in response to IN token of STATUS stage */

@ Used to set boolean features. The current USB Specification Revision 2 specifies no
interface features.

@<Handle {\caps set feature interface}@>=
UEINTX &= ~(1 << RXSTPI);
UECONX |= 1 << STALLRQ; /* return STALL in response to IN token of STATUS stage */

@ @<Handle {\caps clear feature interface}@>=
UEINTX &= ~(1 << RXSTPI);
UECONX |= 1 << STALLRQ; /* return STALL in response to IN token of STATUS stage */

@ Used to set endpoint features. The standard currently defines one endpoint feature
selector |0x00|, which allows the host to stall and clear an endpoint.
Only endpoints other than the default endpoint are recommended to have this functionality.

@<Handle {\caps set feature endpoint}@>=
  wValue = UEDATX | UEDATX << 8;
  if (wValue == 0) {
    wIndex = UEDATX | UEDATX << 8;
    if (wIndex == 0) {
      UEINTX &= ~(1 << RXSTPI);
      UECONX |= 1 << STALLRQ; /* return STALL in response to IN token of STATUS stage */
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
      UEINTX &= ~(1 << RXSTPI);
      UECONX |= 1 << STALLRQ; /* return STALL in response to IN token of STATUS stage */
    }
  }
  else {
    UEINTX &= ~(1 << RXSTPI);
    UECONX |= 1 << STALLRQ; /* return STALL in response to IN token of STATUS stage */
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
      UEINTX &= ~(1 << RXSTPI);
      UECONX |= 1 << STALLRQ; /* return STALL in response to IN token of STATUS stage */
    }
  }
  else {
    UEINTX &= ~(1 << RXSTPI);
    UECONX |= 1 << STALLRQ; /* return STALL in response to IN token of STATUS stage */
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

TODO: manage here hardware flow control
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

TODO: manage here hardware flow control
@^TODO@>

@<Handle {\caps set control line state}@>=
wValue = UEDATX | UEDATX << 8;
UEINTX &= ~(1 << RXSTPI);
UEINTX &= ~(1 << TXINI); /* STATUS stage */
line_status.all = wValue;

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

@ This request allows the host to select an alternate setting for the specified interface.

\S9.4.10 in USB spec.

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

@* Headers.
\secpagedepth=1 % index on current page

@<Header files@>=
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/power.h> /* |clock_prescale_set| */

@* Index.
