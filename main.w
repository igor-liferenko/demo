@ @c
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
typedef unsigned char U8;
typedef unsigned short U16;
typedef unsigned long U32;
typedef unsigned char Bool;
typedef unsigned char Uchar;

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

typedef struct {
  U8 bLength;
  U8 bDescriptorType;
  U16 bscUSB;
  U8 bDeviceClass;
  U8 bDeviceSubClass;
  U8 bDeviceProtocol;
  U8 bMaxPacketSize0;
  U16 idVendor;
  U16 idProduct;
  U16 bcdDevice;
  U8 iManufacturer;
  U8 iProduct;
  U8 iSerialNumber;
  U8 bNumConfigurations;
} S_usb_device_descriptor;

typedef struct {
  U8 bLength;
  U8 bDescriptorType;
  U16 wTotalLength;
  U8 bNumInterfaces;
  U8 bConfigurationValue;
  U8 iConfiguration;
  U8 bmAttibutes;
  U8 MaxPower;
} S_usb_configuration_descriptor;

typedef struct {
  U8 bLength;
  U8 bDescriptorType;
  U8 bInterfaceNumber;
  U8 bAlternateSetting;
  U8 bNumEndpoints;
  U8 bInterfaceClass;
  U8 bInterfaceSubClass;
  U8 bInterfaceProtocol;
  U8 iInterface;
} S_usb_interface_descriptor;

typedef struct {
  U8 bLength;
  U8 bDescriptorType;
  U8 bEndpointAddress;
  U8 bmAttributes;
  U16 wMaxPacketSize;
  U8 bInterval;
} S_usb_endpoint_descriptor;

typedef struct {
  S_usb_configuration_descriptor cfg;
  S_usb_interface_descriptor ifc0;
  U8 CS_INTERFACE[19];
  S_usb_endpoint_descriptor ep3;
  S_usb_interface_descriptor ifc1;
  S_usb_endpoint_descriptor ep1;
  S_usb_endpoint_descriptor ep2;
} S_usb_user_configuration_descriptor;

void (*start_bootloader) (void) = (void (*)(void)) 0x3800;
int uart_usb_putchar(int);
char uart_usb_getchar(void);
U8 flash_read_fuse(unsigned long adr);
Bool usb_user_read_request(U8, U8);
Bool usb_user_get_descriptor(U8, U8);

const S_usb_device_descriptor usb_dev_desc
@t\hskip2.5pt@> @=PROGMEM@> = { @t\1@> @/
  sizeof usb_dev_desc, 0x01, 0x0200, 0x02, 0, 0, 32, 0x03EB, 0x2018, @/
@t\2@> 0x1000, 0x00, 0x00, 0x00, 1 @/
};

const S_usb_user_configuration_descriptor usb_conf_desc
@t\hskip2.5pt@> @=PROGMEM@> = { @t\1@> @/
  {sizeof (S_usb_configuration_descriptor), 0x02, 0x0043, 2, 1, 0, (0x80 | 0x00), 50}, @/
  {sizeof (S_usb_interface_descriptor), 0x04, 0, 0, 1, 0x02, 0x02, 0x01, 0}, @/
  {0x05, 0x24, 0x00, 0x10, 0x01, 0x05, 0x24, 0x01, 0x03, 0x01, 0x04, 0x24, @/
   0x02, 0x06, 0x05, 0x24, 0x06, 0x00, 0x01}, @/
  {sizeof (S_usb_endpoint_descriptor), 0x05, 0x80 | 0x03, 0x03, 0x20, 0xFF}, @/
  {sizeof (S_usb_interface_descriptor), 0x04, 1, 0, 2, 0x0A, 0x00, 0x00, 0}, @/
  {sizeof (S_usb_endpoint_descriptor), 0x05, 0x80 | 0x01, 0x02, 0x20, 0x00}, @/
@t\2@> {sizeof (S_usb_endpoint_descriptor), 0x05, 0x02, 0x02, 0x20, 0x00} @/
};

U8 bmRequestType;
U8 zlp;
U8 endpoint_status[7];
U8 device_status = 1;
U8 data_to_transfer;
PGM_VOID_P pbuffer;
U8 remote_wakeup_feature = 0;
U8 usb_configuration_nb;
U8 usb_remote_wup_feature;
S_serial_state serial_state;
S_serial_state serial_state_saved;
volatile U8 usb_request_break_generation = 0;
volatile U8 rs2usb[10];
volatile U8 cpt_sof;
volatile U16 g_usb_event = 0;
U32 boot_key __attribute__ ((section(".noinit")));
S_line_coding line_coding;
U8 usb_suspended = 0;
U8 usb_connected = 0;
Uchar rx_counter;
S_line_status line_status;

#define EP0 0
#define EP1 1
#define EP2 2
#define EP3 3

int main(void)
{
  UHWCON |= 1 << UVREGE;

  wdt_reset();
  MCUSR = ~(1 << WDRF);
  WDTCSR |= 1 << WDCE;
  WDTCSR = 0x00;

  if (boot_key == 0x55AAAA55) {
    boot_key = 0;
    (*start_bootloader) ();
  }

  clock_prescale_set(0);
  USBCON &= ~(1 << USBE);
  USBCON |= 1 << USBE;
  USBCON |= 1 << OTGPADE;
  USBCON |= 1 << VBUSTE;
  sei();
  usb_remote_wup_feature = 0;
  UBRR1 = (U16) (((U32) 16000 * 1000L) / ((U32) 57600 / 2 * 16) - 1);
  UCSR1A |= 1 << U2X1;
  UCSR1C = 1 << UCSZ11 | 1 << UCSZ10;
  UCSR1B |= 1 << RXEN1 | 1 << TXEN1;
  UCSR1B |= 1 << RXCIE1;
  DDRD |= 1 << PIND5 | 1 << PIND6 | 1 << PIND7;
  DDRC &= ~(1 << PC6);
  PORTC |= 1 << PC6;
  if (flash_read_fuse(0x0003) & 1 << 6) {
    DDRF &= ~(1 << PF4 | 1 << PF5 | 1 << PF6 | 1 << PF7);
    PORTF |= 1 << PF4 | 1 << PF5 | 1 << PF6 | 1 << PF7;
  }
  DDRE &= ~(1 << PINE2), PORTE |= 1 << PINE2;
  UDIEN |= 1 << SOFE;
  fdevopen((int (*)(char, FILE *)) uart_usb_putchar,
           (int (*)(FILE *)) uart_usb_getchar);

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
        UENUM = EP0;
        if (!(UECONX & 1 << EPEN)) {
          UENUM = EP0;
          UECONX |= 1 << EPEN;
          UECFG1X = 1 << EPSIZE1;
          UECFG1X |= 1 << ALLOC;
        }
      }
    }
    if (g_usb_event & 1 << 8) {
      g_usb_event &= ~(1 << 8);
      UERST = 1 << EP0, UERST = 0;
      usb_configuration_nb = 0;
    }
    UENUM = EP0;
    if (UEINTX & 1 << RXSTPI) {
      @<Process SETUP request@>@;
    }
    if (usb_configuration_nb != 0 && line_status.DTR) {
      if (UCSR1A & 0x20) {
        if (!rx_counter) {
          UENUM = EP2;
          if (UEINTX & 1 << RXOUTI) {
            rx_counter = (U8) (UEBCLX);
            if (!rx_counter) {
              UEINTX &= ~(1 << RXOUTI), UEINTX &= ~(1 << FIFOCON);
            }
          }
        }
        if (rx_counter) {
          while (rx_counter) {
            while (!(UCSR1A & 0x20)) ;
            UDR1 = uart_usb_getchar();
            PIND |= 1 << PIND6;
          }
        }
      }
      if (cpt_sof >= 100) {
        if ((0 == (flash_read_fuse(0x0003) & 1 << 6))
            || PINF & 1 << PINF6 ? 0 : 1) {
          printf("Select Pressed !\r\n");
        }
        if ((0 == (flash_read_fuse(0x0003) & 1 << 6))
            || PINF & 1 << PINF7 ? 0 : 1) {
          printf("Right Pressed !\r\n");
          serial_state.bDCD = 1;
        }
        else
          serial_state.bDCD = 0;
        if ((0 == (flash_read_fuse(0x0003) & 1 << 6))
            || PINF & 1 << PINF4 ? 0 : 1) {
          printf("Left Pressed !\r\n");
          serial_state.bDSR = 1;
        }
        else
          serial_state.bDSR = 0;
        if (!(PINC & 1 << PINC6))
          printf("Down Pressed !\r\n");
        if ((0 == (flash_read_fuse(0x0003) & 1 << 6))
            || PINF & 1 << PINF5 ? 0 : 1)
          printf("Up Pressed !\r\n");
        if (!(PINE & 1 << PINE2))
          printf("Hello from ATmega32U4 !\r\n");
        if (serial_state_saved.all != serial_state.all) {
          serial_state_saved.all = serial_state.all;
          UENUM = EP3;
          if (UEINTX & 1 << RWAL) {
            UEDATX = 0xA1;
            UEDATX = 0x20;
            UEDATX = 0x00;
            UEDATX = 0x00;
            UEDATX = 0x00;
            UEDATX = 0x00;
            UEDATX = 0x02;
            UEDATX = 0x00;
            UEDATX = (U8) ((U8 *) & serial_state.all)[0];
            UEDATX = (U8) ((U8 *) & serial_state.all)[1];
            UEINTX &= ~(1 << TXINI), UEINTX &= ~(1 << FIFOCON);
          }
        }
      }
      if (usb_request_break_generation == 1) {
        usb_request_break_generation = 0;
        PIND |= 1 << PIND7;
// !! this is used to reset the chip?
        boot_key = 0x55AAAA55;
        wdt_reset();
        WDTCSR |= 1 << WDCE;
        WDTCSR = 1 << WDE;
        while (1) ;
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

    case 0x2021:
      UEINTX &= ~(1 << RXSTPI);
      while (!(UEINTX & 1 << RXOUTI)) ;
      ((U8 *) & line_coding.dwDTERate)[0] = UEDATX;
      ((U8 *) & line_coding.dwDTERate)[1] = UEDATX;
      ((U8 *) & line_coding.dwDTERate)[2] = UEDATX;
      ((U8 *) & line_coding.dwDTERate)[3] = UEDATX;
      line_coding.bCharFormat = UEDATX;
      line_coding.bParityType = UEDATX;
      line_coding.bDataBits = UEDATX;
      UEINTX &= ~(1 << RXOUTI), UEINTX &= ~(1 << FIFOCON);
      UEINTX &= ~(1 << TXINI);
      while (!(UEINTX & 1 << TXINI)) ;
      UBRR1 =
        (U16) (((U32) 16000 * 1000L) /
               ((U32) line_coding.dwDTERate / 2 * 16) - 1);
      break;
    case 0x2221:
      ((U8 *) & wValue)[0] = UEDATX;
      ((U8 *) & wValue)[1] = UEDATX;
      UEINTX &= ~(1 << RXSTPI);
      UEINTX &= ~(1 << TXINI);
      line_status.all = wValue;
      while (!(UEINTX & 1 << TXINI)) ;
      break;
    case 0x2321:
      UEINTX &= ~(1 << RXSTPI);
      UEINTX &= ~(1 << TXINI);
      usb_request_break_generation = 1;
      while (!(UEINTX & 1 << TXINI)) ;
      break;
    case 0x21A1:
      UEINTX &= ~(1 << RXSTPI);
      UEDATX = (U8) ((U8 *) & line_coding.dwDTERate)[0];
      UEDATX = (U8) ((U8 *) & line_coding.dwDTERate)[1];
      UEDATX = (U8) ((U8 *) & line_coding.dwDTERate)[2];
      UEDATX = (U8) ((U8 *) & line_coding.dwDTERate)[3];
      UEDATX = (U8) line_coding.bCharFormat;
      UEDATX = (U8) line_coding.bParityType;
      UEDATX = (U8) line_coding.bDataBits;
      UEINTX &= ~(1 << TXINI);
      while (!(UEINTX & 1 << TXINI)) ;
      while (!(UEINTX & 1 << RXOUTI)) ;
      UEINTX &= ~(1 << RXOUTI), UEINTX &= ~(1 << FIFOCON);
      break;

void uart_usb_send_buffer(U8 * buffer, U8 nb_data)
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

int uart_usb_putchar(int data_to_send)
{
  uart_usb_send_buffer((U8 *) & data_to_send, 1);
  return data_to_send;
}

char uart_usb_getchar(void)
{
  register Uchar data_rx;

  UENUM = EP2;
  if (!rx_counter) {
    do {
      UENUM = EP2;
      if (UEINTX & 1 << RXOUTI) {
        rx_counter = (U8) UEBCLX;
        if (!rx_counter) {
          UEINTX &= ~(1 << RXOUTI), UEINTX &= ~(1 << FIFOCON);
        }
      }
    } while (!rx_counter);
  }
  data_rx = UEDATX;
  rx_counter--;
  if (!rx_counter)
    UEINTX &= ~(1 << RXOUTI), UEINTX &= ~(1 << FIFOCON);
  return data_rx;
}

ISR(USB_GEN_vect)
{
  if (USBINT & 1 << VBUSTI && USBCON & 1 << VBUSTE) {
    USBINT = ~(1 << VBUSTI);
    if (USBSTA & 1 << VBUS) {
      usb_connected = 1;
      g_usb_event |= 1 << 1;
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
      UENUM = EP0;
      if (!(UECONX & 1 << EPEN)) {
        UENUM = EP0;
        UECONX |= 1 << EPEN;
        UECFG1X = 1 << EPSIZE1;
        UECFG1X |= 1 << ALLOC;
      }
      UDCON &= ~(1 << DETACH);
    }
    else {
      usb_connected = 0;
      usb_configuration_nb = 0;
      g_usb_event |= 1 << 2;
    }
  }
  if (UDINT & 1 << SOFI && UDIEN & 1 << SOFE) {
    UDINT = ~(1 << SOFI);
    cpt_sof++;
  }
  if (UDINT & 1 << SUSPI && UDIEN & 1 << SUSPE) {
    usb_suspended = 1;
    UDINT = ~(1 << WAKEUPI);
    g_usb_event |= 1 << 5;
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
      g_usb_event |= 1 << 6;
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
    g_usb_event |= 1 << 7;
  }
  if (UDINT & 1 << EORSTI && UDIEN & 1 << EORSTE) {
    usb_remote_wup_feature = 0;
    UDINT = ~(1 << EORSTI);
    UENUM = EP0;
    if (!(UECONX & 1 << EPEN)) {
      UENUM = EP0;
      UECONX |= 1 << EPEN;
      UECFG1X = 1 << EPSIZE1;
      UECFG1X |= 1 << ALLOC;
    }
    g_usb_event |= 1 << 8;
  }
}

ISR(USART1_RX_vect)
{
  U8 i = 0;
  U8 save_ep;

  if (usb_configuration_nb != 0) {
    save_ep = UENUM;
    UENUM = EP1;
    do {
      if (UCSR1A & 0x80) {
        rs2usb[i] = UDR1;
        i++;
      }
    } while (!(UEINTX & 1 << RWAL));
    uart_usb_send_buffer((U8 *) & rs2usb, i);
    UENUM = (U8) save_ep;
  }
}

@ The following big switch just dispatches SETUP request.

@<Process SETUP request@>=
U8 addr;
U16 wValue;
U8 wIndex;
U8 configuration_number;
U8 bmRequest;
UEINTX &= ~(1 << RXOUTI); /* TODO: ??? - check if it is non-zero here */
bmRequestType = 0;
bmRequest = 0;
U16 wLength;
U8 dummy;
U8 nb_byte;
zlp = 0;
switch (UEDATX | UEDATX << 8) {
case 0x0680: @/
  switch (UEDATX | UEDATX << 8) {
  case 0x0100: @/
    data_to_transfer = sizeof usb_dev_desc;
    pbuffer = &usb_dev_desc.bLength;
    @<Code which is executed in |0x0680| for both |0x0100| and |0x0200|@>@;
    break;
  case 0x0200: @/
    data_to_transfer = sizeof usb_conf_desc;
    pbuffer = &usb_conf_desc.cfg.bLength;
    @<Code which is executed in |0x0680| for both |0x0100| and |0x0200|@>@;
    break;
  default: /* here go all cases for descriptor\_type different from 0x01 and 0x02 */
    UECONX |= 1 << STALLRQ;
    UEINTX &= ~(1 << RXSTPI);
  }
  break;
case 0x06: /* here go all cases for bmRequestType different from 0x80 */
  usb_user_read_request(bmRequestType, bmRequest);
  break;
case 0x0880: @/
  UEINTX &= ~(1 << RXSTPI);
  UEDATX = (U8) usb_configuration_nb;
  UEINTX &= ~(1 << TXINI), UEINTX &= ~(1 << FIFOCON);
  while (!(UEINTX & 1 << RXOUTI)) ;
  UEINTX &= ~(1 << RXOUTI), UEINTX &= ~(1 << FIFOCON);
  break;
case 0x08: /* here go all cases for bmRequestType different from 0x80 */
  usb_user_read_request(bmRequestType, bmRequest);
  break;
case 0x0500: @/
  addr = UEDATX;
  UDADDR = (UDADDR & 1 << ADDEN) | ((U8) addr & 0x7F);
  UEINTX &= ~(1 << RXSTPI);
  UEINTX &= ~(1 << TXINI);
  while (!(UEINTX & 1 << TXINI)) ;
  UDADDR |= 1 << ADDEN;
  break;
case 0x05: /* here go all cases for bmRequestType different from 0x00 */
  usb_user_read_request(bmRequestType, bmRequest);
  break;
case 0x0900: @/
  configuration_number = UEDATX;
  if (configuration_number <= 1) {
    UEINTX &= ~(1 << RXSTPI);
    usb_configuration_nb = configuration_number;
    UEINTX &= ~(1 << TXINI);

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
  break;
case 0x09: /* here go all cases for bmRequestType different from 0x00 */
  usb_user_read_request(bmRequestType, bmRequest);
  break;
case 0x0100: @/
  UECONX |= 1 << STALLRQ;
  UEINTX &= ~(1 << RXSTPI);
  break;
case 0x0101: @/
  UECONX |= 1 << STALLRQ;
  UEINTX &= ~(1 << RXSTPI);
  break;
case 0x0102: @/
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
      UEINTX &= ~(1 << TXINI);
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
  break;
case 0x01: /* here go all cases for bmRequestType different from 0x00, 0x01 and 0x02 */
  usb_user_read_request(bmRequestType, bmRequest);
  break;
case 0x0300: @/
  UECONX |= 1 << STALLRQ;
  UEINTX &= ~(1 << RXSTPI);
  break;
case 0x0301: @/
  UECONX |= 1 << STALLRQ;
  UEINTX &= ~(1 << RXSTPI);
  break;
case 0x0302:
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
      UEINTX &= ~(1 << TXINI);
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
  break;
case 0x03: /* here go all cases for bmRequestType different from 0x00, 0x01 and 0x02 */
  usb_user_read_request(bmRequestType, bmRequest);
  break;
case 0x0080: @/
  @<Code which is executed in |0x00| for |0x80|, |0x81| and |0x82|@>@;
  break;
case 0x0081: @/
  @<Code which is executed in |0x00| for |0x80|, |0x81| and |0x82|@>@;
  break;
case 0x0082: @/
  @<Code which is executed in |0x00| for |0x80|, |0x81| and |0x82|@>@;
  break;
case 0x00: /* here go all cases for bmRequestType different from 0x80, 0x81 and 0x82 */
  usb_user_read_request(bmRequestType, bmRequest);
  break;
case 0x0A81: @/
  UEINTX &= ~(1 << RXSTPI);
  UEINTX &= ~(1 << TXINI);
  while (!(UEINTX & 1 << RXOUTI)) ;
  UEINTX &= ~(1 << RXOUTI), UEINTX &= ~(1 << FIFOCON);
  break;
case 0x0A: /* here go all cases for bmRequestType different from 0x81 */
  usb_user_read_request(bmRequestType, bmRequest);
  break;
case 0x0B01:
  UEINTX &= ~(1 << RXSTPI);
  UEINTX &= ~(1 << TXINI);
  while (!(UEINTX & 1 << TXINI)) ;
  break;
case 0x0B: /* here go all cases for bmRequestType different from 0x01 */
  break;
default: /* here go all cases for bmRequest different from above */
    UECONX |= 1 << STALLRQ;
    UEINTX &= ~(1 << RXSTPI);
}

@ @<Code which is executed in |0x0680| for both |0x0100| and |0x0200|@>=
    dummy = UEDATX;
    dummy = UEDATX;
    ((U8 *) & wLength)[0] = UEDATX;
    ((U8 *) & wLength)[1] = UEDATX;
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

@ @<Code which is executed in |0x00| for |0x80|, |0x81| and |0x82|@>=
    dummy = UEDATX;
    dummy = UEDATX;
    wIndex = UEDATX;
    switch (bmRequestType) {
    case 0x80:
      UEINTX &= ~(1 << RXSTPI);
      UEDATX = (U8) device_status;
      break;
    case 0x81:
      UEINTX &= ~(1 << RXSTPI);
      UEDATX = (U8) 0x00;
      break;
    case 0x82:
      UEINTX &= ~(1 << RXSTPI);
      wIndex = wIndex & 0x7F;
      UEDATX = (U8) endpoint_status[wIndex];
      break;
    }
    UEDATX = (U8) 0x00;
    UEINTX &= ~(1 << TXINI);
    while (!(UEINTX & 1 << RXOUTI)) ;
    UEINTX &= ~(1 << RXOUTI), UEINTX &= ~(1 << FIFOCON);
