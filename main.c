typedef float Float16;

typedef unsigned char U8;
typedef unsigned short U16;
typedef unsigned long U32;
typedef signed char S8;
typedef signed short S16;
typedef long S32;

typedef unsigned char Bool;

typedef U8 Status;
typedef Bool Status_bool;
typedef unsigned char Uchar;

typedef unsigned char Uint8;
typedef unsigned int Uint16;
typedef unsigned long int Uint32;

typedef char Int8;
typedef int Int16;
typedef long int Int32;

typedef unsigned char Byte;
typedef unsigned int Word;
typedef unsigned long int DWord;

typedef union {
  Uint32 dw;
  Uint16 w[2];
  Uint8 b[4];
} Union32;

typedef union {
  Uint16 w;
  Uint8 b[2];
} Union16;
typedef char p_uart_ptchar;
typedef char r_uart_ptchar;
#include  <avr/interrupt.h>
#include  <avr/pgmspace.h>
#include  <avr/io.h>
#include  <stdio.h>
U8 flash_read_sig(unsigned long adr);

U8 flash_read_fuse(unsigned long adr);
extern void usb_task_init(void);
extern void cdc_task_init(void);
extern void usb_task(void);
extern void cdc_task(void);
void scheduler_init(void);
void scheduler_tasks(void);
void scheduler(void);
void scheduler_empty_fct(void);
#include  <avr/io.h>
#include  <avr/wdt.h>
#include  <avr/power.h>
void set_idle_mode(void);
void set_power_down_mode(void);
void set_adc_noise_reduction_mode(void);
void set_power_save_mode(void);
void set_standby_mode(void);
void set_ext_standby_mode(void);
void Clock_switch_external(void);
void Clock_switch_internal(void);
typedef enum endpoint_parameter { ep_num, ep_type, ep_direction, ep_size,
  ep_bank, nyet_status
} t_endpoint_parameter;
U8 usb_config_ep(U8, U8);
U8 usb_select_enpoint_interrupt(void);
U16 usb_get_nb_byte_epw(void);
U8 usb_send_packet(U8, U8 *, U8);
U8 usb_read_packet(U8, U8 *, U8);
void usb_halt_endpoint(U8);
void usb_reset_endpoint(U8);
U8 usb_init_device(void);
U32 boot_key __attribute__ ((section(".noinit")));
void start_boot_if_required(void);
void start_boot(void);

void (*start_bootloader) (void) = (void (*)(void)) 0x3800;
extern U8 usb_remote_wup_feature;
char uart_usb_getchar(void);
int uart_usb_putchar(int);
extern U8 usb_suspended;
extern U8 usb_configuration_nb;
volatile U16 g_usb_event = 0;
extern U8 usb_connected;
void usb_device_task(void);
void usb_start_device(void);
extern void sof_action(void);
extern volatile U8 usb_request_break_generation;
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
S_serial_state serial_state;
typedef union {
  U8 all;
  struct {
    U8 DTR:1;
    U8 RTS:1;
    U8 unused:6;
  };
} S_line_status;
S_line_status line_status;
U8 uart_usb_test_hit(void);
extern U8 rx_counter;
int uart_putchar(int uc_wr_byte);
volatile U8 cpt_sof;
Bool cdc_update_serial_state();
void usb_process_request(void);
volatile U8 rs2usb[10];
void uart_usb_send_buffer(U8 * buffer, U8 nb_data);
typedef struct {
  U32 dwDTERate;
  U8 bCharFormat;
  U8 bParityType;
  U8 bDataBits;
} S_line_coding;
S_line_coding line_coding;
U8 usb_suspended = 0;
U8 usb_connected = 0;
Uchar rx_counter;
typedef struct {
  U8 bmRequestType;
  U8 bRequest;
  U16 wValue;
  U16 wIndex;
  U16 wLength;
} S_UsbRequest;

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
  U8 bLength;
  U8 bDescriptorType;
  U16 bscUSB;
  U8 bDeviceClass;
  U8 bDeviceSubClass;
  U8 bDeviceProtocol;
  U8 bMaxPacketSize0;
  U8 bNumConfigurations;
  U8 bReserved;
} S_usb_device_qualifier_descriptor;

typedef struct {
  U8 bLength;
  U8 bDescriptorType;
  U16 wlangid;
} S_usb_language_id;

typedef struct {
  U8 bLength;
  U8 bDescriptorType;
  U16 wstring[5];
} S_usb_manufacturer_string_descriptor;

typedef struct {
  U8 bLength;
  U8 bDescriptorType;
  U16 wstring[16];
} S_usb_product_string_descriptor;

typedef struct {
  U8 bLength;
  U8 bDescriptorType;
  U16 wstring[0x05];
} S_usb_serial_number;

typedef struct {
  S_usb_configuration_descriptor cfg;
  S_usb_interface_descriptor ifc0;
  U8 CS_INTERFACE[19];
  S_usb_endpoint_descriptor ep3;
  S_usb_interface_descriptor ifc1;
  S_usb_endpoint_descriptor ep1;
  S_usb_endpoint_descriptor ep2;
} S_usb_user_configuration_descriptor;

PROGMEM const S_usb_device_descriptor usb_dev_desc = {
  sizeof(usb_dev_desc)
    , 0x01, (0x0200)
    , 0x02, 0, 0, 32, (0x03EB)
    , (0x2018)
    , (0x1000)
  , 0x00, 0x00, 0x00, 1
};

PROGMEM const S_usb_user_configuration_descriptor usb_conf_desc = {
  {sizeof(S_usb_configuration_descriptor)
   , 0x02, 0x0043, 2, 1, 0, (0x80 | 0x00)
   , 50}
  ,
  {sizeof(S_usb_interface_descriptor)
   , 0x04, 0, 0, 1, 0x02, 0x02, 0x01, 0}
  ,
  {0x05, 0x24, 0x00, 0x10, 0x01, 0x05, 0x24, 0x01, 0x03, 0x01, 0x04, 0x24,
   0x02, 0x06, 0x05, 0x24, 0x06, 0x00, 0x01}
  ,
  {sizeof(S_usb_endpoint_descriptor)
   , 0x05, 0x80 | 0x03, 0x03, (0x20)
   , 0xFF}
  ,
  {sizeof(S_usb_interface_descriptor)
   , 0x04, 1, 0, 2, 0x0A, 0x00, 0x00, 0}
  ,
  {sizeof(S_usb_endpoint_descriptor)
   , 0x05, 0x80 | 0x01, 0x02, (0x20)
   , 0x00}
  ,
  {sizeof(S_usb_endpoint_descriptor)
   , 0x05, 0x02, 0x02, (0x20)
   , 0x00}

};

PROGMEM const S_usb_manufacturer_string_descriptor
  usb_user_manufacturer_string_descriptor = {
  sizeof(usb_user_manufacturer_string_descriptor)
    , 0x03, {((U16) ('A')), ((U16) ('T')), ((U16) ('M')), ((U16) ('E')),
             ((U16) ('L'))}
};

PROGMEM const S_usb_product_string_descriptor
  usb_user_product_string_descriptor = {
  sizeof(usb_user_product_string_descriptor)
    , 0x03, {((U16) ('A')), ((U16) ('V')), ((U16) ('R')), ((U16) (' ')),
             ((U16) ('U')), ((U16) ('S')), ((U16) ('B')), ((U16) (' ')),
             ((U16) ('C')), ((U16) ('D')), ((U16) ('C')), ((U16) (' ')),
             ((U16) ('D')), ((U16) ('E')), ((U16) ('M')), ((U16) ('O'))}
};

PROGMEM const S_usb_serial_number usb_user_serial_number = {
  sizeof(usb_user_serial_number)
    , 0x03, {((U16) ('1')), ((U16) ('.')), ((U16) ('0')), ((U16) ('.')),
             ((U16) ('0'))}
};

PROGMEM const S_usb_language_id usb_user_language_id = {
  sizeof(usb_user_language_id)
    , 0x03, (0x0409)
};

U8 bmRequestType;
U8 zlp;
U8 endpoint_status[7];
U8 device_status = 1;
void usb_get_descriptor(void);
void usb_set_address(void);
void usb_set_configuration(void);
void usb_clear_feature(void);
void usb_set_feature(void);
void usb_get_status(void);
void usb_get_configuration(void);
void usb_get_interface(void);
void usb_set_interface(void);
Bool usb_user_read_request(U8, U8);
void usb_user_endpoint_init(U8);
U8 data_to_transfer;
PGM_VOID_P pbuffer;
Bool usb_user_get_descriptor(U8, U8);
U8 remote_wakeup_feature = 0;

int main(void)
{
  (UHWCON |= (1 << UVREGE));

  wdt_reset();
  ((MCUSR = ~(1 << WDRF)));
  (WDTCSR |= (1 << WDCE));
  (WDTCSR = 0x00);

  if (boot_key == 0x55AAAA55) {
    boot_key = 0;
    (*start_bootloader) ();
  }

  (clock_prescale_set(0));
  (USBCON &= ~((1 << USBE)));
  (USBCON |= ((1 << USBE)));
  (USBCON |= (1 << OTGPADE));
  (USBCON |= (1 << VBUSTE));
  sei();
  usb_remote_wup_feature = 0;
  ((UBRR1) = (U16) (((U32) 16000 * 1000L) / ((U32) 57600 / 2 * 16) - 1));
  ((UCSR1A) |= (1 << U2X1));
  ((UCSR1C) = 0x06);
  ((UCSR1B) |= 0x10 | 0x08);
  ((UCSR1B) |= 0x80);
  (DDRD |= (1 << PIND5) | (1 << PIND6) | (1 << PIND7));
  DDRC &= ~0x40;
  PORTC |= 0x40;
  if (!(0 == ((flash_read_fuse(0x0003)) & (1 << 6)))) {
    DDRF &= ~0xF0;
    PORTF |= 0xF0;
  }
  (DDRE &= ~(1 << PINE2), PORTE |= (1 << PINE2));
  (UDIEN |= (1 << SOFE));
  fdevopen((int (*)(char, FILE *)) (uart_usb_putchar),
           (int (*)(FILE *)) uart_usb_getchar);

  while (1) {
    if (usb_connected == (0 == 1)) {
      if (((USBSTA & (1 << VBUS)) ? (1 == 1) : (0 == 1))) {
        (USBCON |= ((1 << USBE)));
        usb_connected = (1 == 1);
        (USBCON |= (1 << FRZCLK));

        (PLLFRQ &=
         ~((1 << PDIV3) | (1 << PDIV2) | (1 << PDIV1) | (1 << PDIV0)),
         PLLFRQ |=
         ((0 << PDIV3) | (1 << PDIV2) | (0 << PDIV1) | (0 << PDIV0)) | (0
                                                                        <<
                                                                        PLLUSB),
         PLLCSR = ((1 << PINDIV) | (1 << PLLE)));
        while (!(PLLCSR & (1 << PLOCK))) ;
        (USBCON &= ~(1 << FRZCLK));
        (UDCON &= ~(1 << DETACH));

        (UDCON &= ~(1 << RSTCPU));

        (UDIEN |= (1 << SUSPE));
        (UDIEN |= (1 << EORSTE));
        sei();
        (UENUM = (U8) 0);
        if (!(UECONX & 1 << EPEN)) {
          UENUM = (U8) 0;
          (UECONX |= (1 << EPEN));
          UECFG0X = 0 << 6 | 1 << 1 | 0;
          UECFG1X = (UECFG1X & 1 << ALLOC) | 2 << 4 | 0 << 2;
          (UECFG1X |= (1 << ALLOC));
        }
      }
    }
    if (((g_usb_event & (1 << 8)) ? (1 == 1) : (0 == 1))) {
      (g_usb_event &= ~(1 << 8));
      (UERST = 1 << (U8) 0, UERST = 0);
      usb_configuration_nb = 0;
    }
    (UENUM = (U8) 0);
    if ((UEINTX & (1 << RXSTPI))) {
      U8 bmRequest;

      (UEINTX &= ~(1 << RXOUTI));
      bmRequestType = (UEDATX);
      bmRequest = (UEDATX);

      switch (bmRequest) {
      case 0x06:
        if (((1 << 7) | (0 << 5) | (0)) == bmRequestType) {
          U16 wLength;
          U8 descriptor_type;
          U8 string_type;
          U8 dummy;
          U8 nb_byte;
          zlp = (0 == 1);
          string_type = (UEDATX);
          descriptor_type = (UEDATX);
          switch (descriptor_type) {
          case 0x01:
            data_to_transfer = (sizeof(usb_dev_desc));
            pbuffer = (&(usb_dev_desc.bLength));
            break;
          case 0x02:
            data_to_transfer = (sizeof(usb_conf_desc));
            pbuffer = (&(usb_conf_desc.cfg.bLength));
            break;
          default:
            if (usb_user_get_descriptor(descriptor_type, string_type) ==
                (0 == 1)) {
              (UECONX |= (1 << STALLRQ));
              (UEINTX &= ~(1 << RXSTPI));
              goto out_get_descriptor;
            }
          }
          dummy = (UEDATX);
          dummy = (UEDATX);
          (((U8 *) & wLength)[0]) = (UEDATX);
          (((U8 *) & wLength)[1]) = (UEDATX);
          (UEINTX &= ~(1 << RXSTPI));
          if (wLength > data_to_transfer) {
            if ((data_to_transfer % 32) == 0) {
              zlp = (1 == 1);
            }
            else {
              zlp = (0 == 1);
            }
          }
          else {
            data_to_transfer = (U8) wLength;
          }
          (UEINTX &= ~(1 << NAKOUTI));
          while ((data_to_transfer != 0) && (!(UEINTX & (1 << NAKOUTI)))) {
            while (!(UEINTX & (1 << TXINI))) {
              if ((UEINTX & (1 << NAKOUTI)))
                break;
            }
            nb_byte = 0;
            while (data_to_transfer != 0) {
              if (nb_byte++ == 32) {
                break;
              }
              (UEDATX = (U8) pgm_read_byte_near((unsigned int) pbuffer++));
              data_to_transfer--;
            }
            if ((UEINTX & (1 << NAKOUTI)))
              break;
            else
              (UEINTX &= ~(1 << TXINI));
          }
          if ((zlp == (1 == 1)) && (!(UEINTX & (1 << NAKOUTI)))) {
            while (!(UEINTX & (1 << TXINI))) ;
            (UEINTX &= ~(1 << TXINI));
          }
          while (!((UEINTX & (1 << NAKOUTI)))) ;
          (UEINTX &= ~(1 << NAKOUTI));
          (UEINTX &= ~(1 << RXOUTI));
        out_get_descriptor:;
        }
        else {
          usb_user_read_request(bmRequestType, bmRequest);
        }
        break;

      case 0x08:
        if (((1 << 7) | (0 << 5) | (0)) == bmRequestType) {
          (UEINTX &= ~(1 << RXSTPI));
          (UEDATX = (U8) usb_configuration_nb);
          (UEINTX &= ~(1 << TXINI), (UEINTX &= ~(1 << FIFOCON)));
          while (!(UEINTX & (1 << RXOUTI))) ;
          (UEINTX &= ~(1 << RXOUTI), (UEINTX &= ~(1 << FIFOCON)));
        }
        else {
          usb_user_read_request(bmRequestType, bmRequest);
        }
        break;

      case 0x05:
        if (((0 << 7) | (0 << 5) | (0)) == bmRequestType) {
          U8 addr = (UEDATX);
          (UDADDR = (UDADDR & (1 << ADDEN)) | ((U8) addr & 0x7F));

          (UEINTX &= ~(1 << RXSTPI));

          (UEINTX &= ~(1 << TXINI));
          while (!(UEINTX & (1 << TXINI))) ;

          (UDADDR |= (1 << ADDEN));
        }
        else {
          usb_user_read_request(bmRequestType, bmRequest);
        }
        break;

      case 0x09:
        if (((0 << 7) | (0 << 5) | (0)) == bmRequestType) {
          U8 configuration_number;

          configuration_number = (UEDATX);

          if (configuration_number <= 1) {
            (UEINTX &= ~(1 << RXSTPI));
            usb_configuration_nb = configuration_number;
            (UEINTX &= ~(1 << TXINI));
            usb_user_endpoint_init(usb_configuration_nb);
          }
          else {
            (UECONX |= (1 << STALLRQ));
            (UEINTX &= ~(1 << RXSTPI));
          }
        }
        else {
          usb_user_read_request(bmRequestType, bmRequest);
        }
        break;

      case 0x01:
        if (((0 << 7) | (0 << 5) | (2)) >= bmRequestType) {
          usb_clear_feature();
        }
        else {
          usb_user_read_request(bmRequestType, bmRequest);
        }
        break;

      case 0x03:
        if (((0 << 7) | (0 << 5) | (2)) >= bmRequestType) {
          U8 wValue;
          U8 wIndex;
          U8 dummy;

          switch (bmRequestType) {
          case ((0 << 7) | (0 << 5) | (0)):
            wValue = (UEDATX);
            switch (wValue) {
            case 1:
              if ((wValue == 0x01) && (0 == 1)) {
                device_status |= 0x02;
                remote_wakeup_feature = 1;
                (UEINTX &= ~(1 << RXSTPI));
                (UEINTX &= ~(1 << TXINI));
              }
              else {
                (UECONX |= (1 << STALLRQ));
                (UEINTX &= ~(1 << RXSTPI));
              }
              break;

            default:
              (UECONX |= (1 << STALLRQ));
              (UEINTX &= ~(1 << RXSTPI));
            }
            break;

          case ((0 << 7) | (0 << 5) | (1)):

            (UECONX |= (1 << STALLRQ));
            (UEINTX &= ~(1 << RXSTPI));
            break;

          case ((0 << 7) | (0 << 5) | (2)):
            wValue = (UEDATX);
            dummy = (UEDATX);

            if (wValue == 0x00) {
              wIndex = ((UEDATX) & 0x7F);

              if (wIndex == 0) {
                (UECONX |= (1 << STALLRQ));
                (UEINTX &= ~(1 << RXSTPI));
              }

              (UENUM = (U8) wIndex);
              if (((UECONX & (1 << EPEN)) ? (1 == 1) : (0 == 1))) {
                (UECONX |= (1 << STALLRQ));
                (UENUM = (U8) 0);
                endpoint_status[wIndex] = 0x01;
                (UEINTX &= ~(1 << RXSTPI));
                (UEINTX &= ~(1 << TXINI));
              }
              else {
                (UENUM = (U8) 0);
                (UECONX |= (1 << STALLRQ));
                (UEINTX &= ~(1 << RXSTPI));
              }
            }
            else {
              (UECONX |= (1 << STALLRQ));
              (UEINTX &= ~(1 << RXSTPI));
            }
            break;

          default:
            (UECONX |= (1 << STALLRQ));
            (UEINTX &= ~(1 << RXSTPI));
          }

        }
        else {
          usb_user_read_request(bmRequestType, bmRequest);
        }
        break;

      case 0x00:
        if ((0x7F < bmRequestType) & (0x82 >= bmRequestType)) {
          U8 wIndex;
          U8 dummy;

          dummy = (UEDATX);
          dummy = (UEDATX);
          wIndex = (UEDATX);

          switch (bmRequestType) {
          case ((1 << 7) | (0 << 5) | (0)):
            (UEINTX &= ~(1 << RXSTPI));
            (UEDATX = (U8) device_status);
            break;
          case ((1 << 7) | (0 << 5) | (1)):
            (UEINTX &= ~(1 << RXSTPI));
            (UEDATX = (U8) 0x00);
            break;
          case ((1 << 7) | (0 << 5) | (2)):
            (UEINTX &= ~(1 << RXSTPI));
            wIndex = wIndex & 0x7F;
            (UEDATX = (U8) endpoint_status[wIndex]);
            break;
          default:
            (UECONX |= (1 << STALLRQ));
            (UEINTX &= ~(1 << RXSTPI));
            goto out_get_status;
          }

          (UEDATX = (U8) 0x00);
          (UEINTX &= ~(1 << TXINI));

          while (!(UEINTX & (1 << RXOUTI))) ;
          (UEINTX &= ~(1 << RXOUTI), (UEINTX &= ~(1 << FIFOCON)));
        out_get_status:;
        }
        else {
          usb_user_read_request(bmRequestType, bmRequest);
        }
        break;

      case 0x0A:
        if (bmRequestType == ((1 << 7) | (0 << 5) | (1))) {
          (UEINTX &= ~(1 << RXSTPI));
          (UEINTX &= ~(1 << TXINI));
          while (!(UEINTX & (1 << RXOUTI))) ;
          (UEINTX &= ~(1 << RXOUTI), (UEINTX &= ~(1 << FIFOCON)));
        }
        else {
          usb_user_read_request(bmRequestType, bmRequest);
        }
        break;

      case 0x0B:
        if (bmRequestType == ((0 << 7) | (0 << 5) | (1))) {
          (UEINTX &= ~(1 << RXSTPI));
          (UEINTX &= ~(1 << TXINI));
          while (!(UEINTX & (1 << TXINI))) ;
        }
        break;

      case 0x07:
      case 0x0C:
      default:
        if (usb_user_read_request(bmRequestType, bmRequest) == (0 == 1)) {
          (UECONX |= (1 << STALLRQ));
          (UEINTX &= ~(1 << RXSTPI));
        }
      }
    }
    if (((usb_configuration_nb != 0) ? (1 == 1) : (0 == 1))
        && line_status.DTR) {
      if (((UCSR1A) & 0x20)) {
        if (!rx_counter) {
          (UENUM = (U8) 0x02);
          if ((UEINTX & (1 << RXOUTI))) {
            rx_counter = ((U8) (UEBCLX));
            if (!rx_counter) {
              (UEINTX &= ~(1 << RXOUTI), (UEINTX &= ~(1 << FIFOCON)));
            }
          }
        }
        if (rx_counter) {
          while (rx_counter) {
            while (!(UCSR1A & 0x20)) ;
            UDR1 = uart_usb_getchar();
            (PIND |= (1 << PIND6));
          }
        }
      }

      if (cpt_sof >= 100) {
        if (((0 == ((flash_read_fuse(0x0003)) & (1 << 6)))
             || (PINF & (1 << PINF6)) ? (0 == 1) : (1 == 1))) {
          printf("Select Pressed !\r\n");
        }
        if (((0 == ((flash_read_fuse(0x0003)) & (1 << 6)))
             || (PINF & (1 << PINF7)) ? (0 == 1) : (1 == 1))) {
          printf("Right Pressed !\r\n");
          serial_state.bDCD = (1 == 1);
        }
        else
          serial_state.bDCD = (0 == 1);

        if (((0 == ((flash_read_fuse(0x0003)) & (1 << 6)))
             || (PINF & (1 << PINF4)) ? (0 == 1) : (1 == 1))) {
          printf("Left Pressed !\r\n");
          serial_state.bDSR = (1 == 1);
        }
        else
          serial_state.bDSR = (0 == 1);

        if (((PINC & (1 << PINC6)) ? (0 == 1) : (1 == 1)))
          printf("Down Pressed !\r\n");

        if (((0 == ((flash_read_fuse(0x0003)) & (1 << 6)))
             || (PINF & (1 << PINF5)) ? (0 == 1) : (1 == 1)))
          printf("Up Pressed !\r\n");

        if (((PINE & (1 << PINE2)) ? (0 == 1) : (1 == 1)))
          printf("Hello from ATmega32U4 !\r\n");

        cdc_update_serial_state();
      }

      if (usb_request_break_generation == (1 == 1)) {
        usb_request_break_generation = (0 == 1);
        (PIND |= (1 << PIND7));
// !! this is used to reset the chip?
        boot_key = 0x55AAAA55;
        wdt_reset();
        (WDTCSR |= (1 << WDCE));
        (WDTCSR = (1 << WDE));
        while (1) ;
      }
    }
  }
  return 0;
}

char __low_level_init(void) __attribute__ ((section(".init3"), naked));
char __low_level_init()
{
  (clock_prescale_set(0));
  return 1;
}

void uart_usb_send_buffer(U8 * buffer, U8 nb_data)
{
  U8 zlp;

  if (nb_data % 0x20) {
    zlp = (0 == 1);
  }
  else {
    zlp = (1 == 1);
  }

  (UENUM = (U8) 0x01);
  while (nb_data) {
    while ((UEINTX & (1 << RWAL)) == (0 == 1)) ;
    while ((UEINTX & (1 << RWAL)) && nb_data) {
      (UEDATX = (U8) * buffer);
      buffer++;
      nb_data--;
    }
    (UEINTX &= ~(1 << TXINI), (UEINTX &= ~(1 << FIFOCON)));
  }
  if (zlp) {
    while ((UEINTX & (1 << RWAL)) == (0 == 1)) ;
    (UEINTX &= ~(1 << TXINI), (UEINTX &= ~(1 << FIFOCON)));
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

  (UENUM = (U8) 0x02);
  if (!rx_counter) {
    do {
      (UENUM = (U8) 0x02);
      if ((UEINTX & (1 << RXOUTI))) {
        rx_counter = ((U8) (UEBCLX));
        if (!rx_counter) {
          (UEINTX &= ~(1 << RXOUTI), (UEINTX &= ~(1 << FIFOCON)));
        }
      }
    } while (!rx_counter);
  }
  data_rx = (UEDATX);
  rx_counter--;
  if (!rx_counter)
    (UEINTX &= ~(1 << RXOUTI), (UEINTX &= ~(1 << FIFOCON)));
  return data_rx;
}

ISR(USB_GEN_vect)
{
  if (((USBINT & (1 << VBUSTI)) ? (1 == 1) : (0 == 1))
      && ((USBCON & (1 << VBUSTE)) ? (1 == 1) : (0 == 1))) {
    (USBINT = ~(1 << VBUSTI));
    if (((USBSTA & (1 << VBUS)) ? (1 == 1) : (0 == 1))) {
      usb_connected = (1 == 1);
      (g_usb_event |= (1 << 1));
      (UDIEN |= (1 << EORSTE));
      (USBCON |= (1 << FRZCLK));

      (PLLFRQ &=
       ~((1 << PDIV3) | (1 << PDIV2) | (1 << PDIV1) | (1 << PDIV0)),
       PLLFRQ |=
       ((0 << PDIV3) | (1 << PDIV2) | (0 << PDIV1) | (0 << PDIV0)) | (0 <<
                                                                      PLLUSB),
       PLLCSR = ((1 << PINDIV) | (1 << PLLE)));
      while (!(PLLCSR & (1 << PLOCK))) ;
      (USBCON &= ~(1 << FRZCLK));
      (UDCON &= ~(1 << DETACH));

      (UDCON &= ~(1 << RSTCPU));

      (UDIEN |= (1 << SUSPE));
      (UDIEN |= (1 << EORSTE));
      sei();
      (UENUM = (U8) 0);
      if (!(UECONX & 1 << EPEN)) {
        UENUM = (U8) 0;
        (UECONX |= (1 << EPEN));
        UECFG0X = 0 << 6 | 1 << 1 | 0;
        UECFG1X = (UECFG1X & 1 << ALLOC) | 2 << 4 | 0 << 2;
        (UECFG1X |= (1 << ALLOC));
      }
      (UDCON &= ~(1 << DETACH));
    }
    else {
      usb_connected = (0 == 1);
      usb_configuration_nb = 0;
      (g_usb_event |= (1 << 2));
    }
  }
  if (((UDINT & (1 << SOFI)) ? (1 == 1) : (0 == 1))
      && ((UDIEN & (1 << SOFE)) ? (1 == 1) : (0 == 1))) {
    (UDINT = ~(1 << SOFI));
    cpt_sof++;
  }
  if (((UDINT & (1 << SUSPI)) ? (1 == 1) : (0 == 1))
      && ((UDIEN & (1 << SUSPE)) ? (1 == 1) : (0 == 1))) {
    usb_suspended = (1 == 1);
    (UDINT = ~(1 << WAKEUPI));
    (g_usb_event |= (1 << 5));
    (UDINT = ~(1 << SUSPI));
    (UDIEN |= (1 << WAKEUPE));
    (UDIEN &= ~(1 << EORSME));
    (USBCON |= (1 << FRZCLK));
    (PLLCSR &= (~(1 << PLLE)), PLLCSR = 0);
  }
  if (((UDINT & (1 << WAKEUPI)) ? (1 == 1) : (0 == 1))
      && ((UDIEN & (1 << WAKEUPE)) ? (1 == 1) : (0 == 1))) {
    if ((PLLCSR & (1 << PLOCK)) == (0 == 1)) {
      (PLLFRQ &=
       ~((1 << PDIV3) | (1 << PDIV2) | (1 << PDIV1) | (1 << PDIV0)),
       PLLFRQ |=
       ((0 << PDIV3) | (1 << PDIV2) | (0 << PDIV1) | (0 << PDIV0)) | (0 <<
                                                                      PLLUSB),
       PLLCSR = ((1 << PINDIV) | (1 << PLLE)));

      while (!(PLLCSR & (1 << PLOCK))) ;
    }
    (USBCON &= ~(1 << FRZCLK));
    (UDINT = ~(1 << WAKEUPI));
    if (usb_suspended) {
      (UDIEN |= (1 << EORSME));
      (UDIEN |= (1 << EORSTE));
      (UDINT = ~(1 << WAKEUPI));
      (UDIEN &= ~(1 << WAKEUPE));
      (g_usb_event |= (1 << 6));
      (UDIEN |= (1 << SUSPE));
      (UDIEN |= (1 << EORSME));
      (UDIEN |= (1 << EORSTE));
    }
  }
  if (((UDINT & (1 << EORSMI)) ? (1 == 1) : (0 == 1))
      && ((UDIEN & (1 << EORSME)) ? (1 == 1) : (0 == 1))) {
    usb_suspended = (0 == 1);
    (UDIEN &= ~(1 << WAKEUPE));
    (UDINT = ~(1 << EORSMI));
    (UDIEN &= ~(1 << EORSME));
    (g_usb_event |= (1 << 7));
  }
  if (((UDINT & (1 << EORSTI)) ? (1 == 1) : (0 == 1))
      && ((UDIEN & (1 << EORSTE)) ? (1 == 1) : (0 == 1))) {
    usb_remote_wup_feature = 0;
    (UDINT = ~(1 << EORSTI));
    (UENUM = (U8) 0);
    if (!(UECONX & 1 << EPEN)) {
      UENUM = (U8) 0;
      (UECONX |= (1 << EPEN));
      UECFG0X = 0 << 6 | 1 << 1 | 0;
      UECFG1X = (UECFG1X & (1 << ALLOC)) | 2 << 4 | 0 << 2;
      (UECFG1X |= (1 << ALLOC));
    }
    (g_usb_event |= (1 << 8));
  }
}

ISR(USART1_RX_vect)
{
  U8 i = 0;
  U8 save_ep;

  if (((usb_configuration_nb != 0) ? (1 == 1) : (0 == 1))) {
    save_ep = (UENUM);
    (UENUM = (U8) 0x01);
    do {
      if (((UCSR1A) & 0x80)) {
        rs2usb[i] = ((UDR1));
        i++;
      }
    } while ((UEINTX & (1 << RWAL)) == (0 == 1));
    uart_usb_send_buffer((U8 *) & rs2usb, i);
    (UENUM = (U8) save_ep);
  }
}
