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
U8 flash_read_sig(unsigned long adr);

U8 flash_read_fuse(unsigned long adr);
extern void sof_action(void);
extern void suspend_action(void);
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
extern volatile U16 g_usb_event;
extern U8 g_usb_mode;
extern U8 usb_remote_wup_feature;
void usb_task_init(void);
void usb_task(void);

extern volatile U8 private_sof_counter;
void usb_process_request(void);

void usb_generate_remote_wakeup(void);

extern U8 usb_configuration_nb;
extern U8 remote_wakeup_feature;
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

extern PROGMEM const S_usb_device_descriptor usb_dev_desc;
extern PROGMEM const S_usb_user_configuration_descriptor usb_conf_desc;
extern PROGMEM const S_usb_user_configuration_descriptor
  usb_other_conf_desc;
extern PROGMEM const S_usb_device_qualifier_descriptor usb_qual_desc;
extern PROGMEM const S_usb_manufacturer_string_descriptor
  usb_user_manufacturer_string_descriptor;
extern PROGMEM const S_usb_product_string_descriptor
  usb_user_product_string_descriptor;
extern PROGMEM const S_usb_serial_number usb_user_serial_number;
extern PROGMEM const S_usb_language_id usb_user_language_id;

Bool usb_user_read_request(U8, U8);
Bool usb_user_get_descriptor(U8, U8);
void usb_user_endpoint_init(U8);
void cdc_get_line_coding();
void cdc_set_line_coding();
void cdc_set_control_line_state(U16);
void cdc_send_break(U16);
Bool cdc_update_serial_state();

typedef struct {
  U32 dwDTERate;
  U8 bCharFormat;
  U8 bParityType;
  U8 bDataBits;
} S_line_coding;

typedef union {
  U8 all;
  struct {
    U8 DTR:1;
    U8 RTS:1;
    U8 unused:6;
  };
} S_line_status;

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

void usb_get_descriptor(void);
void usb_set_address(void);
void usb_set_configuration(void);
void usb_clear_feature(void);
void usb_set_feature(void);
void usb_get_status(void);
void usb_get_configuration(void);
void usb_get_interface(void);
void usb_set_interface(void);
extern U8 zlp;
extern U8 endpoint_status[7];
extern U8 device_status;


U16 wInterface;

extern U8 bmRequestType;
U8 remote_wakeup_feature = 0;
U8 usb_configuration_nb;
extern U8 usb_connected;
extern PROGMEM const S_usb_device_descriptor usb_user_device_descriptor;
extern PROGMEM const S_usb_user_configuration_descriptor
  usb_user_configuration_descriptor;

U8 usb_remote_wup_feature;

void usb_get_status(void)
{
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
    return;
  }

  (UEDATX = (U8) 0x00);
  (UEINTX &= ~(1 << TXINI));

  while (!(UEINTX & (1 << RXOUTI))) ;
  (UEINTX &= ~(1 << RXOUTI), (UEINTX &= ~(1 << FIFOCON)));
}

void usb_set_feature(void)
{
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
      break;
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
    break;
  }
}

void usb_clear_feature(void)
{
  U8 wValue;
  U8 wIndex;
  U8 dummy;

  if (bmRequestType == ((0 << 7) | (0 << 5) | (0))) {
    wValue = (UEDATX);
    if ((wValue == 0x01) && (0 == 1)) {
      device_status &= ~0x02;
      remote_wakeup_feature = 0;
      (UEINTX &= ~(1 << RXSTPI));
      (UEINTX &= ~(1 << TXINI));
    }
    else {
      (UECONX |= (1 << STALLRQ));
      (UEINTX &= ~(1 << RXSTPI));
    }
    return;
  }
  else if (bmRequestType == ((0 << 7) | (0 << 5) | (1))) {

    (UECONX |= (1 << STALLRQ));
    (UEINTX &= ~(1 << RXSTPI));
    return;
  }
  else if (bmRequestType == ((0 << 7) | (0 << 5) | (2))) {
    wValue = (UEDATX);
    dummy = (UEDATX);

    if (wValue == 0x00) {
      wIndex = ((UEDATX) & 0x7F);

      (UENUM = (U8) wIndex);
      if (((UECONX & (1 << EPEN)) ? (1 == 1) : (0 == 1))) {
        if (wIndex != 0) {
          (UECONX |= (1 << STALLRQC));
          (UERST = 1 << (U8) wIndex, UERST = 0);
          (UECONX |= (1 << RSTDT));
        }
        (UENUM = (U8) 0);
        endpoint_status[wIndex] = 0x00;
        (UEINTX &= ~(1 << RXSTPI));
        (UEINTX &= ~(1 << TXINI));
      }
      else {
        (UENUM = (U8) 0);
        (UECONX |= (1 << STALLRQ));
        (UEINTX &= ~(1 << RXSTPI));
        return;
      }
    }
    else {
      (UECONX |= (1 << STALLRQ));
      (UEINTX &= ~(1 << RXSTPI));
      return;
    }
  }
}
