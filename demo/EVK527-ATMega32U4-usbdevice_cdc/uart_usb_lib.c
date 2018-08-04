typedef float Float16;

typedef unsigned char U8 ;
typedef unsigned short U16;
typedef unsigned long U32;
typedef signed char S8 ;
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

typedef union
{
  Uint32 dw;
  Uint16 w[2];
  Uint8 b[4];
} Union32;

typedef union
{
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
typedef enum endpoint_parameter{ep_num, ep_type, ep_direction, ep_size, ep_bank, nyet_status} t_endpoint_parameter;
U8 usb_config_ep (U8, U8);
U8 usb_select_enpoint_interrupt (void);
U16 usb_get_nb_byte_epw (void);
U8 usb_send_packet (U8 , U8*, U8);
U8 usb_read_packet (U8 , U8*, U8);
void usb_halt_endpoint (U8);
void usb_reset_endpoint (U8);
U8 usb_init_device (void);
extern volatile U16 g_usb_event;
extern U8 g_usb_mode;
extern U8 usb_remote_wup_feature;
void usb_task_init (void);
void usb_task (void);

extern volatile U8 private_sof_counter;
void usb_process_request( void);










void usb_generate_remote_wakeup(void);

extern U8 usb_configuration_nb;
extern U8 remote_wakeup_feature;
extern void sof_action(void);
extern void suspend_action(void);
typedef struct
{
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
   U16 wstring[ 5 ];
} S_usb_manufacturer_string_descriptor;






typedef struct {
   U8 bLength;
   U8 bDescriptorType;
   U16 wstring[ 16 ];
} S_usb_product_string_descriptor;






typedef struct {
   U8 bLength;
   U8 bDescriptorType;
   U16 wstring[ 0x05 ];
} S_usb_serial_number;




typedef struct
{
   S_usb_configuration_descriptor cfg;
   S_usb_interface_descriptor ifc0;
   U8 CS_INTERFACE[19];
   S_usb_endpoint_descriptor ep3;
   S_usb_interface_descriptor ifc1;
   S_usb_endpoint_descriptor ep1;
   S_usb_endpoint_descriptor ep2;
} S_usb_user_configuration_descriptor;
void uart_usb_init(void);
U8  uart_usb_test_hit(void);
char uart_usb_getchar(void);
U8  uart_usb_tx_ready(void);
int uart_usb_putchar(int);
void uart_usb_flush(void);
void uart_usb_send_buffer(U8 *buffer, U8 nb_data);






Uchar rx_counter;
void uart_usb_init(void)
{
  rx_counter = 0;
}
U8  uart_usb_test_hit(void)
{
  if (!rx_counter)
  {
    (UENUM = (U8) 0x02 ) ;
    if ( (UEINTX&(1<<RXOUTI)) )
    {
      rx_counter =  ((U8)(UEBCLX)) ;
      if (!rx_counter)
      {
        (UEINTX &= ~(1<<RXOUTI), (UEINTX &= ~(1<<FIFOCON)) ) ;
      }
    }
  }
  return (rx_counter!=0);
}
char uart_usb_getchar(void)
{
  register Uchar data_rx;

  (UENUM = (U8) 0x02 ) ;
  if (!rx_counter) while (!uart_usb_test_hit());
  data_rx= (UEDATX) ;
  rx_counter--;
  if (!rx_counter)  (UEINTX &= ~(1<<RXOUTI), (UEINTX &= ~(1<<FIFOCON)) ) ;
  return data_rx;
}
U8  uart_usb_tx_ready(void)
{
  if (! (UEINTX&(1<<RWAL)) )
  {
    return  (0==1) ;
  }
  return  (1==1) ;
}
int uart_usb_putchar(int data_to_send)
{
   uart_usb_send_buffer((U8*)&data_to_send, 1);
   return data_to_send;
}
void uart_usb_send_buffer(U8 *buffer, U8 nb_data)
{
   U8 zlp;


   if(nb_data% 0x20 )
   { zlp= (0==1) ;}
   else { zlp= (1==1) ; }

   (UENUM = (U8) 0x01 ) ;
   while (nb_data)
   {
      while( (UEINTX&(1<<RWAL)) == (0==1) );
      while( (UEINTX&(1<<RWAL))  && nb_data)
      {
         (UEDATX = (U8)*buffer) ;
         buffer++;
         nb_data--;
   }
      (UEINTX &= ~(1<<TXINI), (UEINTX &= ~(1<<FIFOCON)) ) ;
   }
   if(zlp)
   {
      while( (UEINTX&(1<<RWAL)) == (0==1) );
      (UEINTX &= ~(1<<TXINI), (UEINTX &= ~(1<<FIFOCON)) ) ;
}
}
