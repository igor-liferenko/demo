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

U8 usb_select_enpoint_interrupt(void)
{
  U8 interrupt_flags;
  U8 ep_num;

  ep_num = 0;
  interrupt_flags = (UEINT);

  while (ep_num < 7) {
    if (interrupt_flags & 1) {
      return (ep_num);
    }
    else {
      ep_num++;
      interrupt_flags = interrupt_flags >> 1;
    }
  }
  return 0;
}

U8 usb_send_packet(U8 ep_num, U8 * tbuf, U8 data_length)
{
  U8 remaining_length;

  remaining_length = data_length;
  (UENUM = (U8) ep_num);
  while ((UEINTX & (1 << RWAL)) && (0 != remaining_length)) {
    (UEDATX = (U8) * tbuf);
    remaining_length--;
    tbuf++;
  }
  return remaining_length;
}

U8 usb_read_packet(U8 ep_num, U8 * rbuf, U8 data_length)
{
  U8 remaining_length;

  remaining_length = data_length;
  (UENUM = (U8) ep_num);

  while ((UEINTX & (1 << RWAL)) && (0 != remaining_length)) {
    *rbuf = (UEDATX);
    remaining_length--;
    rbuf++;
  }
  return remaining_length;
}

void usb_halt_endpoint(U8 ep_num)
{
  (UENUM = (U8) ep_num);
  (UECONX |= (1 << STALLRQ));
}
