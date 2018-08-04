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
    ep_bank, nyet_status } t_endpoint_parameter;
U8 usb_config_ep(U8, U8);
U8 usb_select_enpoint_interrupt(void);
U16 usb_get_nb_byte_epw(void);
U8 usb_send_packet(U8, U8 *, U8);
U8 usb_read_packet(U8, U8 *, U8);
void usb_halt_endpoint(U8);
void usb_reset_endpoint(U8);
U8 usb_init_device(void);
extern U32 boot_key __attribute__ ((section(".noinit")));
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
  {
    DDRC &= ~0x40;
    PORTC |= 0x40;
    if (!(0 == ((flash_read_fuse(0x0003)) & (1 << 6)))) {
      DDRF &= ~0xF0;
      PORTF |= 0xF0;
    }
  };
  (DDRE &= ~(1 << PINE2), PORTE |= (1 << PINE2));
  (UDIEN |= (1 << SOFE));
  fdevopen((int (*)(char, FILE *)) (uart_usb_putchar),
           (int (*)(FILE *)) uart_usb_getchar);
  while (1) {
    usb_device_task();
    cdc_task();
  }
  return 0;
}

char __low_level_init(void) __attribute__ ((section(".init3"), naked));
char __low_level_init()
{
  (clock_prescale_set(0));
  return 1;
}

ISR(USB_GEN_vect)
{

  if (((USBINT & (1 << VBUSTI)) ? (1 == 1) : (0 == 1))
      && ((USBCON & (1 << VBUSTE)) ? (1 == 1) : (0 == 1))) {
    (USBINT = ~(1 << VBUSTI));
    if (((USBSTA & (1 << VBUS)) ? (1 == 1) : (0 == 1))) {
      usb_connected = (1 == 1);
      ;
      (g_usb_event |= (1 << 1));
      (UDIEN |= (1 << EORSTE));
      usb_start_device();
      (UDCON &= ~(1 << DETACH));
    } else {
      ;
      usb_connected = (0 == 1);
      usb_configuration_nb = 0;
      (g_usb_event |= (1 << 2));
    }
  }

  if (((UDINT & (1 << SOFI)) ? (1 == 1) : (0 == 1))
      && ((UDIEN & (1 << SOFE)) ? (1 == 1) : (0 == 1))) {
    (UDINT = ~(1 << SOFI));
    sof_action();;
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
    ;
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

      while (!(PLLCSR & (1 << PLOCK)));
    }
    (USBCON &= ~(1 << FRZCLK));
    (UDINT = ~(1 << WAKEUPI));
    if (usb_suspended) {
      (UDIEN |= (1 << EORSME));
      (UDIEN |= (1 << EORSTE));
      (UDINT = ~(1 << WAKEUPI));
      (UDIEN &= ~(1 << WAKEUPE));
      ;
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
    ;
    (g_usb_event |= (1 << 7));
  }

  if (((UDINT & (1 << EORSTI)) ? (1 == 1) : (0 == 1))
      && ((UDIEN & (1 << EORSTE)) ? (1 == 1) : (0 == 1))) {

    usb_remote_wup_feature = 0;

    (UDINT = ~(1 << EORSTI));
    usb_init_device();
    ;
    (g_usb_event |= (1 << 8));
  }

}
