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
  usb_device_task_init();
  usb_remote_wup_feature = 0;
  cdc_task_init();
  while (1) {
    usb_task();
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
