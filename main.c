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
      usb_process_request();
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
