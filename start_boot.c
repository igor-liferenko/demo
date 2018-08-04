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
extern U32 boot_key __attribute__ ((section(".noinit")));
void start_boot_if_required(void);
void start_boot(void);
#include  <avr/io.h>
#include  <avr/wdt.h>

U32 boot_key __attribute__ ((section(".noinit")));

void start_boot(void)
{
  boot_key = 0x55AAAA55;

  wdt_reset();
  (WDTCSR |= (1 << WDCE));
  (WDTCSR = (1 << WDE));

  while (1);
}
