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
#include  <avr/power.h>
void set_idle_mode(void);
void set_power_down_mode(void);
void set_adc_noise_reduction_mode(void);
void set_power_save_mode(void);
void set_standby_mode(void);
void set_ext_standby_mode(void);
void Clock_switch_external(void);
void Clock_switch_internal(void);
void set_power_down_mode(void)
{
   (SMCR=0,SMCR |= (1<<SE)+(1<<SM1)) ;
   {asm("SLEEP");} ;
}
void set_idle_mode(void)
{
   (SMCR=0,SMCR |= (1<<SE)) ;
   {asm("SLEEP");} ;
}









void set_adc_noise_reduction_mode(void)
{
   (SMCR=0,SMCR |= (1<<SE)+(1<<SM0)) ;
   {asm("SLEEP");} ;
}









void set_power_save_mode(void)
{
   (SMCR=0,SMCR |= (1<<SE)+(1<<SM1)+(1<<SM0)) ;
   {asm("SLEEP");} ;
}









void set_standby_mode(void)
{
   (SMCR=0,SMCR |= (1<<SE)+(1<<SM2)+(1<<SM1)) ;
   {asm("SLEEP");} ;
}









void set_ext_standby_mode(void)
{
   (SMCR=0,SMCR |= (1<<SE)+(1<<SM2)+(1<<SM1)+(1<<SM0)) ;
   {asm("SLEEP");} ;
}
void Clock_switch_external(void)
{
  (CLKSEL0 |= (1<<EXTE)) ;
  while (! (((CLKSTA&(1<<EXTON)) != 0) ? (1==1) : (0==1) ) );
  (CLKSEL0 |= (1<<CLKS)) ;
  (CLKSEL0 &= ~(1<<RCE)) ;
}










void Clock_switch_internal(void)
{
  (CLKSEL0 |= (1<<RCE)) ;
  while (! (((CLKSTA&(1<<RCON)) != 0) ? (1==1) : (0==1) ) );
  (CLKSEL0 &= ~(1<<CLKS)) ;
  (CLKSEL0 &= ~(1<<EXTE)) ;
}
