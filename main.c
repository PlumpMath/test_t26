#include <ioavr.h>
#include <intrinsics.h>

/*
ATTiny26 1.0MHz

FL: 0xE1
FH: 0x1E
LB: 0x03
*/

#define PRINTF_STDIO 0

#if PRINTF_STDIO
#include <stdio.h>
#else
#include "printf.h"
#define printf tfp_printf
#endif

#include "pt.h"

static int protothread1_flag, protothread2_flag;

struct timer { int start, interval; };
static int  timer_expired(struct timer *t);
static void timer_set(struct timer *t, int usecs);

//TIMER0 initialize - prescale:1024
// desired value: 100mSec
// actual value: 99.328mSec (0.7%)
void timer0_init(void)
{
  TCCR0 = 0x00; //stop
  TCNT0 = 0x9F; //set count
  TCCR0 = 0x05; //start timer
}

#pragma vector=TIMER0_OVF0_vect
__interrupt void timer0_ovf(void)
{
   static unsigned char tick = 0;
   
   TCNT0 = 0x9F; //reload counter value
 
   tick++;
   if ( tick >= 10 ) {
      tick = 0;
      
      
   }
}

//PWM mode 
void timer1_init()
{
  TCCR1B = 0;
  // init counter
  TCNT1 = 0;
  OCR1A = 0;
  OCR1B = 0;
  OCR1C = 255; //TOP
  TCCR1A = (1<<COM1B1) | (0<<COM1B0) // OC1B cleared on compare match, Set Whe TCNT1 = 1, ~OC1B not connected 
    | (0<<PWM1A) | (1<<PWM1B); //PWM mode, TOP=OC1C, OC1B
  TCCR1B = (1<<CS12) | (1<<CS10); // prescaler = 1024  
}

#pragma vector=TIMER1_OVF1_vect
__interrupt void timer1_ovf(void)
{  

}

//TCNT1 = 1 __/--
//TCNT1 = OCR1B  --\__
//TCNT1 = OCR1C (TOP)
void timer1_pwm(unsigned char duty, unsigned char top)
{
  unsigned long tmp;
  
  tmp = duty;
  tmp *= top;
  tmp /= 100;
  
  OCR1B = (unsigned char)tmp;
  OCR1C = top;
}

/*
//X-tal 12.0MHz 
//http://wormfood.net/avrbaudcalc.php
void uart0_init(void)
{
   UCSRB = 0x00;
   UCSRA = (1<<U2X);
   UCSRC = (1<<URSEL) | 0x06;
   UBRRL = 0x0C;
   UBRRH = 0x00;
   UCSRB = 0x18;
}

// function to send data
void uart_transmit (unsigned char data)
{
    while (!( UCSRA & (1<<UDRE)));                // wait while register is free
    UDR = data;                                   // load data in the register
}

// function to receive data
unsigned char uart_recieve (void)
{
    while(!(UCSRA) & (1<<RXC));                   // wait while data is being received
    return UDR;                                   // return 8-bit data
}

void putchar(char c)
{
    if ( c == '\n' )
        putchar('\r');
    
    uart_transmit(c);
}
*/

static int clock_time(void)
{
  int t;
  
  t = 1;
  return t;  
}

static int timer_expired(struct timer *t)

{ 
  return (int)(clock_time() - t->start) >= (int)t->interval; 
}



static void timer_set(struct timer *t, int interval)

{
  t->interval = interval; t->start = clock_time(); 
}

static int protothread1(struct pt *pt)
{
  static int count = 0;
  
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, protothread2_flag != 0);
    //uart_transmit('A');
    printf("%05u %d\n", count++, pt);
    protothread2_flag = 0;
    protothread1_flag = 1;

    PT_YIELD(pt);    
  }    
  PT_END(pt);

}

static int
protothread2(struct pt *pt)
{
  PT_BEGIN(pt);
  while(1) {
    protothread2_flag = 1;
    PT_WAIT_UNTIL(pt, protothread1_flag != 0);
    
    //uart_transmit('B');
    printf("B %d\n", pt);
    protothread1_flag = 0;
    
    PT_YIELD(pt);
  }
  
  PT_END(pt);

}

//buadrate 38400 @ 1.000MHz
#define SOFT_TX_PORT  PORTA
#define SOFT_TX_BIT   0
#define DELAY_1       18
#define DELAY_2       13
#define DELAY_3       4
#define STOP_BITS     1

#pragma optimize=none
void software_putchar(char c)
{
  unsigned char  bit_mask;
  
  // start bit
  SOFT_TX_PORT &= ~(1<<SOFT_TX_BIT);
  __delay_cycles(DELAY_1);

  // data bits
  for ( bit_mask = 0x01; bit_mask; bit_mask <<= 1 ) {
    if ( c & bit_mask ) {
      SOFT_TX_PORT |= (1<<SOFT_TX_BIT);
    }
    else {
      SOFT_TX_PORT &= ~(1<<SOFT_TX_BIT);
    }
    __delay_cycles(DELAY_2);
  }
  __delay_cycles(DELAY_3);
  // stop bit(s)
  SOFT_TX_PORT |= (1<<SOFT_TX_BIT);
  __delay_cycles(DELAY_1 * STOP_BITS);
}

#if PRINTF_STDIO
int putchar(int c)
#else
void putchar(char c)
#endif
{
  __istate_t state;
    
  if ( c == '\n' )
    putchar('\r');
  
  state = __get_interrupt_state();
  __disable_interrupt();
  
  software_putchar(c);
  
  __set_interrupt_state(state);

#if PRINTF_STDIO
  return c;
#endif
}


static struct pt pt1, pt2;

int main(void)
{
  //uart0_init();
  
  //LED
  PORTA |=  (1 << 0);
  DDRA  |=  (1 << 0);
  
  //MOTOR PA2=EN, PA6=A, PA7=B
  PORTA &= ~(1 << 2);
  DDRA  |=  (1 << 2);   
  PORTA |=  (3 << 6);
  DDRA  |=  (3 << 6);
  
  //BUTTON
  PORTB |=  (1 << 6);
  DDRB  &= ~(1 << 6);    
  //PORTB |=  (1 << 3);
  //DDRB  &= ~(1 << 3);
  
  //OC1B PB3
  PORTB |= (1 << 3);
  DDRB  |= (1 << 3);
   
  timer0_init();
  timer1_init();
  
  MCUCR = 0x00;
  TIMSK = (1<<TOIE1) | (1<<TOIE0);
  GIMSK = 0x00;
  PLLCSR = 0x00;
 
  PT_INIT(&pt1);
  PT_INIT(&pt2);
   
  __enable_interrupt();
  
  timer1_pwm(1, 100);
  
  while (1) {
    static unsigned char duty = 0;
    
    protothread1(&pt1);
    protothread2(&pt2);
    
    //PORTA ^= (1 << 0);
    //software_putchar('A');
    //software_putchar('T');
    
    timer1_pwm(duty++, 200);
    
    if ( duty >= 100 )
      duty = 0;
    
    //__delay_cycles(100000);
  }
}
