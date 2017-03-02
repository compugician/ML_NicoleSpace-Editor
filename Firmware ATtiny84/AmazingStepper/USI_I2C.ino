// Ultra lean I2C setup to show how the USI works on the Tiny 84 as I2C slave
//
// Craig Limber, December 2014
//
// All this does is store a byte sent by the master and send it back with the
// read function.  If nothing written before reading it just returns zero.
// 
// When learning something new I like to start with the simplest example
// possible.  So, I made this as lean as I could reasonably get away with:
//   each write to a register is done explicitely without macros or #defines
//   no external modules or header files
//   no functions, just the main and two interrupt handlers
//   only two variables 
//
// When you see "PDF XXX" that means the docs for that thing are in atmel 
// PDF for the tiny 24/44/84 on page XXX.  Document 8006K-avr-10/10
// 
// This example has all of the bits I ueed to work with my logic analyzer
// removed to try and keep it lean and simple for you.
//
//                +-------------------------------+
//                |1          VCC  GND          14|
//                |2          pb0  pa0 adc0     13| 
//                |3          pb1  pa1 adc1     12|              
//                |4        RESET  pa2 adc2     11|
//                |5 0c0a     pb2  pa3 adc3     10|
//                |6 0c0b  c7 pa7  pa4 adc4 SCK  9| SCL    
//            SDA |7 C1A MOSI pa6  pa5 MISO C1B  8|            
//                +-------------------------------+
//
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit)) // for avr-libc 1.4 or later
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#include <inttypes.h>
#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/power.h>
#define F_CPU 8000000L 
#include <util/delay.h>

// states to handle in the overflow interrupt 
enum
{
  CHECK_ADDRESS_STATE,   // first state after the start condition 
  DATA_READ_STATE,       // set up USI to receive a byte after the address ACK
  DATA_SEND_STATE,       // set up USI to send a byte after the address ACK
  DATA_READ_ACK_STATE,   // set up USI to send an ack after receiving a byte
  DATA_SEND_ACK_STATE,   // set up USI to ignore ack from master for sent byte
  DONE_STATE             // shut down overflow interrupt until next start cond.
};

// how the states flow each time the overflow interrupt is called 
//start -> CHECK_ADDRESS_STATE -> DATA_READ_STATE -> DATA_READ_ACK_STATE -> done
//start -> CHECK_ADDRESS_STATE -> DATA_SEND_STATE -> DATA_SEND_ACK_STATE -> done

uint8_t deviceAddress = 0x40; // the address of this device 

uint8_t curstate = CHECK_ADDRESS_STATE; // current state. Only used in overflow 
                                        // interrupt.  Tells us what to do next

//-----------------------------------------------------------------------------
ISR(USI_STR_vect) 
// 
// This here interrupt is called when a start or stop condition occurs. 
// This happens when the SDA changes state while SCL is low.  PDF 123.
// If SDA is low in interrupt it was a start condition so set state machine to 
// CHECK_ADDRESS_STATE, clear USI counter and enable overflow interrupt.
// The overflow interrupt is not enabled before this to prevent overflows 
// from happening when we don't want it to.  
//
// If SDA is high a stop condition occured.  Disable overflow interrupt
// and wait for another start condition 
//
{
  curstate = CHECK_ADDRESS_STATE;  
  cbi(DDRA, DDA6);   // set SDA pin for input 
  // wait for start to end and checking if SDA is clear falls through with stop
  while(bit_is_set(PINA, PINA4) && bit_is_clear(PINA, PINA6))  // SCL 1, SDA 0
    asm volatile ("NOP" ::); 
  // check if SDA line low indicating a start.  Can also check USIPF 
  if(bit_is_clear(PINA, PINA6))  // is SDA line low? if yes, it's a start cond. 
  {
    // received start condition 
    USICR = (1 << USISIE)  // enable start condition interrupt
          | (1 << USIOIE)  // enable overflow interrupt 
          | (1 << USIWM1)  // two wire serial mode 
          | (1 << USIWM0)  // two wire serial mode, stretch SCL
          | (1 << USICS1)  // next three bits define how clock works 
          | (0 << USICS0)  //   counter triggered both edge
          | (0 << USICLK)  //   USIDR shifts on positive edge only 
          | (0 << USITC);  // don't need to toggle clock pin 
  }
  else
  {
    // received stop condition
    USICR = (1 << USISIE)  // enable start condition interrupt 
          | (0 << USIOIE)  // disable overflow interrupt for now 
          | (1 << USIWM1)  // two wire serial mode 
          | (1 << USIWM0)  // two wire serial mode, stretch SCL
          | (1 << USICS1)  // next three bits define how clock works:
          | (0 << USICS0)  //   counter triggered both edge
          | (0 << USICLK)  //   USIDR shifts on positive edge only 
          | (0 << USITC);  // don't need to toggle clock pin 
  }

  USISR = (1 << USISIF)     // clear all of the interrupt flags 
        | (1 << USIOIF)     // clear all of the interrupt flags 
        | (1 << USIPF)      // clear all of the interrupt flags 
        | (1 << USIDC)      // clear all of the interrupt flags 
        | (0 << USICNT3)    // clear USI counter 
        | (0 << USICNT2)    
        | (0 << USICNT1)    
        | (0 << USICNT0);   
}

//-----------------------------------------------------------------------------
ISR(USI_OVF_vect)
// 
// This here interrupt is called when the USI counter rolls over to 0. 
// Depending on the state stored in the variable curstate the interrupt will
// decide what to do with what's in the USIDR.  
// 
// This version is set up to stretch the SCL clock only when receiving a 
// byte of data.  The SCL is released when the overflow interrupt flag is 
// cleared.  Don't forget, to "clear" an interrupt flag on this device you
// write a one to the corresponding bit!
//
// With a "read" operation we are sending data to the master.  Don't let
// that mess you up!  A read operation from the master corresponds to a 
// data send from here and vice-versa.  The states are what the USI is 
// DOING not what the master is doing.  
// 
{
  switch (curstate)
  {
    case CHECK_ADDRESS_STATE:  // got 8 bits just after start cond. For us?
      if((USIDR == 0) || ((USIDR >> 1) == deviceAddress)) 
      {        
        if (USIDR & 0x01)   // are we reading or writing?  a 1 means "read"
          curstate = DATA_SEND_STATE;   // state to move to once we send ack
        else
          curstate = DATA_READ_STATE;   
        USIDR = 0;                // set SDA to low to send ACK   PDF 127
        sbi(DDRA, DDA6);          // let the bus see it.  SCL still held low
        USISR = (1 << USISIF)     // clear all of the interrupt flags 
              | (1 << USIOIF)     // clear all of the interrupt flags 
              | (1 << USIPF)      // clear all of the interrupt flags 
              | (1 << USIDC)      // clear all of the interrupt flags 
              | (1 << USICNT3)    // set counter to overflow after one SCL clock
              | (1 << USICNT2)    // since one SCL clock counts as two we'll
              | (1 << USICNT1)    // use 0x0E instead of 0x0F
              | (0 << USICNT0);   
      }
      else  // address not for us, reset and wait for another start condition
      {
        USICR = (1 << USISIE)   // start condition interrupt enable
              | (0 << USIOIE)   // counter overflow interrupt disable
              | (1 << USIWM1)   // wire mode I2C
              | (1 << USIWM1)   //   hold SCL on counter overflow 
              | (1 << USICS1)   // clock source
              | (0 << USICS0)   // clock source
              | (0 << USICLK)   // clock strobe
              | (0 << USITC);   // clock port toggle pin
        USISR = (1 << USISIF)   // clear all of the interrupt flags 
              | (1 << USIOIF)   // clear all of the interrupt flags 
              | (1 << USIPF)    // clear all of the interrupt flags 
              | (1 << USIDC)    // clear all of the interrupt flags 
              | (0 << USICNT3)  // clear the counter
              | (0 << USICNT2)  
              | (0 << USICNT1) 
              | (0 << USICNT0);   
      }
      break;

    case DATA_READ_STATE:      // after ack from check address, now rx 8 bits
      cbi(DDRA, DDA6);         // set SDA for input 
      USISR = (1 << USISIF)    // clear all of the interrupt flags 
            | (1 << USIOIF)    // clear all of the interrupt flags 
            | (1 << USIPF)     // clear all of the interrupt flags 
            | (1 << USIDC)     // clear all of the interrupt flags 
            | (0 << USICNT3)   // clear the counter; going to receive 8 bits
            | (0 << USICNT2)  
            | (0 << USICNT1) 
            | (0 << USICNT0);   
      curstate = DATA_READ_ACK_STATE;   // next overflow will be our ACK
      break;

    case DATA_READ_ACK_STATE:  // just got 8 bits from master, must ACK
      userdata = USIDR;        // store the data the master sent 
      curstate = DONE_STATE;   // next state turns stuff off 
      USIDR = 0;               // set SDA to low to send ACK
      sbi(DDRA, DDA6);         // let the outside world see it 
      USISR = (1 << USISIF)    // clear all of the interrupt flags 
            | (1 << USIOIF)    // clear all of the interrupt flags 
            | (1 << USIPF)     // clear all of the interrupt flags 
            | (1 << USIDC)     // clear all of the interrupt flags 
            | (1 << USICNT3)   // set counter to overflow after one SCL clock
            | (1 << USICNT2)   // since one SCL clock counts as two in the USI
            | (1 << USICNT1)   // we'll use 0x0E instead of 0x0F
            | (0 << USICNT0);   
      break;

    case DATA_SEND_STATE:      // set up USI to echo data sent to us
      USIDR = userdata;
      curstate = DATA_SEND_ACK_STATE;
      sbi(DDRA, DDA6);         // set up SDA as output 
      USISR = (0 << USISIF)    // clear all of the interrupt flags 
            | (1 << USIOIF)    // clear all of the interrupt flags 
            | (1 << USIPF)     // clear all of the interrupt flags 
            | (1 << USIDC)     // clear all of the interrupt flags 
            | (0 << USICNT3)   // clear the counter
            | (0 << USICNT2)  
            | (0 << USICNT1) 
            | (0 << USICNT0);   
      break;

    case DATA_SEND_ACK_STATE:  // master would ack but we don't care so phooey
      curstate = DONE_STATE;   // where to go next after ACK 
      USISR = (1 << USISIF)    // clear all of the interrupt flags 
            | (1 << USIOIF)    // clear all of the interrupt flags 
            | (1 << USIPF)     // clear all of the interrupt flags 
            | (1 << USIDC)     // clear all of the interrupt flags 
            | (1 << USICNT3)   // set the counter to 0x0E.  ACK is a single
            | (1 << USICNT2)   // SCL clock pulse or two edges counted by 
            | (1 << USICNT1)   // the USI 
            | (0 << USICNT0);   
      break;

    case DONE_STATE:           // all done, turn off overflow interrupt
        cbi(DDRA, DDA6);       // make sure SDA is not outputting anything
        USICR = (1 << USISIE)  // start condition interrupt enable
              | (0 << USIOIE)  // counter overflow interrupt disable
              | (1 << USIWM1)  // wire mode I2C, don't hold SCL line low
              | (0 << USIWM1)  // wire mode
              | (1 << USICS1)  // clock source
              | (0 << USICS0)  // clock source
              | (0 << USICLK)  // clock strobe
              | (0 << USITC);  // clock port toggle pin
        break;
  }
}

//-----------------------------------------------------------------------------
void setupUSI_I2C(uint8_t addr) 
{
  deviceAddress = addr;
  
  USICR = (1 << USISIE)    // enable start condition interrupt enable
        | (0 << USIOIE)    // disable overflow interrupt for now 
        | (1 << USIWM1)    // set to two wire serial mode 
        | (1 << USIWM0)    // set to two wire serial mode hold SCL down 
        | (1 << USICS1)    // next three bits define how clock works 
        | (0 << USICS0)    // counter triggered both edge
        | (0 << USICLK)    // USIDR shifts on positive edge only 
        | (0 << USITC);
  // clear all of the USI flags by writing 1 to them and clear counter
  USISR = (1 << USISIF)    // start condition interrupt flag 
        | (1 << USIOIF)    // counter overflow interrupt flag 
        | (1 << USIPF)     // stop condition flag 
        | (1 << USIDC)     // data output collision 
        | (0 << USICNT3)   // clear the USI counter 
        | (0 << USICNT2)      
        | (0 << USICNT1)      
        | (0 << USICNT0);   

  cbi(DDRA, DDA4); sbi(PORTA, PA4);  // SCL, start as input and hi-z   PDF 127
  cbi(DDRA, DDA6); sbi(PORTA, PA6);  // SDA. start as input and hi-z   PDF 127

//  CLKPR = _BV(CLKPCE);    // set system clock to 1Mhz; bang on door first
//  CLKPR =  _BV(CLKPS0)    // turn pre-scale to 8 for 1 mhz clock
//           | _BV(CLKPS1);

  sei();                 // enable interrupts globally
}
