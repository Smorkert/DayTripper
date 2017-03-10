#define SAMPLE_RATE 44 // 44.1kHz sample rate
#define ADCS 1 // only 1 ADC used here -> MOD0
#define HYST 16
#define LHPVOL 0
#define RHPVOL 0
#define LINVOL 20
#define RINVOL 20

#include <Wire.h>
#include <SPI.h>
#include "AudioCodecMod.h"


/* Encoder Commands: C0, C1, C2
   --------------------------
   read_enable:           PORTC &= ~7; (000)
   write_enable:          PORTC = (1|PORTC)&~6; (001)
   count_up:              PORTC = (2|PORTC)&~5; (010)
   count_down:            PORTC = (3|PORTC)&~4; (011)
   reset:                 PORTC = (4|PORTC)&~3; (100)
   LE_Latch:              PORTC = (5|PORTC)&~2; (101)
   unused:                PORTC = (6|PORTC)&~1; (110)
   clear:                 PORTC |= 7; (111)

   Toggle Lower address Byte: PORTB ^= 2;
   Max address: 262144 (4194304bits/16)
*/

//Audio Codec Variables
volatile uint16_t mod_value = 0;
int16_t left_in = 0; //in from codec (LINE_IN)
int16_t right_in = 0;
int16_t left_out = 0; //out to codec (HP_OUT)
int16_t right_out = 0;
int16_t Dsample = 0;

//SRAM read/write variables
uint8_t LSB, MSB, state = 0; 
volatile uint8_t effect = 0;
uint32_t DelayR = 0, DelayD = 0;
uint32_t address = 0; 

//Chorus Variables
int16_t delBuff[512];
uint16_t LFO, pointer, delay_pointer, unPhase, temp; //n must be <= max amplitude of LFO
// create sinewave lookup table (512, 0-511)
// PROGMEM stores the values in the program memory
PROGMEM  const uint16_t wavetable[]  = {
  #include "sinewave_chorus.h"
};

// lookup table value location
uint32_t phase; // this is a 32bit number


//Function to clear SRAM at startup to avoid outputing noise
void clearSRAM(){
  while(address < 262144){
    writeSRAM(MSB, LSB, 0);
    //increment address
    PORTC = (2|PORTC)&~5; //set low
    PORTC |= 7; //set high
    address++;
    __asm__("nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t");
  }
  address = 0;
}

void setup(){
  //set C0-C2 to outputs
  DDRC |= 7;
  //set ABT to output
  DDRB |= (1<<1);
  //write low initially
  PORTB &= ~(1<<1);
  //reset timer
  PORTC = (4|PORTC)&~3;
  __asm__("nop\n\t" "nop\n\t");
  PORTC |= 7; //(111)
  clearSRAM();
  //call this last if you are setting up other things
  AudioCodec_init(); //setup codec and microcontroller registers
}

void loop() {
  while(1);
}

//left in not working?
ISR(TIMER1_COMPA_vect, ISR_NAKED){

  // &'s are necessary on data_in variables
  AudioCodec_data(&left_in, &right_in, left_out, right_out);

  switch(effect){
      //Reverse
      case 0:
      //get sample from memory
      right_out = readSRAM(MSB,LSB);
      left_out = left_in;
      //write input to memory
      writeSRAM(MSB, LSB, right_in);
      //Read Delay value
      DelayR = (((uint32_t)mod_value+1)<<2)-1; //scales to 256-262143 (5.8mS - 5.94s)
      
      //Address counting up or down depending on state
      switch(state){
        case 0:
          if(address >= DelayR){
            state = 1;
          }
          else{
            //increment address
            PORTC = (2|PORTC)&~5; //set low
            PORTC |= 7; //set high
            address++;
          }
        break;
        case 1:
          if(address <= 0){
            state = 0;
          }
          else{
            //decrement address
            PORTC = (3|PORTC)&~4; //set low
            PORTC |= 7; //set high
            address--;
          }
        break;
      }
      break;
  
      /*
      //Reverse and Delay are on (1)
      case 1:
      right_out = readSRAM(MSB,LSB);
      //For reverse delay----------------
      left_out = left_in; //analog mixing
      //Write to memory
      Dsample = right_in-(right_in>>3);
      writeSRAM(MSB, LSB, Dsample); //analog mixing
      //Read Delay value 
      DelayD = (((uint32_t)mod_value+1)<<2)-1; //scales to 256-65535 (5.8mS - 1.5ms)
      //Address counting up or down depending on state
      switch(state){
        case 0:
          if(address >= DelayD){
            state = 1;
          }
          else{
            //increment address
            PORTC = (2|PORTC)&~5; //set low
            PORTC |= 7; //set high
            address++;
          }
        break;
        case 1:
          if(address <= 0){
            state = 0;
          }
          else{
            //decrement address
            PORTC = (3|PORTC)&~4; //set low
            PORTC |= 7; //set high
            address--;
          }
        break;
      }
      break;
      */
  //Chorus
  case 1: 
  
    //increment phase
    phase += 1 + (mod_value >> 8);
    //check overflow
    phase &= 0x0003ffff;
    //get location
    unPhase = (phase >> 9);
    //LFO value
    LFO = pgm_read_word_near(wavetable + unPhase);
    //get location of delayed sample
    temp = (uint16_t)(pointer-LFO);
    //"modulus function" with %512, keeps delay in range 0-511
    delay_pointer = temp-((temp>>9)<<9);

    left_out = left_in;
    right_out = delBuff[delay_pointer];
    //increment buffer
    pointer++;
    //check overflow
    pointer &= 0x01FF;
    //store result into buffer
    delBuff[pointer] = right_in;

  break;
  }

  //Set PORTD:
  //PINS: 0 to input, read switch
  DDRD &= ~0x01; //Set PORTD to input excluding PIN5
  
  //open gate on transistor to read switch 
  PORTC = (5|PORTC)&~2;
  __asm__("nop\n\t" "nop\n\t");

  //read status of switch to determine effect
  effect = PIND&0x01;
  __asm__("nop\n\t");
  effect = PIND&0x01;
  __asm__("nop\n\t");
  
  //Close latch to set port D back to data bus
  PORTC |= 7;
  AudioCodec_ADC(&mod_value);
  
  reti(); // dont forget to return from the interrupt
}
