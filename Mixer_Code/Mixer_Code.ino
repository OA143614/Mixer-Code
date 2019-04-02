/* ALLEN AND KEITH AUDIO MIXER CODE
 *  
 *  
 *  Analog inputs
 *    A0-A3 are Channel 1 to Channel 4 faders
 *    A4-A5 are the Main and Aux mix output faders
 *    A6-A9 are Channel 1 to Channel 4 aux levels
 *    [A10 or pin 49] is headphone volume control - how do you use pin A10?
 *    
 *  Digital inputs
 *    29,30,31,24 are the Channel 1 to Channel 4 Mutes (latching buttons)
 *    27 and 28 are the Main and Aux Mix Mute buttons  (latching buttons)
 *    9 and 10, 25 and 26, 54 and 55 are pairs for each of the rotary encoders
 *    33-36 are the four menu  buttons (momentary buttons)
*/

#include <FrequencyTimer2.h>
#include <Arduino.h>
#include <SPI.h>
#include <Encoder.h>

void setup() 
{//initialization and setup of all initial values and initial state
  
  //set up devices

  //poll initial values of all controls
  poll_controls();

  //initialize filters with zero values
  //look at the MATLAB code for the eq

  //setup the timed interrupt with FrequencyTimer2
  pinMode(FREQUENCYTIMER2_PIN, OUTPUT);
  FrequencyTimer2::setPeriod(22);
  FrequencyTimer2::enable();
  FrequencyTimer2::setOnOverflow(sample);

  Encoder knob1(9, 10);
  Encoder knob2(25, 26);
  Encoder knob3(54, 55);
}

volatile uint16_t current_sample1;
volatile uint16_t current_sample2;
volatile uint16_t current_sample3;
volatile uint16_t current_sample4 ;

void sample()
{//this function is attached to the timed interrupt
  digitalWrite(33, HIGH);
   SPI.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
  input1 = SPI.transfer16(0);
  SPI.endTransaction();
  digitalWrite(33, LOW);
  Serial.println(input1);
}

void back_button_interrupt()
{
  
}

void next_button_interrupt()
{
  
}

void previous_button_interrupt()
{
  
}

void enter_button_interrupt()
{
  
}

int Channel1Fader;
int Channel2Fader;
int Channel3Fader;
int Channel4Fader;
int mainOutFader;
int auxOutFader;
int Channel1AuxLevel;
int Channel2AuxLevel;
int Channel3AuxLevel;
int Channel4AuxLevel;
int HeadphoneVolumeControl;
int Channel1MuteButton;
int Channel2MuteButton;
int Channel3MuteButton;
int Channel4MuteButton;
int mainMixMuteButton;
int auxMixMuteButton;
int RotaryEncoder1Val;
int RotaryEncoder2Val;
int RotaryEncoder3Val;
int MenuButton1Book;
int MenuButton1Next;
int MenuButton1Previous;
int MenuButton1Enter;

void poll_controls()
{//check the values from all the UI controls and update their values
  
  Channel1Fader = analogRead(14);
  Channel2Fader = analogRead(15);
  Channel3Fader = analogRead(16);
  Channel4Fader = analogRead(17);
  mainOutFader    = analogRead(18);
  auxOutFader    = analogRead(19);
  Channel1AuxLevel= analogRead(20);
  Channel2AuxLevel= analogRead(21);
  Channel3AuxLevel= analogRead(22);
  Channel4AuxLevel= analogRead(23);
  HeadphoneVolumeControl= analogRead(49);
  Channel1MuteButton  = digitalRead(29);
  Channel2MuteButton  = digitalRead(30);
  Channel3MuteButton  = digitalRead(31);
  Channel4MuteButton  = digitalRead(24);
  mainMixMuteButton = digitalRead(27);
  auxMixMuteButton    = digitalRead(28);

  
  RotaryEncoder1Val   = knob1.read();
  RotaryEncoder2Val   = knob2.read();
  RotaryEncoder3Val   = knob3.read();

  
  MenuButton1Back      = digitalRead(33); //these will need to be handled by interrupts
  MenuButton1Next      = digitalRead(34);     //keep current menu state as a number -
  MenuButton1Previous  = digitalRead(35);     //display function is a switch statement based on this number
  MenuButton1Enter     = digitalRead(36);     //in the interrupts, make sure we know the transitions associated
}                                             //with the 

void loop()                     
{//this will run in the time when the interrupt is not running

  //screens update

  //check the values of all controls
    poll_controls();
}
