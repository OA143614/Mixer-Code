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
#include <Bounce.h>

//this integer holds the current state of the menu
int menustate;

//these hold the analog values from the faders
int Channel1Fader;
int Channel2Fader;
int Channel3Fader;
int Channel4Fader;

int mainOutFader;
int auxOutFader;

//these hold the analog values from the aux mix pots
int Channel1AuxLevel;
int Channel2AuxLevel;
int Channel3AuxLevel;
int Channel4AuxLevel;

//this holds the analog value from the headphone volume control pot
int HeadphoneVolumeControl;

//these hold the digital values for all the mute buttons
int Channel1MuteButton;
int Channel2MuteButton;
int Channel3MuteButton;
int Channel4MuteButton;

int mainMixMuteButton;
int auxMixMuteButton;

//these are used to hold the acumulated values for the rotary encoders
int RotaryEncoder1Val;
int RotaryEncoder2Val;
int RotaryEncoder3Val;

void setup() 
{//initialization and setup of all initial values and initial state
  
  //set up encoders
  Encoder knob1(9, 10);
  Encoder knob2(25, 26);
  Encoder knob3(54, 55);
  
  //initialize values for encoders
  knob1.write(0);
  knob2.write(0);
  knob3.write(0);
  //should this happen again when we enter a new menu state?
  
   
  
  //set up bounce library for the 4 menu buttons
  Bounce backbutton = Bounce(33, 10); 
  Bounce nextbutton = Bounce(34, 10); 
  Bounce prevbutton = Bounce(35, 10); 
  Bounce entrbutton = Bounce(36, 10);
  
  //set up screens
  
  //poll initial values of all controls
  poll_controls();

  //initialize filters with zero values
  //look at the MATLAB code for the eq

  //setup the timed interrupt with FrequencyTimer2
  pinMode(FREQUENCYTIMER2_PIN, OUTPUT);
  FrequencyTimer2::setPeriod(22);
  FrequencyTimer2::enable();
  FrequencyTimer2::setOnOverflow(sample);
  
  menustate = 1;
}

//these hold the values read from the ADCs
volatile uint16_t current_sample1;
volatile uint16_t current_sample2;
volatile uint16_t current_sample3;
volatile uint16_t current_sample4;

void sample()
{//this function is attached to the timed interrupt
  digitalWrite(33, HIGH);
   SPI.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
  input1 = SPI.transfer16(0);
  SPI.endTransaction();
  digitalWrite(33, LOW);
  Serial.println(input1);
}

//the menu to be displayed on the larger screen is structured as follows
// 1 - the welcome screen
// 2,3,4,5 - the top level menu options (Parametric eq, graphic eq, compression, monitor channel select)
// 6,7,8 - the bands of the parametric eq (band 1, band 2, band 3)
// 9 - the graphic eq bands (three bands adjusted by the three knobs)
// 10-15 - the different channels in the compression menu (in1, in2, in3, in4, mainout, auxout, in order)
// 16-21 - the monitor channel selections (in1, in2, in3, in4, mainout, auxout, in order)

void back_button()
{
  if(menustate >= 1 && menustate <= 5)
  {//this is the top level menu - go to the 'welcome' screen
    menustate = 1;
  }
  else if(menustate >= 6 && menustate <= 8)
  {//the menu starts in the parametric eq - go to the parametric eq top level menu option
    menustate = 2;
  }
  else if(menustate == 9)
  {//the menu starts in the graphic eq - go to the graphic eq top level menu option
    menustate = 3;
  }
  else if(menustate >= 10 && menustate <= 15)
  {//the menu starts in the compression menu - go to the compression top level menu option
    menustate = 4;
  }
  else if(menustate >= 16 && menustate <= 21)
  {//the menu starts in the monitor channel select menu - go to the monitor channel select top level menu option
    menustate = 5;
  }
}

void next_button()
{
  switch(menustate)
  {
    case 5://wraparound for the top level menu
      menustate = 1;
      break;
    case 8://wraparound for the parametric eq submenu
      menustate = 6;
      break;
    case 9://only one option in this submenu - no update neccesary
      break;
    case 15://wraparound for the compression submenu
      menustate = 10;
      break;
    case 21://wraparound for the monitor channel select submenu
      menustate = 16;
      break;
    default://in general, except for the preceding 5 cases, this increments the menustate
      menustate++;
      break;
  }
}

void previous_button()
{
  switch(menustate)
  {
    case 1://wraparound for the top level menu
      menustate = 5;
      break;
    case 6://wraparound for the parametric eq submenu
      menustate = 8;
      break;
    case 9://only one option in this submenu - no update neccesary
      break;
    case 10://wraparound for the compression submenu
      menustate = 15
      break;
    case 16://wraparound for the monitor channel select submenu
      menustate = 21;
      break;
    default://in general, except for the preceding 5 cases, this decrements the menustate
      menustate--;
      break;
  }
}

void enter_button()
{
  switch(menustate)
  {
    case 2://you are selecting the parametric eq submenu
      menustate = 6;
      break;
    case 3://you are selecting the graphic eq submenu
      menustate = 9;
      break;
    case 4://you are selecting the compression submenu
      menustate = 10;
      break;
    case 5://you are selecting the monitor channel select submenu
      menustate = 16;
      break;
    default:
      break;
  }
}

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
  mainMixMuteButton   = digitalRead(27);
  auxMixMuteButton    = digitalRead(28);

  RotaryEncoder1Val   = knob1.read();
  RotaryEncoder2Val   = knob2.read();
  RotaryEncoder3Val   = knob3.read();

  if(backbutton.update())
    if(backbutton.risingEdge())
      back_button();
  
  if(nextbutton.update())
    if(nextbutton.risingEdge())
      next_button();
  
  if(prevbutton.update())
    if(prevbutton.risingEdge())
      previous_button();
  
  if(entrbutton.update())
    if(entrbutton.risingEdge())
      enter_button();
 
}                                           

void loop()                     
{//this will run in the time when the interrupt is not running

  //screens update

  //check the values of all controls
    poll_controls();
}
