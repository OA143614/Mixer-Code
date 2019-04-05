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
#include <math.h>

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

//filter coefficients
//how do these look, how many are there, where are they kept


#define NUM_SAMPLES 20 //is this enough

//these will be circular buffers used to draw out the waveform
uint16_t channel1_samples[NUM_SAMPLES];
int channel1_samples_current_index;

uint16_t channel2_samples[NUM_SAMPLES];
int channel2_samples_current_index;

uint16_t channel3_samples[NUM_SAMPLES];
int channel3_samples_current_index;

uint16_t channel4_samples[NUM_SAMPLES];
int channel4_samples_current_index;


void setup() 
{//initialization and setup of all initial values and initial state
 
  channel1_samples_current_index = 0;
  channel2_samples_current_index = 0;
  channel3_samples_current_index = 0;
  channel4_samples_current_index = 0;
  
  //initializing the circular buffers associated with each of the input channels
  //these values will be used to draw out the waveform
  for(int i = 0; i < NUM_SAMPLES; i++)
  {
    channel1_samples[i] = 0;
    channel2_samples[i] = 0;
    channel3_samples[i] = 0;
    channel4_samples[i] = 0;
  }
 
  //set up encoders
  Encoder knob1(9, 10);
  Encoder knob2(25, 26);
  Encoder knob3(54, 55);
  
  //initialize values for encoders
  knob1.write(0);
  knob2.write(0);
  knob3.write(0);
   
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
  
  //apply parametric eq for input 1
  //apply parametric eq for input 2
  //apply parametric eq for input 3
  //apply parametric eq for input 4
  
  //apply compression for input 1
  //apply compression for input 2
  //apply compression for input 3
  //apply compression for input 4
  
  //do the mixing operation for main mix
  //do the mixing operation for aux mix
  
  //apply graphic eq to main mix
  //apply graphic eq to aux mix
  
  //apply compression to main mix
  //apply compression to aux mix
  
}

//the menu to be displayed on the larger screen is structured as follows
// 1 - the welcome screen
// 2,3,4,5 - the top level menu options (Parametric eq, graphic eq, compression, monitor channel select) - enter to toggle main/aux
// 6,7,8 - the bands of the parametric eq (band 1, band 2, band 3) - enter to toggle main/aux
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

void peakfiltercalc(int number, int dBgain, int freq, int Q)
{
  int srate = 44100;
  
  double A = pow(10, dBgain / 40);
  double O = 2 * 3.14159 * (freq / srate);
  
  double sn = sin(O);
  double cs = cos(O);
  
  double alpha = sn / (2 * Q);
  double beta = sqrt(2 * A);
  
  double b0 = 1 + (alpha * A);
  double b1 = -2 * cs;
  double b2 = 1 - (alpha * A);
  double a0 = 1 + (alpha / A);
  double a1 = -2 * cs;
  double a2 = 1 - (alpha / A);
  
  
  //return these, or pass them in by reference
  double c0 = b0 / a0;
  double c1 = b1 / a0;
  double c2 = b2 / a0;
  double c3 = a1 / a0;
  double c4 = a2 / a0;

}
  
void notchfiltercalc(int number, int dBgain, int freq, int Q)
{
  int srate = 44100;
  
  double A = pow(10, dBgain / 40);
  double O = 2 * 3.14159 * (freq / srate);
  
  double sn = sin(O);
  double cs = cos(O);
  
  double alpha = sn / (2 * Q);
  double beta = sqrt(2 * A);
    
  double b0 = 1;
  double b1 = -2 * cs;
  double b2 = 1;
  double a0 = 1 + alpha;
  double a1 = -2 * cs;
  double a2 = 1 - alpha;
  
  
  //return these, or have values in by reference
  double c0 = b0 / a0;
  double c1 = b1 / a0;
  double c2 = b2 / a0;
  double c3 = a1 / a0;
  double c4 = a2 / a0;
}

void poll_controls()
{//check the values from all the UI controls and update their values
  
  Channel1Fader = analogRead(14);
  Channel2Fader = analogRead(15);
  Channel3Fader = analogRead(16);
  Channel4Fader = analogRead(17);
  mainOutFader = analogRead(18);
  auxOutFader = analogRead(19);
  Channel1AuxLevel = analogRead(20);
  Channel2AuxLevel = analogRead(21);
  Channel3AuxLevel = analogRead(22);
  Channel4AuxLevel = analogRead(23);
  HeadphoneVolumeControl = analogRead(49);
  Channel1MuteButton  = digitalRead(29);
  Channel2MuteButton  = digitalRead(30);
  Channel3MuteButton  = digitalRead(31);
  Channel4MuteButton  = digitalRead(24);
  mainMixMuteButton   = digitalRead(27);
  auxMixMuteButton    = digitalRead(28);
  

  //if(abs(val=knob.read())) - if the read returns a nonzero value, this if statement will evaluate to true.
    //positive integers make if evaluate true - the value is significant if it is nonzero, positive or negative, thus, abs()
  if(abs(RotaryEncoder1Val = knob1.read()))
     encoder1_update(RotaryEncoder1Val);//update the values, based upon the current menustate
     
     
  if(abs(RotaryEncoder2Val = knob2.read()))
     encoder2_update(RotaryEncoder2Val);
     
     
  if(abs(RotaryEncoder3Val = knob3.read()))
     encoder3_update(RotaryEncoder3Val);
     
     
  //write all zero values so as to allow that deviation from zero to be significant
  knob1.write(0);
  knob2.write(0);
  knob3.write(0);
 
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
     
//Encoder update functions - 
     //these will need to be sensitive to the current state of the menu, 
     //i.e. menustate will effect what the value is used for in each function
     
//relevant menustates -
     //6 - adjust parameters of band 1 of the parametric equalizer
     //7 - adjust parameters of band 2 of the parametric equalizer
     //8 - adjust parameters of band 3 of the parametric equalizer
     
     //9 - adjust the gains of each band of the graphic equalizer
     
     //10 - adjust the parameters of input 1's compression
     //11 - adjust the parameters of input 2's compression
     //12 - adjust the parameters of input 3's compression
     //13 - adjust the parameters of input 4's compression
     //14 - adjust the parameters of main output's compression
     //15 - adjust the parameters of aux output's compression
     
//parametric eq's have 
     //Q, gain, center_frequency
     
//graphic equalizer has
     //gain for 'bass', gain for 'mid', gain for 'high'
     
//compression has 
     //threshold(dB), ratio, third knob does nothing (or maybe makeup gain)
     

encoder1_update(int increment)
{
   switch(menustate)
   {

     default:
       break;
   }
}
encoder2_update(int increment)
{
  switch(menustate)
  {
      
     default:
       break;
  }
}
encoder3_update(int increment)
{
  switch(menustate)
  {
      
     default:
       break;
  }
}
     
update_screens()
{
  switch(menustate)
  {
    case 1://show the welcome screen
      break;
    case 2://show the parametric EQ top level menu option
      break;
    case 3://show the graphic EQ top level menu option
      break;
    case 4://show the compression top level menu option
      break;
    case 5://show the monitor channel select top level menu option
      break;
    case 6://show the band 1 parametric EQ selection
      break;
    case 7://show the band 2 parametric EQ selection
      break;
    case 8://show the band 3 parametric EQ selection
      break;
    case 9://show the bands of the graphic EQ
      break;
    case 10://show the input 1 compression options
      break;
    case 11://show the input 2 compression options
      break;
    case 12://show the input 3 compression options
      break;
    case 13://show the input 4 compression options
      break;
    case 14://show the main output compression options
      break;
    case 15://show the aux output compression options
      break;
    case 16://show the option to select input 1 for the headphone monitor
      break;
    case 17://show the option to select input 2 for the headphone monitor
      break;
    case 18://show the option to select input 3 for the headphone monitor
      break;
    case 19://show the option to select input 4 for the headphone monitor
      break;
    case 20://show the option to select main output for the headphone monitor
      break;
    case 21://show the option to select aux output for the headphone monitor
      break;
    default:
      break;
      
  }
}

void loop()                     
{//this will run in the time when the interrupt is not running

  //screens update
  update_screens();

  //check the values of all controls
  poll_controls();
}
