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
 *    29,30,31,24 are the Channel 1 to Channel 4 Mutes (latching buttons) - pin 31 is used, we need to move channel 3 mute
 *    27 and 28 are the Main and Aux Mix Mute buttons  (latching buttons)
 *    9 and 10, 25 and 26, 54 and 55 are pairs for each of the rotary encoders
 *    33-36 are the four menu  buttons (momentary buttons)
 *
 *  Screens
 *    I2C Bus number zero has SCL on pin 7 and SDA on pin 8
 *    I2C Bus number one has SCL on pin 37 and SDA on pin 38
 *    I2C Bus number two has SCL on pin 3 and SDA on pin 4
 *    I2C Bus number three has SCL on pin 57 and SDA on pin 56
 * 
 *    SPI bus number 0 has MOSI on pin 11, MISO on pin 12, SCK on pin 13
 *    SPI bus number 1 has MOSI on pin 0, MISO on pin 1, SCK on pin 32
 *    SPI bus number 2 has MOSI on pin 44, MISO on pin 45, SCK on pin 46
*/

#include <FrequencyTimer2.h>
#include <Arduino.h>
#include <SPI.h>
#include <Encoder.h>
#include <Bounce.h>
#include <math.h>
#include "Adafruit_GFX.h"
#include "Adafruit_HX8357.h"
#include "Adafruit_SSD1306.h"

int spi_speed = 30000000;
IntervalTimer ADC_timed_interrupt;

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


//this allows the user to apply the parametric EQs to each of the input channels
int currently_selected_menu_channel;

//this keeps the number of the channel that the headphone monitor is tapped into
int currently_selected_headphone_channel;

//filter coefficients
//how do these look, how many are there, where are they kept


//define main display
#define TFT_CS 31
#define TFT_DC 30 //this needs to change
#define MOSI 0
#define SCK 32
#define TFT_RST -1 // RST can be set to -1 if you tie it to Arduino's reset
#define MISO 1

Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, MOSI, SCK, TFT_RST, MISO);




#define NUM_SAMPLES 20 //is this enough?

//these will be circular buffers used to draw out the waveform
uint16_t channel1_samples[NUM_SAMPLES];
uint16_t channel2_samples[NUM_SAMPLES];
uint16_t channel3_samples[NUM_SAMPLES];
uint16_t channel4_samples[NUM_SAMPLES];

void setup() 
{//initialization and setup of all initial values and initial state
 
  samples_buffer_current_index = 0;
  
  //initializing the circular buffers associated with each of the input channels
  //these values will be used to draw out the waveform
  for(int i = 0; i < NUM_SAMPLES; i++)
  {//initialize with zeroes
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
 
  //initialize with the current channel for the menu set to 1
  currently_selected_menu_channel = 1;
 
  //initialize the value for the current headphone channel
  currently_selected_headphone_channel = 0; //not listening to any channel
  
  //set up screens
  tft.begin();
  tft.fillScreen(HX8357_WHITE);
  tft.setRotation(1);
 
  //poll initial values of all controls
  poll_controls();

  //initialize filters with zero values
  //look at the MATLAB code for the eq

  //setup the timed interrupt with intervaltimer
  ADC_timed_interrupt.begin(sample,21);
 
 
 //initalize DAC
  analogWriteFrequency(A21,4000000);
  analogWriteFrequency(A22,4000000);
  analogWriteResolution(16);
 
  menustate = 1;
}

//these hold the values read from the ADCs
volatile uint16_t current_sample1;
volatile uint16_t current_sample2;
volatile uint16_t current_sample3;
volatile uint16_t current_sample4;

void sample()
{//this function is attached to the timed interrupt
  digitalWrite(33, HIGH);//begin converting
  delayMicroseconds(1);
  SPI.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
 
  current_sample1 = SPI.transfer16(0);
  current_sample2 = SPI.transfer16(0);
  current_sample3 = SPI.transfer16(0);
  current_sample4 = SPI.transfer16(0);
 
  SPI.endTransaction();
  digitalWrite(33, LOW);//converting finished
 
  //store these samples
  channel1_samples[samples_buffer_current_index] = 0;
  channel2_samples[samples_buffer_current_index] = 0;
  channel3_samples[samples_buffer_current_index] = 0;
  channel4_samples[samples_buffer_current_index] = 0;
 
  //increment the index for the circular buffer, and reset to zero if it is past the bounds
  samples_buffer_current_index++;
  if(samples_buffer_current_index == NUM_SAMPLES)
    samples_buffer_current_index = 0;
 
 
  //How are we going to do all these?
  
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
 
  //OUTPUT THE APPROPRIATE CHANNEL FOR THE HEADPHONES
  //OUTPUT THE MAIN AND AUX MIXES
  switch(samples_buffer_current_index){
   case 1:
    //output input 1's value
    break;
   case 2:
    //output input 2's value
    break;
   case 3:
    //output input 3's value
    break;
   case 4:
    //output input 4's value
    break;
   case 5:
    //output main output's value
    break;
   case 6:
    //output aux mix's value
    break;
   default:
    break;//i.e. ==0, listening to nothing
   }
  
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
    case 22://wraparound for the monitor channel select submenu
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
      menustate = 22;
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
    case 6:
    case 7:
    case 8:
    case 9://IN THE PARAMETRIC EQ SETTINGS, THIS HAS TO CHANGE THE SELECTED CHANNEL, AS WELL
      currently_selected_menu_channel++; //move to the next channel
      if(currently_selected_menu_channel == 5) //if it's past the end, reset it to the first channel
        currently_selected_menu_channel = 1;        
      break;
   case 16: //set the headphone monitor to listen to input 1
      
      break;
   case 17: //set the headphone monitor to listen to input 2
      break;
   case 18: //set the headphone monitor to listen to input 3
      break;
   case 19: //set the headphone monitor to listen to input 4
      break;
   case 20: //set the headphone monitor to listen to main out
      break;
   case 21: //set the headphone monitor to listen to aux out
      break;
   case 22: //set the headphone monitor to listen to nothing
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
  int srate = 48000;
  
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
  if(abs(RotaryEncoder1Val = knob1.read())) encoder1_update(RotaryEncoder1Val);//update the values, based upon the current menustate
     
  if(abs(RotaryEncoder2Val = knob2.read())) encoder2_update(RotaryEncoder2Val);
    
  if(abs(RotaryEncoder3Val = knob3.read())) encoder3_update(RotaryEncoder3Val);
    
     
 
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
     //threshold(dB), ratio, third knob does nothing
     

encoder1_update(int increment)
{
   knob1.write(0); //write zero value so as to allow that deviation from zero to be significant
   switch(menustate)//depends on where you are in the menu
   {

     default:
       break;
   }
}
encoder2_update(int increment)
{
  knob2.write(0);//write zero value so as to allow that deviation from zero to be significant
  switch(menustate)//depends on where you are in the menu
  {
      
     default:
       break;
  }
}
encoder3_update(int increment)
{
  knob3.write(0);//write zero value so as to allow that deviation from zero to be significant
  switch(menustate)//depends on where you are in the menu
  {
      
     default:
       break;
  }
}
     
update_screens()
{
  tft.fillScreen(HX8357_WHITE);
  switch(menustate)
  {
    case 1://show the welcome screen
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("ALLEN&KEITH\nDIGITAL AUDIO MIXER");
      break;
    case 2://show the parametric EQ top level menu option
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Parametric EQ");
      break;
    case 3://show the graphic EQ top level menu option
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Graphic EQ");
      break;
    case 4://show the compression top level menu option
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Compression");
      break;
    case 5://show the monitor channel select top level menu option
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Monitor Channel Select");
      break;
    case 6://show the band 1 parametric EQ selection
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Band 1");
      break;
    case 7://show the band 2 parametric EQ selection
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Band 2");
      break;
    case 8://show the band 3 parametric EQ selection
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Band 3");
      break;
    case 9://show the bands of the graphic EQ
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Graphic EQ Bands");
      break;
    case 10://show the input 1 compression options
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Compression Settings - Input 1");
      break;
    case 11://show the input 2 compression options
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Compression Settings - Input 2");
      break;
    case 12://show the input 3 compression options
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Compression Settings - Input 3");
      break;
    case 13://show the input 4 compression options
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Compression Settings - Input 4");
      break;
    case 14://show the main output compression options
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Compression Settings - Main Output");
      break;
    case 15://show the aux output compression options
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Compression Settings - Aux Output");
      break;
    case 16://show the option to select input 1 for the headphone monitor
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Input 1");
      break;
    case 17://show the option to select input 2 for the headphone monitor
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Input 2");
      break;
    case 18://show the option to select input 3 for the headphone monitor
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Input 3");
      break;
    case 19://show the option to select input 4 for the headphone monitor
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Input 4");
      break;
    case 20://show the option to select main output for the headphone monitor
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Main Output");
      break;
    case 21://show the option to select aux output for the headphone monitor
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Aux Output");
      break;
    case 22://show the option to turn off the headphone monitor
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("None");
      break;
    default:
      break;
      
  }
}

void loop()                     
{//this will run in the time when the interrupt is not running
 //maindisplay
 
  //screens update
  update_screens();
 

  //check the values of all controls
  poll_controls();
}
