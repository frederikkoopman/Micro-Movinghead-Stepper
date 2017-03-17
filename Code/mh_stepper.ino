
 /* This code is highly influenced by the example codes of the Conceptinetics (https://www.tindie.com/products/Conceptinetics/dmx-shield-for-arduino-remote-device-management-capable/) libarie. 
   Thanks for the example code. 
   
  Copyright (c) 2016 Micro movinghead project. All right reserved.

   This code is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 3 of the License, or (at your option) any later version.

   This code is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

// Import of libaries
#include <Conceptinetics.h>
#include <TM1637Display.h>
#include <EEPROM.h>

// Set number of DMX channels
#define DMX_SLAVE_CHANNELS 9

// Set RX enable
#define RXEN_PIN                2

// Strobo configuration
#define STROBO_start_level 10 // Value of the DMX strobo channel needs to exceed to start strobo
#define STROBO_flash_offset 51 // Offset of length of flash in ms
#define STROBO_flash_ratio -6 // Inverse ratio of DMX strobo level and length of flash in ms
#define STROBO_blackout_offset 4130 // Offset of length of blackout in ms
#define STROBO_blackout_ratio -16 // Inverse ratio of DMX strobo level and length of blackout in ms

// Display connection pins (Digital Pins)
#define CLK A5
#define DIO A4

// initiation of DMX slave controller
DMX_Slave dmx_slave ( DMX_SLAVE_CHANNELS , RXEN_PIN );

// Pin numbers of the led output
#define RED_out 6
#define GREEN_out 5
#define BLUE_out 3

// Set motorpins of the steppers
#define pan_pin_1 4
#define pan_pin_2 13
#define pan_pin_3 12
#define pan_pin_4 11
#define tilt_pin_1 10
#define tilt_pin_2 9
#define tilt_pin_3 8
#define tilt_pin_4 7

// Set input pins
#define buttonup A1
#define buttondown A0
#define pansensor A2
#define tiltsensor A3

// Set delay between steps in ms
#define stepperdelay 2

// Delay between buttons pressed and displaytimeout in ms
#define startbuttondelay 300
#define displaytimeout 5000

// initiation of display
TM1637Display display(CLK, DIO);

// Decalaration and initiation of global variables

// Color channels
int RED = 0;
int GREEN = 0;
int BLUE = 0;

// Dimmer and strobo channel
int DIM = 0;
int STROBO = 0;

// DMX start adress
int adress;

// timing variables
unsigned long now;
unsigned long start_led;
unsigned long buttonreleased;
unsigned long laststep[2];
unsigned long lastFrameReceivedTime;
const unsigned long dmxTimeoutMillis = 2000UL;
int buttondelay=startbuttondelay;

// display settings
bool screenon=true;
const uint8_t emptydisplay[] = { 0x00, 0x00, 0x00, 0x00 };

// stepper variables
unsigned int currentstep[2];
unsigned int wantedstep[2];

void setup()
{
  // Set buttons and sensors as inputs
  pinMode(buttonup,INPUT);
  pinMode(buttondown,INPUT);
  pinMode(pansensor,INPUT);
  pinMode(tiltsensor,INPUT);
  // Set pull-up resistors
  digitalWrite(buttonup,HIGH);
  digitalWrite(buttondown,HIGH);
  digitalWrite(pansensor,HIGH);
  digitalWrite(tiltsensor,HIGH);

  // Set motorpins as output
  pinMode(pan_pin_1, OUTPUT);
  pinMode(pan_pin_2, OUTPUT);
  pinMode(pan_pin_3, OUTPUT);
  pinMode(pan_pin_4, OUTPUT);
  pinMode(tilt_pin_1, OUTPUT);
  pinMode(tilt_pin_2, OUTPUT);
  pinMode(tilt_pin_3, OUTPUT);
  pinMode(tilt_pin_4, OUTPUT);
  
  // get old adress and display it
  adress=EEPROM.read(0);;
  display.setBrightness(0x0f);
  display.showNumberDec(adress, false);
  buttonreleased=millis();

  // Enable DMX slave interface and start recording
  // DMX data
  dmx_slave.enable ();  
    
  // Set startadress
  dmx_slave.setStartAddress (adress);

  // Set led pins as output pins
  pinMode ( RED_out, OUTPUT );
  pinMode ( GREEN_out, OUTPUT );
  pinMode ( BLUE_out, OUTPUT );

  calibrateSteppers();

  // DMX timeout setting
  dmx_slave.onReceiveComplete ( OnFrameReceiveComplete );
}

void loop()
{
  // In case of user input 
  while(digitalRead(buttonup)==LOW or digitalRead(buttondown)==LOW){
    changeadress();
  }
  // In case of no new user input set screen off after a timeout and reset buttondelay
  screentimeout(); 

  now=millis(); // Get new now time
    if ( now - lastFrameReceivedTime < dmxTimeoutMillis ) {
      // In case of no timeout the DMX data is stored in internal variables
      // NOTE:
      // getChannelValue is relative to the configured startaddress
      
      // Reading (new) DMX data
      // steppers are conntrolled by a 12-bit value build from bitshifting 2 dmx channels and limiting to max 3600 (since the steppers can not do a full rotation
      wantedstep[0]=((dmx_slave.getChannelValue (1))<<4)+((dmx_slave.getChannelValue (2))>>4);
      wantedstep[1]=((dmx_slave.getChannelValue (3))<<4)+((dmx_slave.getChannelValue (4))>>4);
      wantedstep[0]=min(wantedstep[0],3500);
      wantedstep[1]=min(wantedstep[1],3500);
      
      // led values
      RED = dmx_slave.getChannelValue (5);   
      GREEN = dmx_slave.getChannelValue (6);
      BLUE = dmx_slave.getChannelValue (7);  
      DIM = dmx_slave.getChannelValue (8);
      STROBO = dmx_slave.getChannelValue (9);

      // set led
      setled();
  } else {
    // In case of DMX timeout the movingheads will go to there default position (leds off, and steppers in mid position)
    analogWrite(RED_out,0);  
    analogWrite(GREEN_out,0);
    analogWrite(BLUE_out,0);
    wantedstep[0]=2048;
    wantedstep[1]=2048;    
  }
  // move steppers 1 step if nessarry
  movestepper();
}

void changeadress() {
  // if screen is already on change DMX adress
  if (screenon==true){
      if (digitalRead(buttonup)==LOW){
        if (adress==511) {
          adress=1;
        } else {
          adress++;
        }
      } else if (digitalRead(buttondown)==LOW){
        if (adress==1) {
          adress=511;
        } else {
          adress--;
        }
      }
  }
  
  // wait for button release or continue with a faster adress change
  delay(buttondelay);
  if (buttondelay!=10){
    buttondelay=buttondelay-10;
  }

  // Show new adress on display
  display.showNumberDec(adress, false);
  // Set new time for last time button pressed
  buttonreleased=millis();
  // 
  screenon=true;
}


void screentimeout(){
  // reset buttondelay
  buttondelay=startbuttondelay;
  // check if display is on. in case of on check if it should be set off and store adress in epprom
  if (screenon==true){
    now=millis();
    if (now-buttonreleased>displaytimeout){
      // set display off
      display.setSegments(emptydisplay);
      screenon=false;
      // store adress in eeprom (save is only done here to reduce eeprom write cycles)
      EEPROM.put(0, adress);   
    }
  }
}


void movestepper() {
  // do algorithm for both pan and tilt stepper
  for(int i=0;i<2;i++){
    // check if next step may be set
    now=millis();
    if((now-laststep[i])>stepperdelay){
        // check if a step need to be set and which direction else do nothing
        if((currentstep[i]<wantedstep[i])){
            // save the new location of the stepper
            currentstep[i]++;
            // set the step
            setstep((currentstep[i] % 8),i);
            // save time of last step done 
            laststep[i]=millis();
        } else if ((currentstep[i]>wantedstep[i])){
            // comments same as above 
            currentstep[i]--;
            setstep((currentstep[i] % 8),i);
            laststep[i]=millis();
        }
      }
    }
}

void setstep(int Step,int pan){
  // function to set a step for a steppermotor
  //collect the correct motorpins
  int motor_pin_1;
  int motor_pin_2;
  int motor_pin_3;
  int motor_pin_4;
  if (pan==0){
    motor_pin_1=pan_pin_1;
    motor_pin_2=pan_pin_2;
    motor_pin_3=pan_pin_3;
    motor_pin_4=pan_pin_4;
  } else {
    motor_pin_1=tilt_pin_1;
    motor_pin_2=tilt_pin_2;
    motor_pin_3=tilt_pin_3;
    motor_pin_4=tilt_pin_4;
  }
  
  // set the motorpins to step
      switch (Step) {
   case 0:
     digitalWrite(motor_pin_1, LOW); 
     digitalWrite(motor_pin_2, LOW);
     digitalWrite(motor_pin_3, LOW);
     digitalWrite(motor_pin_4, HIGH);
   break; 
   case 1:
     digitalWrite(motor_pin_1, LOW); 
     digitalWrite(motor_pin_2, LOW);
     digitalWrite(motor_pin_3, HIGH);
     digitalWrite(motor_pin_4, HIGH);
   break; 
   case 2:
     digitalWrite(motor_pin_1, LOW); 
     digitalWrite(motor_pin_2, LOW);
     digitalWrite(motor_pin_3, HIGH);
     digitalWrite(motor_pin_4, LOW);
   break; 
   case 3:
     digitalWrite(motor_pin_1, LOW); 
     digitalWrite(motor_pin_2, HIGH);
     digitalWrite(motor_pin_3, HIGH);
     digitalWrite(motor_pin_4, LOW);
   break; 
   case 4:
     digitalWrite(motor_pin_1, LOW); 
     digitalWrite(motor_pin_2, HIGH);
     digitalWrite(motor_pin_3, LOW);
     digitalWrite(motor_pin_4, LOW);
   break; 
   case 5:
     digitalWrite(motor_pin_1, HIGH); 
     digitalWrite(motor_pin_2, HIGH);
     digitalWrite(motor_pin_3, LOW);
     digitalWrite(motor_pin_4, LOW);
   break; 
     case 6:
     digitalWrite(motor_pin_1, HIGH); 
     digitalWrite(motor_pin_2, LOW);
     digitalWrite(motor_pin_3, LOW);
     digitalWrite(motor_pin_4, LOW);
   break; 
   case 7:
     digitalWrite(motor_pin_1, HIGH); 
     digitalWrite(motor_pin_2, LOW);
     digitalWrite(motor_pin_3, LOW);
     digitalWrite(motor_pin_4, HIGH);
   break; 
      }
}

void setled(){
    if (STROBO < STROBO_start_level) {
      // In case of no strobo (Strobo channel is below STROBO_start_level) the led is set by it's color channel and a dimmer channel. Also the strobo timing clock will be reset
      PWM_intensities(RED,GREEN,BLUE,DIM);
      start_led = millis();
    } else {
      now = millis();
      if ((now-start_led) < STROBO_flash_offset+STROBO/STROBO_flash_ratio) {
        // In case a strobo is active then there will first be a flash (controlled by the colorchannels and dimmer channel) of STROBO_flash_length
        PWM_intensities(RED,GREEN,BLUE,DIM);
      } else if (now-start_led < STROBO_blackout_offset+(STROBO_blackout_ratio*STROBO)) {
        // After which a blackout will happen. It's time is depended on the value of the strobo channel. A lower value will result in a longer blackout
        analogWrite(RED_out,0);  
        analogWrite(GREEN_out,0);
        analogWrite(BLUE_out,0);
      } else {
        // After the blackout happened the led will reset the strobo timing clock and start a new flash
        start_led = millis();
        PWM_intensities(RED,GREEN,BLUE,DIM);
      }
    }
}

void OnFrameReceiveComplete (unsigned short channelsReceived)
{
  // Update receive time to determine signal timeout
  lastFrameReceivedTime = millis ();
}

// Calculation and setting of PWM intensities
void PWM_intensities(int r,int g, int b, int dimmer){
      analogWrite(RED_out,r*dimmer/255);  
      analogWrite(GREEN_out,g*dimmer/255);
      analogWrite(BLUE_out,b*dimmer/255);
}

void calibrateSteppers(){
  // calibrate steppers
  currentstep[0]=8193; 
  currentstep[1]=8193;
  wantedstep[0]=0; // ask te steppers to move to 2 full rotation back, the move is terminated after the zero position is found
  wantedstep[1]=0;
  movestepper(); // start steppers at a step zero (modulo 8=0)
  bool beginFound[2];
  beginFound[0]=false;
  beginFound[1]=false;

  while(!beginFound[0] or !beginFound[1]){
    // move the steppers untill they reach the zero position (switch triggered and step zero)
    if (!digitalRead(pansensor) and ((currentstep[0]%8)==0)){
      beginFound[0]=true;
      currentstep[0]=0;
    }
    if (!digitalRead(tiltsensor) and ((currentstep[1]%8)==0)){
      beginFound[1]=true;
      currentstep[1]=0;
    }
    movestepper();
  }
}
