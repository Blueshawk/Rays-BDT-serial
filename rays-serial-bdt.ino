/* Ray's much more complicated than neccesary barn door tracker.


  Serial comm format = :LLnnn#
  Commands:
  S0 = stop
  SS = run sidreal
  SL = run lunar
  SO = run solar time
  SR = high speed return
  ST = time lapse mode
  T+ = increment speed trim for mode unit is in.
  T- = decrement speed trim for current mode.
  MSnnn = set max speed to nnn
  MXnnn = set max steps to nnn

  Returns format *LLnnn*
  GC Get current position count     ex: *C5030* = current position count
  GS Get current stepspersecond value    ex: *S206.5* = current speed

  Parts list:
   28byj-48 stepper motor
   spur and gear from an RC car (main shaft threaded 1/4-20)
   DRV8825 driver board
   Arduino nano v3.0
   12v battery
   5in piano hinge
   18" 1/4-20 threaded rod bent to 12" radius
   4 safety nuts and washers
   2- 14 inch boards hinged and drilled for bolt at 12"
*/

#include <AccelStepper.h>
#include <Arduino.h>
#include <EEPROM.h>                   // needed for EEPROM
#include "eepromanything.h"           // needed for EEPROM

#define EEPROMSIZE 1024               // ATMEGA328P 1024 EEPROM

//using a pin header between the nano and DRV8825 on the digital I/o side.
//Be careful to get it installed the right way.
//DRV8825 pins
#define Dir     5       // direction
#define Step    6      // step 
#define slp     7      // set high to run
#define rst     8       // set high to run
#define M2      9       // microstepping lines
#define M1      10       // microstepping lines
#define M0      11      // microstepping lines
#define Enable  12       //set low to run

// Setup motor class with parameters
static AccelStepper motor(AccelStepper::DRIVER, Step, Dir);

//Global variables
int trackMode = 0;
int ustep = 16; //microstep mode multiplier
long int maxSteps = 41690 * ustep;   // 50degrees --number of steps that equals max safe angle or rod.
float stepsPerSecond = 208.5 * ustep; //  3.475   208.5  ; //basic drive speed 3rpm on motor stud = 1rpm at screw
char inChar; //serial port characters
float trimSpeed; //speed modifier

struct config_t { //separate speed trim values - to be stored in eeprom
  int validdata;
  int trimSolar;
  int trimSidereal;
  int trimLunar;
  int trimTimelapse;
  long int returnSpeed; //this is also max speed but is only used for remote rewind
  long int maxSteps;
} savedata;

int datasize;               // size of the struct savedata (so it can change as needed)
int nlocations;             // number of storage locations available in EEPROM
int currentaddr;            // will be address in eeprom of the data stored
boolean writeNow;           // should we update values in eeprom
boolean found;              // did we find any stored values?
long prevMillis = 0L; // timer for screen data polling
long previousMillis = 0L;   // used as a delay whenever the EEPROM settings need to be updated
long myinterval = 5000L;   // interval in milliseconds to wait after a move before writing settings to EEPROM, 10s

//serial setup
#define MAXCOMMAND 20
char mycmd[MAXCOMMAND];         // these are for handling and processing serial commands
char param[MAXCOMMAND];         //
char line[MAXCOMMAND];          //
char proccessCmd;               //
int eoc = 0;                    // end of command
int idx = 0;                    // index into command string


void setup() {
  delay(500); //prevents code bricking.

  Serial.begin(9600);

  //define pinmodes
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(Enable, OUTPUT);
  pinMode(slp, OUTPUT);
  pinMode(rst, OUTPUT);
  //set outputs to enable motor
  digitalWrite(Enable, LOW); //disable=1 //enable motor run
  digitalWrite(slp, HIGH); //sleep=0
  digitalWrite(rst, HIGH); //rst=0

  //seet initial motor direction here. It can then be changed serially later, independent of count/position
  motor.setPinsInverted(false, false, false);  //directionInvert,stepInvert,enableInvert
  motor.setMaxSpeed(1000);

  eoc = 0;
  idx = 0;

  //setup for eeprom
  currentaddr = 0;    // start at 0 if not found later
  found = false;
  writeNow = false;
  datasize = sizeof( savedata );    // should be 14 bytes
  nlocations = EEPROMSIZE / datasize;  // for AT328P = 1024 / datasize = 73 locations

  for (int lp1 = 0; lp1 < nlocations; lp1++ )
  {
    int addr = lp1 * datasize;
    EEPROM_readAnything( addr, savedata );
    // check to see if the data is valid
    if ( savedata.validdata == 99 )
    {
      //data is okay so set found bit to load data
      currentaddr = addr;
      found = true;
    }
  }
  if ( found == true )
  { EEPROM_readAnything( currentaddr, savedata ); // restore saved settings
    savedata.validdata = 0;  //save 0 so wrong dataset won't be found on reboot
    EEPROM_writeAnything(currentaddr, savedata);    // update values in EEPROM
    // each time the program starts it will read current storage, and increment the start address
    // for eeprom writes so it doesnt destroy the eeprom as fast.
    // using it like an array of [0-nlocations], ie 100 storage locations for 1k EEPROM
    // set next free address for writing data
    currentaddr += datasize; //increment the address for future writes.
    // bound check the eeprom storage and if greater than last index [0-EEPROMSIZE-1] then set to 0
    if ( currentaddr >= (nlocations * datasize) )
      currentaddr = 0;
    writeNow = true; //write the restored dataset to the new eprom address when eeprom_update runs.
  }
  else
  {
    Serial.print ("load defaults");
    // set defaults because no data was found in eeprom
    savedata.validdata = 99;
    savedata.trimSolar = 1;
    savedata.trimLunar = 2;
    savedata.trimSidereal = 3;
    savedata.returnSpeed = 500;
    savedata.maxSteps = 50000;
    EEPROM_writeAnything(currentaddr, savedata);    // update values in EEPROM
    
  }
  //print out the saved data -
  Serial.println ("BDT-Serial");
  Serial.println (savedata.trimLunar);
  Serial.println (savedata.trimSolar);
  Serial.println (savedata.trimSidereal);
  Serial.println (savedata.returnSpeed);
  Serial.println (savedata.maxSteps);

} //end of setup

// m0/m1/m2 sets stepping mode 000 = F, 100 = 1/2, 010 = 1/4, 110 = 1/8, 001 = 1/16, 101 = 1/32
void uStepMode() {
  if (ustep = 1) {
    digitalWrite(M0, 0);
    digitalWrite(M1, 0);
    digitalWrite(M2, 0);
  }
  if (ustep = 2) {
    digitalWrite(M0, 1);
    digitalWrite(M1, 0);
    digitalWrite(M2, 0);
  }
  if (ustep = 4) {
    digitalWrite(M0, 0);
    digitalWrite(M1, 1);
    digitalWrite(M2, 0);
  }
  if (ustep = 8) {
    digitalWrite(M0, 1);
    digitalWrite(M1, 1);
    digitalWrite(M2, 0);
  }
  if (ustep = 16) {
    digitalWrite(M0, 0);
    digitalWrite(M1, 0);
    digitalWrite(M2, 1);
  }
  if (ustep = 32) {
    digitalWrite(M0, 1);
    digitalWrite(M1, 0);
    digitalWrite(M2, 1);
  }
}

void Tracking() {
  uStepMode(); //set microsteps
  if (motor.currentPosition() <= maxSteps) {
    digitalWrite(Enable, LOW);
    digitalWrite(slp, HIGH);
    motor.moveTo(maxSteps); //go toward arm max angle
    motor.setSpeed((stepsPerSecond) + trimSpeed); //basic speed +-trim for tracking type.
    motor.runSpeedToPosition();
  }
}

/*High speed return.
   Doing this at ustep=1 requires scaling the
   current position without calling ustepmode.
*/
void highSpeed() {
  long int  curPos = motor.currentPosition() / ustep; //rescale the count for ustep=1
  if (curPos = 0) {
    motor.stop();
    trackMode = 0;
  } else;
  digitalWrite(M0, 0); //set usteps to 1 for max reversing speed
  digitalWrite(M1, 0);
  digitalWrite(M2, 0);
  digitalWrite(Enable, LOW);
  digitalWrite(slp, HIGH);
  motor.moveTo(0); //number of steps to back up.
  savedata.returnSpeed = (-1 * savedata.returnSpeed); //this is also max speed :MSnnn#
  motor.setSpeed(savedata.returnSpeed); //-500 Return as quickly as motor will handle it.
  motor.runSpeedToPosition();
}


// SerialEvent occurs whenever new data comes in the serial RX.
void serialEvent()
{
  // : starts the command, # ends the command, do not store these in the command buffer
  // read the serial until the terminating # character
  bool eoc = 0;
  while (Serial.available() && !eoc)
  {
    inChar = Serial.read();
    if (inChar != '#' && inChar != ':')  {
      line[idx++] = inChar;
      if (idx >= MAXCOMMAND) {
        idx = MAXCOMMAND - 1;
      }
    }
    else
    {
      if (inChar == '#')
      {
        eoc = 1;
        idx = 0;
        // process the command string when a hash arrives:
        processCmd(line);
        eoc = 0;
      }
    }
  }
}         //end of serial event routine

// Process serial commands
void processCmd(String command)
{
  memset( mycmd, 0, MAXCOMMAND);
  memset(param, 0, MAXCOMMAND);
  int len = strlen(line);
  if (len >= 2) {
    strncpy( mycmd, line, 2);
  }
  if (len > 2)  {
    strncpy(param, line + 2, len - 2);
  }
  memset(line, 0, MAXCOMMAND);
  eoc = 0;
  idx = 0;

  //parse set commands
  // :S0# trackmode = stop
  if (!strcasecmp( mycmd, "S0")) {
    trackMode = 0; Serial.print (trackMode);
    Serial.print ("*M0*");
    return;
  }

  // :SS# set trackmode=sidereal time
  else if (!strcasecmp( mycmd, "SS")) {
    trackMode = 1;
    trimSpeed = savedata.trimSidereal;
    Serial.print ("*M1*");
    return;
  }

  //:SL#  set trackmode=Lunar time
  else if (!strcasecmp( mycmd, "SL")) {
    trackMode = 2;
    trimSpeed = savedata.trimLunar;
    Serial.print ("*M2*");
    return;
  }
  //:SO#  set trackmode=sOlar time
  else if (!strcasecmp( mycmd, "SO")) {
    trackMode = 3;
    trimSpeed = savedata.trimSolar;
    Serial.print ("*M3*");
    return;
  }

  //:ST#  set trackmode=timelapse
  else if (!strcasecmp( mycmd, "ST")) {
    trackMode = 4;
    Serial.print ("*M4*");
    return;
  }

  //:SR#  set trackmode=high speed return
  else if (!strcasecmp( mycmd, "SR")) {
    trackMode = 5;
    Serial.print ("*M4*");
    return;
  }

  //:T+#  increment trimSpeed for selected mode
  else if (!strcasecmp( mycmd, "T+")) {
    trimSpeed++;
    if (trackMode == 4) {
      savedata.trimTimelapse = trimSpeed;
    }
    if (trackMode == 3) {
      savedata.trimSolar = trimSpeed;
    }
    if (trackMode == 2) {
      savedata.trimLunar = trimSpeed;
    }
    if (trackMode == 1) {
      savedata.trimSidereal = trimSpeed;
    }
    Serial.print ("*T");
    Serial.print (trimSpeed);
    Serial.print ("*");

    writeNow = true;
    return;
  }

  //:T-#  increment trimSpeed for selected mode
  else if (!strcasecmp( mycmd, "T-")) {
    trimSpeed--;
    if (trackMode == 4) {
      savedata.trimTimelapse = trimSpeed;
    }
    if (trackMode == 3) {
      savedata.trimSolar = trimSpeed;
    }
    if (trackMode == 2) {
      savedata.trimLunar = trimSpeed;
    }
    if (trackMode == 1) {
      savedata.trimSidereal = trimSpeed;
    }
    Serial.print ("*T");
    Serial.print (trimSpeed);
    Serial.print ("*");
    return;
  }

  //:MSxxx# set max speed param = xxx
  else if (!strcasecmp( mycmd, "MS")) {
    savedata.returnSpeed = param;
    Serial.print (savedata.returnSpeed);
    writeNow = true;
    return;
  }
  /*:MXnnn# set max number of steps to take.
    Too high may flop your camera so use a safety nut or cap on the traverse screw as a stop.*/
  else if (!strcasecmp( mycmd, "MX")) {
    maxSteps = param;
    Serial.print (maxSteps);
    writeNow = true;
    return;
  }
  //:MR# reset count to 0 -- move door to home by hand
  else if (!strcasecmp( mycmd, "MR")) {
    motor.setCurrentPosition(0);
    Serial.print ("*D0*");
    return;
  }
  //:DF# set motor direction forward
  else if (!strcasecmp( mycmd, "DF")) {
    motor.setPinsInverted(false, false, false);  //(directionInvert,stepInvert,enableInvert
    Serial.print ("*FWD*");
    return;
  }
  //:DR# set motor direction reverse
  else if (!strcasecmp( mycmd, "DF")) {
    motor.setPinsInverted(true, false, false);  //(directionInvert,stepInvert,enableInvert
    Serial.print ("*REV*");
    return;
  }

  /*****parse get info commands****/

  //:GM# Get max speed
  else if (!strcasecmp( mycmd, "GM")) {
    Serial.print (savedata.returnSpeed);
    return;
  }
  //:GT# Get trackmode
  else if (!strcasecmp( mycmd, "GT")) {
    Serial.print ("*M");
    Serial.print (trackMode);
    Serial.print ("*");
    return;
  }

  //:GC# Get current position count
  else if (!strcasecmp( mycmd, "GC")) {
    double tmp = (motor.currentPosition());
    Serial.print ("*D");
    Serial.print (tmp);
    Serial.print ("*");
    return;
  }

  //:GS# Get current speed value in stepspersec
  else if (!strcasecmp( mycmd, "GS")) {
    float curSpeed = stepsPerSecond + trimSpeed;
    Serial.print ("*S");
    Serial.print (curSpeed);
    Serial.print ("*");
    return;
  }
  //:GX# Get Max Steps
  else if (!strcasecmp( mycmd, "GX")) {
    Serial.print ("*X");
    Serial.print (maxSteps);
    Serial.print ("*");
    return;
  }
}

void updateEeprom() {
  // is it time to update EEPROM settings?
  if ( writeNow == true )
  {
    // decide if we have waited 5s after the last change, if so, update the EEPROM
    long currentMillis = millis();
    if ( ((currentMillis - previousMillis) > myinterval) && (writeNow == true) )
    {
      // copy current settings and write the data to EEPROM
      if (trackMode == 1) {
        savedata.trimSidereal = trimSpeed;
      }
      if (trackMode == 2) {
        savedata.trimLunar = trimSpeed;
      }
      if (trackMode == 3) {
        savedata.trimSolar = trimSpeed;
      }
      if (trackMode == 4) {
        savedata.trimTimelapse = trimSpeed;
      }
      savedata.maxSteps = maxSteps;

      savedata.validdata = 99;
      EEPROM_writeAnything(currentaddr, savedata);    // update values in EEPROM
      writeNow = false;
      previousMillis = currentMillis;    // update the timestamp
      Serial.print("saved data");
    }
  }
}


void pollData() {

  // decide if we have waited 5s after the last change, if so, update the screen
  long curMillis = millis();
  if ((curMillis - prevMillis) > myinterval / 5) {
    double tmp = (motor.currentPosition() / ustep);
    Serial.print ("*D");
    Serial.print (tmp);
    Serial.print ("*");
    delay(1);
    Serial.print ("*T");
    Serial.print (trimSpeed);
    Serial.print ("*");
    delay(1);
    /* float curSpeed = stepsPerSecond + trimSpeed;
      Serial.print ("*S");
      Serial.print (curSpeed);
      Serial.print ("*");
      delay(1);*/
    Serial.print ("*M");
    if (trackMode == 0) Serial.print ("Stop");
    if (trackMode == 1) Serial.print ("Sidereal");
    if (trackMode == 2) Serial.print ("Lunar");
    if (trackMode == 3) Serial.print ("Solar");
    if (trackMode == 5) Serial.print ("Rewind");
    Serial.print ("*");
    prevMillis = curMillis;    // update the timestamp
  }
}

void loop() {
  pollData();
  if (writeNow == true) {
    updateEeprom();
  }
  if  (trackMode == 0) { //stop
    motor.stop();
    digitalWrite(Enable, HIGH);
    digitalWrite(slp, LOW);
    return;
  }
  if (trackMode == 4) { //rewind
    highSpeed(); //rewind motor
    return;
  }
  if (trackMode == 3) {
    trimSpeed = savedata.trimSolar;
  }
  if (trackMode == 2) {
    trimSpeed = savedata.trimLunar;
  }
  if (trackMode == 1) {
    trimSpeed = savedata.trimSidereal;
  }
  Tracking(); //move motor

}// end of loop
