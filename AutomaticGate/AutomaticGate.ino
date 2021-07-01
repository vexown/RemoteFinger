//////////////////////////////////////*INCLUDES, DEFINES, DECLARATIONS*//////////////////////////////////////////////////////////////
#include <ThreeWire.h>  
#include <RtcDS1302.h>
#include <Wire.h>
#include <Servo.h>

#define DIR 6 // low/high switches direction
#define STEP 7 // LOW->HIGH = move one step
#define SLEEP 8 // LOW=Sleep, HIGH=Active
#define RESET 9 // enable/disable the H-bridge output: LOW=disable, HIGH=enable
#define LIGHT 10 //light relay
#define BLOCKER 11 //stopping mechanism servo: servo.write(0) = block, (90) = unblock
#define led 3
#define block 30
#define unblock 120
#define toggleDoorButton 14 //A0
#define testButton 15 //A1
#define autoManualSwitch 16 //A2
#define countof(a) (sizeof(a) / sizeof(a[0]))
#define eeprom_address 0x50 //default device address

uint16_t currentDayOfTheYear, oswiecenie1Time, oswiecenie2Time, otwarcieTime,zamkniecieTime,zgaszenie1Time,zgaszenie2Time;
boolean lightOn = 0;
boolean doorOpen;
boolean autoMode = 1;
boolean pendingToggleDoor = 1;
boolean pendingTest = 1;
uint16_t  daysFromMonths[] = {0,31,59,90,120,151,181,212,243,273,304,334}; 

ThreeWire myWire(4,5,2); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);
Servo myservo;

void writeData(unsigned int eaddress, uint16_t data);
uint16_t readData(unsigned int eaddress);
void openDoor();
void closeDoor();
void testSeq();

//////////////////////////////////////*SETUP*//////////////////////////////////////////////////////////////
void setup() 
{
  Serial.begin(57600);
  Rtc.Begin();
  Wire.begin();
  myservo.attach(BLOCKER);
  gpioInit();
  doorOpen = readData(2100);

}
//////////////////////////////////////*MAIN LOOP*//////////////////////////////////////////////////////////////
void loop() 
{

 //if power was reset, human input is required to run the machine
 pendingToggleDoor = digitalRead(toggleDoorButton);
 
 if(!pendingToggleDoor)
 {
  digitalWrite(led,HIGH);
  pendingToggleDoor = 1;
  delay(1000);
  
  while(1)
  {
      //read button states:
     autoMode = digitalRead(autoManualSwitch);
     pendingToggleDoor = digitalRead(toggleDoorButton);
     pendingTest = digitalRead(testButton);
     doorOpen = readData(2100);

    if(autoMode)
    {
      RtcDateTime now = Rtc.GetDateTime();
      for(int p=0;p<3;p++)
      {
        now = Rtc.GetDateTime();
        delay(20);
      }
      currentDayOfTheYear = now.Day() + daysFromMonths[(now.Month())-1];
      uint16_t currentTimeMinOnly = (now.Hour()*60) + now.Minute();
    
      if(currentDayOfTheYear>=366)
      {
        currentDayOfTheYear = 365;
      }
    
    //Check if it's sunrise yet and turn on the light for an hour if it is:
     oswiecenie1Time = (readData(currentDayOfTheYear*2))+120;
     zgaszenie1Time = (readData(734 + (currentDayOfTheYear*2)))+120;
     otwarcieTime = (readData(3000 + (currentDayOfTheYear*2)))+120;
     oswiecenie2Time = (readData(4000 + (currentDayOfTheYear*2)))+120;
     zgaszenie2Time = (readData(5000 + (currentDayOfTheYear*2)))+120;
     zamkniecieTime = (readData(6000 + (currentDayOfTheYear*2)))+120;
      
      if(currentTimeMinOnly>=oswiecenie1Time && currentTimeMinOnly<zgaszenie1Time && lightOn==0)
      {
        lightOn = 1;
        digitalWrite(LIGHT, HIGH);
      }
      if(currentTimeMinOnly>=zgaszenie1Time && currentTimeMinOnly<oswiecenie2Time && lightOn)
      {
        digitalWrite(LIGHT, LOW);
        lightOn = 0;
      }
      if(currentTimeMinOnly>=otwarcieTime && currentTimeMinOnly<zamkniecieTime && doorOpen==0)
      {
        openDoor(); 
      }
    
      //Check if it's sunset yet and turn on the light 0.5h before sunset for an hour:
      if(currentTimeMinOnly>=oswiecenie2Time && currentTimeMinOnly<zgaszenie2Time && lightOn==0)
      {
        lightOn = 1;
        digitalWrite(LIGHT, HIGH);
      }
      if(currentTimeMinOnly>=zgaszenie2Time && lightOn)
      {
        digitalWrite(LIGHT, LOW);
        lightOn = 0;
      }
       if(currentTimeMinOnly>=zamkniecieTime && doorOpen)
      {
        closeDoor();
      } 
      //buttons for changing rotations (more/less)
      if(!pendingToggleDoor)
      {
      delay(100);
      digitalWrite(led,LOW);
      delay(100);
      digitalWrite(led,HIGH);
        uint16_t steps = readData(2000);
        writeData(2000, (steps+100));
        delay(100);
      }
      if(!pendingTest)
      {
      delay(100);
      digitalWrite(led,LOW);
      delay(100);
      digitalWrite(led,HIGH);
        uint16_t steps = readData(2000);
        if(steps>0)
        {
           writeData(2000, (steps-100));
        }
        delay(50);
      }
    }
    else
    {
      if(!pendingToggleDoor)
      {
        doorOpen ? closeDoor() : openDoor();
        delay(100);
      }
      if(!pendingTest)
      {
        writeData(2100, 0);
        doorOpen = 0;
        
        testSeq();
        delay(100);
      }
    }
    
    if(doorOpen)
    {
      myservo.write(block);
    }
    else
    {
      myservo.write(unblock);
    }
      
      delay(100);
      }
  
 }
 
 if(doorOpen)
{
  myservo.write(block);
}
else
{
  myservo.write(unblock);
}
 delay(200);
}

//////////////////////////////////////*FUNCTIONS DEFINITIONS*//////////////////////////////////////////////////////////////

  void openDoor()
  {
    uint16_t steps = readData(2000);
    
    digitalWrite(SLEEP,1);
    digitalWrite(DIR,1);
    for(int i=0;i<(steps);i++) 
    {
    digitalWrite(STEP,1);
    delay(5);
    digitalWrite(STEP,0);
    delay(5);
    }
    myservo.write(block);
    delay(2000);
    myservo.write(block);
    digitalWrite(SLEEP,0); 

    writeData(2100, 1);
    myservo.write(block);
    doorOpen = 1;
  }
  
  void closeDoor()
  {
    uint16_t steps = readData(2000);
    
    digitalWrite(SLEEP,1);
    digitalWrite(DIR,0);
    myservo.write(unblock);
    delay(2000);
    for (int i = 0; i<10; i++)
    {
       myservo.write(unblock);
    }
    for(int i=0;i<(steps);i++)
    {
    digitalWrite(STEP,1);
    delay(5);
    digitalWrite(STEP,0);
    delay(5);
    }
    digitalWrite(SLEEP,0);

    writeData(2100, 0);
    doorOpen = 0;
  }
  
  // writes a byte of data in memory location eaddress
  void writeData(unsigned int eaddress, uint16_t data) 
  {
    //send first byte
    Wire.beginTransmission(eeprom_address);
    // set the pointer position
    Wire.write((int)(eaddress >> 8));
    Wire.write((int)(eaddress & 0xFF));
    Wire.write((byte)(data >> 8));
    Wire.endTransmission();
    delay(10);
    
    //send second byte
    Wire.beginTransmission(eeprom_address);
    // set the pointer position
    Wire.write((int)((eaddress+1) >> 8));
    Wire.write((int)((eaddress+1) & 0xFF));
    Wire.write((byte)(data & 0xFF));
    Wire.endTransmission();
    delay(10);
    
  }
   
  // reads a byte of data from memory location eaddress
  uint16_t readData(unsigned int eaddress) 
  {
    uint16_t result;
  
    //Read the first byte
    Wire.beginTransmission(eeprom_address);
    // set the pointer position
    Wire.write((int)(eaddress >> 8));
    Wire.write((int)(eaddress & 0xFF));
    Wire.endTransmission();
    Wire.requestFrom(eeprom_address,1); // get the byte of data
    delay(10);
    result = Wire.read() << 8;
  
    //Read the second byte
    Wire.beginTransmission(eeprom_address);
    // set the pointer position
    Wire.write((int)((eaddress+1) >> 8));
    Wire.write((int)((eaddress+1) & 0xFF));
    Wire.endTransmission();
    Wire.requestFrom(eeprom_address,1); // get the byte of data
    result = result | Wire.read();
    delay(10);
    return result;
  }
  
  void testSeq()
  {
    //light test:
    digitalWrite(LIGHT, HIGH);
    delay(10000); //10 sec delay
    digitalWrite(LIGHT, LOW);
    delay(10000);
    
    //door test:
    openDoor();
    delay(10000);
    closeDoor();
    delay(10000);
  
    //RTC test:
    if(Rtc.IsDateTimeValid() == 1)
    {
      digitalWrite(LIGHT, HIGH);
      delay(1000); //10 sec delay
      digitalWrite(LIGHT, LOW);
      delay(1000);
    }
  
  }
  
  void gpioInit()
  {
    pinMode(DIR,OUTPUT);
    pinMode(STEP,OUTPUT);
    pinMode(SLEEP,OUTPUT);
    pinMode(RESET,OUTPUT);
    pinMode(LIGHT, OUTPUT);
    pinMode(led, OUTPUT);
    pinMode(toggleDoorButton, INPUT_PULLUP);
    pinMode(testButton, INPUT_PULLUP);
    pinMode(autoManualSwitch, INPUT_PULLUP);
  
    digitalWrite(DIR,0);
    digitalWrite(SLEEP,0);
    digitalWrite(RESET, 1);
    digitalWrite(STEP, 0);
    digitalWrite(LIGHT, 0);
    digitalWrite(led, 0);
  
  }
