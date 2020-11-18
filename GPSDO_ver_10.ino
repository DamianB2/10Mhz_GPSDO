// **********************************************************************
// *           -----------   GPSDO_ver_x  ------------                  *
// * This code runs a GPS disciplined 10MHz signal source.  I uses the  *
// * 1 pps signal from a GPS module and uses it to measure cycles 		*
// * counted from a VCTCXO.  It then usses 2 PWM channel to create      *
// * adjust the frequency on the VCTXCO.  This code is written for a	*
// * Arduino Nano.														*
// **********************************************************************


// **********************************************************************
// *              -----------   INCLUDES  ------------                  *
// **********************************************************************
#include <LiquidCrystal_I2C.h>
#include "DigitalIO.h"


// **********************************************************************
// *              -----------   #defines  ------------                  *
// **********************************************************************
#define D0 0    //D0 = pin 1
#define D1 1    //D3 = pin 1
#define D2 2    //D2 = pin 2
#define D3 3    //D3 = pin 3
#define D4 4    //D4 = pin 4
#define D5 5    //D5 = pin 5
#define D6 6    //D6 = pin 6
#define D7 7    //D7 = pin 7
#define D8 8    //D8 = pin 8
#define D9 9    //D29 = pin 9
#define D10 10  //D10 = pin 10
#define D11 11  //D11 = pin 11
#define D12 12  //D12 = pin 12
#define D13 13  //D13 = pin 13

#define TenMeg 10000000l
#define state0  0
#define state1  1
#define state2  2

// **********************************************************************
// *              -----------   PROTOTYPES  ------------                *
// **********************************************************************
void captureCount();  // Interrupt routine
byte getCounterByte();
void generateStats1(unsigned long newCount);
void calState0();
void calState1();
void calState2();
void addCommasToLong(unsigned long num);
void writeLCD();


// **********************************************************************
// * ---  CREATE AN LCD OBJECT WITH THE LCD LiquidCrystal_I2C CLASS --- *
// * Wiring: SDA pin is connected to A4 and SCL pin to A5.				*
// * Connect to LCD via I2C, default address 0x27						*
// **********************************************************************
LiquidCrystal_I2C lcd(0x27,20,4);


// **********************************************************************
// *         -----------   VARIABLE DEFINITIONS  ------------           *
// **********************************************************************
// Types
//const enum stateName{state1, state2, state3} states;

// Global Variables
//stateName calState;
//stateName stateSeconds[] = {
//  [state1] = 1,
 // [state2] = 10,
 // [state3] = 100
//};

//struct stateInfo {
//  unsigned int second = {1,10,100};
  
//}
//stateInfo state[3];
byte calState = state0;  //keeps track of calibration state

int interruptFlag = 0;
unsigned long time;
unsigned int y[8];
byte byte1, byte2, byte3, byte4;
unsigned long totalCount;
unsigned long aveCountArray[3] = {0, 0, 0};
unsigned long aveCount;
long aveCountErr;  // Could be + or -
int i;
unsigned long readingNum = 0;

// latencyOffset: 1.00 usec (10 cycle) from read reg pulse to clr reg pulse
// Also ran test with up to 400 seconds to get trendline and y intercept
const int latencyOffset = 10; //10; 

float val;
int CourseValue;
int FineValue;

// Used for LCD stuff
char buf[20];

// **********************************************************************
// *          -----------   SETTINGS FOR STATES  ------------           *
// **********************************************************************
// Number of seconds to take a count reading
const int stateSeconds[3] = {1, 10, 100};

// The number of counts that the aveCount can vary from stateSeconds*TenMeg 
// If the error exceeds this the unit will be considered unlocked
const int stateLockedErr[3] = {10, 10, 15};

// The number of readings that must be within stateLockedErr to declare a lock
const int stateRequiredlockedCount[3] = {100, 100, 25};  // . This is currently only used on state2

// **********************************************************************
// *           -----------   PIN DEFINITIONS  ------------              *
// **********************************************************************
const int PWMCoursePin = 10;    // Coarse PWM Control Pin
const int PWMFinePin = 11;      // Fine PWM Control Pin

PinIO RCLK(13);
PinIO notCCLR(9);

const int selectA1 = D4;
const int selectB1 = D3;

const int pps = D2; 



//=======================================================================

// **********************************************************************
// *              -----------   setup  ------------                     *
// * This is the initialization part of the code.					    *
// * 															  		*
// **********************************************************************
void setup() {
  // put your setup code here, to run once:

	junk

   Serial.begin(9600);
   pinMode(pps, INPUT);

   pinMode(D5, INPUT);
   pinMode(D6, INPUT);
   pinMode(D6, INPUT);
   pinMode(D8, INPUT);
   pinMode(A0, INPUT);
   pinMode(A1, INPUT);
   pinMode(A2, INPUT);
   pinMode(A3, INPUT);

    pinMode(selectA1, OUTPUT);
    pinMode(selectB1, OUTPUT);

    notCCLR.config(OUTPUT,HIGH);  // 74LV8154 ~CCLR
    notCCLR.high();  //  74LV8154 ~CCLR initialize to high
    RCLK.mode(OUTPUT);  // 74LV8154 RCLK
    RCLK.low();  // 74LV8154 RCLK initialize to LOW

    attachInterrupt(digitalPinToInterrupt(pps), captureCount, RISING);

    CourseValue = 164;//164;//163;
    FineValue = 121;//251;
  
    Serial.println("");
    Serial.println("============ Start ===========");


    lcd.init();
    lcd.backlight();

    lcd.setCursor(0,0);
    lcd.print("    Starting up");
}


// **********************************************************************
// *              -----------   loop  ------------                      *
// * This is the MAIN part of the code.								    *
// * 															  		*
// **********************************************************************
void loop() {
  
  analogWrite(PWMCoursePin, CourseValue);
  analogWrite(PWMFinePin, FineValue);


  if (interruptFlag == 1) {   // If 1 an interrupt has occurred
   
    //Read 4th (highest) byte of 32 bit register
    digitalWrite(selectA1,HIGH);
    digitalWrite(selectB1,HIGH);
    byte4 = getCounterByte();

    //Read 3rd (second highest) byte of 32 bit register
    digitalWrite(selectA1,HIGH);
    digitalWrite(selectB1,LOW);
    byte3 = getCounterByte();

    //Read 2nd (second lowest) byte of 32 bit register
    digitalWrite(selectA1,LOW);
    digitalWrite(selectB1,HIGH);
    byte2 = getCounterByte();

    //Read 1st (lowest) byte of 32 bit register
    digitalWrite(selectA1,LOW);
    digitalWrite(selectB1,LOW);
    byte1 = getCounterByte();

    totalCount = byte4*0 +byte3*65536ul + byte2*256 + byte1;
    totalCount = byte4 << 8;
    totalCount = (totalCount | byte3) << 8;
    totalCount = (totalCount | byte2) << 8;
    totalCount = (totalCount | byte1);

    totalCount = totalCount + latencyOffset;   // A calibration for latency error due to interrupt time

    // Filter total Count by baxcar averaging 3 readings
    aveCountArray[0] = aveCountArray[1];
    aveCountArray[1] = aveCountArray[2];
    aveCountArray[2] = totalCount; 
    aveCount = (aveCountArray[0] + aveCountArray[1] + aveCountArray[2])/3;
    
    // Calulate the count error from the expected result
    aveCountErr = aveCount - (tenMeg * stateSeconds[calState]) ;
    
    // Call the appropriate state handling routine
    switch (calState) {
      case state0:
        calState0();
        break;
        
      case state1:
        calState1(); 
        break;
        
      case State2:
        calState2();
        break;      
    }

      
    interruptFlag = 0;
    readingNum++;  // Increment number of readings taken

  }


}


// **********************************************************************
// *         -----------   captureCount  ------------                   *
// *      ======  THIS IS THE COUNTER INTERRUPT ROUTINE  ======			*
// * This reads the current couny into a register on the 74LV8154	    *
// * It also then clears the counter.  This must run very fast as this  *															  		*
// * counts are missed between the time between the register capture    *
// * and the register clear.  This lost count is, refered to as latency *
// * is added back in in another part code.								*
// **********************************************************************
void captureCount() { // This is the pps interrupt routine
  static int intCount = 0;

  intCount++;
  if (intCount == stateSeconds[calState]) {
    
    
    interruptFlag = 1; // Set global flag that interrupt occurred
    
    // Toggle RCLK line to latch in count
    RCLK.toggle();  // 74LV8154 RCLK engaged ~250ns
    RCLK.toggle();  // 74LV8154 RCLK released  ~250ns

    // Toggle ~CCLR line to clear count    
    notCCLR.toggle(); //  74LV8154 ~CCLR engaged
    notCCLR.toggle(); //  74LV8154 ~CCLR  released
    
    intCount = 0;
  }  
    // ***** Counter restarts counting from zero here *****  
//Serial.println("in interrupt");
}  


// **********************************************************************
// *         -----------   getCounterByte  ------------                 *
// * This reads one byte from the 4 byte counter.					    *
// * 															  		*
// * 			                                                   	    *
// **********************************************************************
byte getCounterByte() {

    byte byteRead = 0;

    // Read 74LV8154 register
    //Byte select lines set befor call to this routine
    y[0] = digitalRead(A0);
    y[1] = digitalRead(A1);
    y[2] = digitalRead(A2);
    y[3] = digitalRead(A3);
    y[4] = digitalRead(D5);
    y[5] = digitalRead(D6);
    y[6] = digitalRead(D7);
    y[7] = digitalRead(D8);

    // Build the byte
    byteRead = y[7];
    for (i=6; i>=0; i--) {
      byteRead = byteRead << 1;
      byteRead = byteRead | y[i];
    }
    
    return byteRead;  
}


// **********************************************************************
// *         -----------   generateStats1  ------------                 *
// * This calculates various statistics and prints some to the serial   *
// * port.															  	*
// * 			                                                   	    *
// **********************************************************************
void generateStats1(unsigned long newCount) {
  static unsigned long totalStatCount;
  static int sampleNum = 0;
  int decimalPart;
  
  if (sampleNum > 2) {
      sampleNum = 0;
      totalStatCount = 0;
  }    

  sampleNum++;
  totalStatCount = totalStatCount + newCount; 

  Serial.println("");
   Serial.print("Reading Count = ");
  Serial.println(sampleNum); 
  Serial.print("Stat Total Count = ");
  Serial.println(totalStatCount);
  Serial.print("Average Freq = ");
  
  Serial.print(totalStatCount/sampleNum);
  Serial.print(".");
  decimalPart = (totalStatCount % sampleNum)*1000/sampleNum;
  if (decimalPart<100) Serial.print("0");
  if (decimalPart<10) Serial.print("0");
  Serial.println(decimalPart); 
}


// **********************************************************************
// *            -----------   calState0  ------------                   *
// * This is the first calibration state.  It will adjust the VCTCXO    *
// * as needed.  After a fixed number of readings it will move to the  	*
// * next state.                                                   	    *
// **********************************************************************
void calState0() {
  
  static int stateCount = 0;
  
  if (aveCount > 10000001) {
    // Adjust FineValue
    FineValue--;
  }
  else {
      if (aveCount < 9999999) {
        // Adjust FineValue
        FineValue++;
      }
  }

  if (FineValue > 255) {
    FineValue = 128;
    CourseValue ++;
  }
  else {
    if (FineValue < 0) {
      FineValue = 128;
      CourseValue --; 
    }
  }

  stateCount++;
  // Check to see if it is time to change to the next state
  if (stateCount >= stateLockedCount[0]) {
    calState = state1;
    aveCountArray[0] = TenMeg *stateSeconds[1];
    aveCountArray[1] = aveCountArray[0];
    aveCountArray[2] = aveCountArray[0];
  }
   writeLCD();

 if (stateCount >3) Serial.println(aveCount);
}


// **********************************************************************
// *            -----------   calState1  ------------                   *
// * This is the second calibration state.  It will adjust the VCTCXO   *
// * as needed.  After a fixed number of readings it will move to the  	*
// * next state.                                                   	    *
// **********************************************************************
void calState1() {
  //aveCount = totalCount; // ??? temporary ???
    static int stateCount = 0;
    
  if (aveCount > 100000001) {
    // Adjust FineValue
 //     Serial.println(FineValue);
    FineValue--;
 //   Serial.print(" Stage 2 Adjusted Down FineValue = ");
  }
  else {
      if (aveCount < 99999999) {
        // Adjust FineValue
 //       Serial.println(FineValue);
        FineValue++;
 //       Serial.print(" Stage 2 Adjusted Up FineValue = ");
      }
  }
  
  if (FineValue > 255) {
    FineValue = 128;
    CourseValue ++;
  }
  else {
    if (FineValue < 0) {
      FineValue = 128;
      CourseValue --; 
    }
  }
  
  // ??? Check that its in bounds or adjust CourseValue
  //Serial.print("Stage 2 Adjusted FineValue = ");
  
//  Serial.println(FineValue);
    stateCount++;
   if (stateCount >= stateLockedCount[1]) {
    calState = state2;
    aveCountArray[0] = TenMeg *stateSeconds[1];
    aveCountArray[1] = aveCountArray[0];
    aveCountArray[2] = aveCountArray[0];
  }
  writeLCD();

  if (stateCount >3) Serial.println(aveCount);
}


// **********************************************************************
// *            -----------   calState2  ------------                   *
// * This is the main running state.  It continous to adjust the VCTCXO *
// * as needed but the system is considered out of the calibration 		*
// * phase.  It will keep track of lock and unlocked status       	    *
// **********************************************************************
void calState2() {
  //aveCount = totalCount; // ??? temporary ???
  static int stateCount = 0;
  static int lockCount = 0;  // Counts how mant times in a row theaceCount stated within bounds
	
  // Check if aveCount is within bounds to consider it locked
  if (abs(aveCountErr) <= stateLockedCount[2]) {
    lockCount++;
  }
  else {
	     lockCount = 0;
  }
    
  // Adjust FineValue if needed    
  if (aveCountErr > 0) {
    FineValue--;
  }
  else
    if (aveCountErr < 0) {
      FineValue++;
     }

  // Check to see if we need to jump to the previous or next course value
  if (FineValue > 255) {
    FineValue = 128;  // Move FineValue to the center
    CourseValue ++;
  }
  else {
    if (FineValue < 0) {
      FineValue = 128;  // Move FineValue to the center
      CourseValue --; 
    }
  }
  
  stateCount++;

  // Write infor to the display LCD
  writeLCD();

  if (stateCount >3) Serial.print(aveCount);
   Serial.print("  ");
  Serial.print(aveCountErr) ;
  
  // Check to see if we consider the device locked to 10 MHz
  if (lockCount > stateRequiredlockedCount[2]) {
	  lockedState = LOCKED;
	  Serial.println("  Locked");
  }	  
  else {
	lockedState = LOCKED;
    Serial.println("  Unlocked");
  }
}


// **********************************************************************
// *         -----------   addCommasToLong  ------------                *
// * This converts a passed in long to a char buffer that contains      *
// * commas ever 3 digits.                                              *
// * May want to add decimal point to this later                        *
// **********************************************************************
void addCommasToLong(unsigned long num) {
  // Converts a long to a buffer with string of number with comma thousands seperators
  // Result is in global variable buf

  unsigned long billions, millions, thousands, hundreds;
  
  billions = num/1000000000;
  millions = (num/1000000) % 1000;
  thousands = (num % 1000000)/1000;
  hundreds = num % 1000;
  if (num < 1000000000) {
    snprintf(buf, sizeof(buf), "%ld,%03ld,%03ld   ", millions, thousands, hundreds);
  }
  else
  {
    snprintf(buf, sizeof(buf), "%ld,%03ld,%03ld,%03ld   ", billions, millions, thousands, hundreds);
  }
}


// **********************************************************************
// *             -----------   writeLCD  ------------                   *
// * This routine does all the writing to the 4x20 display LCD          *
// **********************************************************************
void writeLCD() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Freq = ");

  addCommasToLong(aveCount);  // Convert to string with commas
  
  lcd.print(buf);
  lcd.setCursor(0,1);
  lcd.blink();
  
  if lockedState = LOCKED lcd.print("  *** UNLOCKED ***");      
  else lcd.print("  *** UNLOCKED ***");
  
  lcd.noBlink();
  lcd.setCursor(0,2);
  lcd.print("PWM Course = ");
  lcd.print(CourseValue);
  lcd.setCursor(0,3);
  lcd.print("PWM Fine = ");
  lcd.print(FineValue);
}
