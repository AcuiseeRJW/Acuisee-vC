
// EVENTS ******************************************************************************************************
// a - left front PB pushed
// b - left front pb released
// c - right front pb pushed
// d - right front pb released
// e - step fwd manual pb pushed
// f - step fwd manual pb released
// g - step rev manual pb pushed
// h - step rev manual pb released
// i - step returned ls pushed
// j - step returned ls released
// k - retract begin
// l - retract complete
// m - ls fault
// n - step full forward (opposite of i)
// s - bad BT character received . . . reSend

// COMMANDS *****************************************************************************************************

// S - stats
// R - reset
// X - software restart
// Z - noop (returns a z)
// 1 thru 9 - rotate and deliver # ml of paste from syringe

// **************************************************************************************************************
//          Acuisee -vC  Veterinary (canine -vC) Vision Unit
// **************************************************************************************************************

// HISTORY ******************************************************************************************************
// date          who  ver   what
// ----------------------------------------------------------------------------------------------------
// 10.23.2019    rjw  8.0.0 Began total rework of -vR original code to work for the veterinary (-vC) canine unit
// 10.25.2016     rjw 8.0.5 implemented steo counter and soft stop software, 1-9 travel commands

// HISTORY ******************************************************************************************************

// **************************************************************************************************************
//          Acuisee -vC  Veterinary (canine -vC) Vision Unit
// **************************************************************************************************************

#include <SoftwareSerial.h>
#define BluetoothRx    2   // RX-I
#define BluetoothTx    3   // TX-O
#define steps_per_ml   450 // steps per ml of travel
SoftwareSerial Bluetooth(BluetoothTx, BluetoothRx);

// VARIABLE DECLARATIONS ****************************************************************************************

const String  Version            = "8.0.5";

const byte    leftPBled          =  9 ;
const byte    rightPBled         =  11;
const byte    runLED             =  13;
const byte    rightPB            =  14;
const byte    stepRetLS          =  15;
const byte    leftPB             =  16;
const byte    stepFwdPB          =  17;
const byte    stepRevPB          =  18;
const byte    STEP_PIN           = 4  ;           // Stepper board pwm pin
const byte    DIR_PIN            = 5  ;           //Stepper board direction pin

const int     interval           = 1000;          // interval at which to blink (milliseconds)
const int     step_size          = 8 ;            // 1/16 (16), 1/8 (8), 1/4 (4), 1/2 (2), 1/1 (1)
const int     steps_per_rev      = 42 ;
const int     steps_total        = step_size * steps_per_rev;  // total steps at resolution of stepper driver settings
const int     delay_time         = 100 ;          // time to delay between step waveform pulses - greater the microstep, less time needed

const long    maxSteps           = 14900 ;        // steps for full travel
const long    maxTravelTime      = 15000;         // 15 sec maximum stepper up travel time


unsigned long currentMillis      = millis();      // current clock value
unsigned long previousMillis     = 0;             // will store last time run LED was updated
unsigned long badCharTimer       = millis();

long          lastDebounceTime   = 0;             // the last time the input check was made
long          debounceDelay      = 25;            // the debounce time 50ms=.0050sec
long          badCharTimerLimit  = 500;
long          accumulatedTravelTime = 0;          // variable to time stepper motion
long          stepCounter        = 0;
long          prevstepCounter    = 0; 

int           divisor            = 1;             // used to control blink rate of run LED
int           leftPBstate        = LOW;           // previous state of left PB, LOW = true
int           rightPBstate       = LOW;           // previous state of right PB, LOW = true
int           stepFwdPBstate     = LOW;           // previous state of PB, LOW = true
int           stepRevPBstate     = LOW;           // previous state of PB, LOW = true
int           stepRetLSstate     = LOW;           // previous state of LS
int           badCharCounter     = 0;             // Bluetooth bad char counter
int           num1               = 0;             // variable used to pass number of steps to move

boolean       stepperLSfault     = false;
boolean       badCharFlag        = false ;

char inChar = -1; // Where to store the character read
// VARIABLE DECLARATIONS END ************************************************************************************



// SETUP ********************************************************************************************************
void setup() {

  // COMMUNICATIONS
  Serial.begin(9600);
  Serial.println(F("-----------------------------------------"));
  Serial.print(F("AcuiSee -vC Version: "));
  Serial.println(Version);
  Serial.println(F("-----------------------------------------"));

  Bluetooth.begin(115200);            // The Bluetooth Mate defaults to 115200bps
  Bluetooth.print("$");              // Print three times individually to enter command mode
  Bluetooth.print("$");
  Bluetooth.print("$");
  delay(100);                     // Short delay, wait for the BT module to send back CMD
  Bluetooth.println("U,9600,N"); // Change the baudrate to 9600, no parity, for better reliability
  delay(100);
  Bluetooth.begin(9600);      // Start Bluetooth serial at 9600
  // COMMUNICATIONS END

  // stepper
  pinMode(DIR_PIN,  OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  // IO
  pinMode(stepFwdPB,          INPUT_PULLUP); // since PULLUP, LOW is ON
  pinMode(stepRetLS, INPUT_PULLUP); // since PULLUP, LOW is ON
  pinMode(leftPB,             INPUT_PULLUP); // since PULLUP, LOW is ON
  pinMode(rightPB,            INPUT_PULLUP); // since PULLUP, LOW is ON
  pinMode(stepRevPB,          INPUT_PULLUP);

  pinMode(leftPBled,  OUTPUT);
  pinMode(rightPBled, OUTPUT);
  pinMode(runLED,     OUTPUT);

  // Set Initial Values for LEDs
  digitalWrite(leftPBled,  HIGH);
  digitalWrite(rightPBled, LOW);
  digitalWrite(runLED,     LOW);

  // Invert to state flags to force a read
  leftPBstate        = !(digitalRead(leftPB));         
  rightPBstate       = !(digitalRead(rightPB));        
  stepFwdPBstate     = !(digitalRead(stepFwdPB));      
  stepRevPBstate     = !(digitalRead(stepRevPB));      
  stepRetLSstate     = !(stepRetLSstate);      

  // Flash the PB LEDs on startup
  Serial.print("Initializing ");
  for (int count = 1; count <= 10; count++) {
    digitalWrite(leftPBled,  !(digitalRead(leftPBled)));
    digitalWrite(rightPBled, !(digitalRead(rightPBled)));
    digitalWrite(runLED,     !(digitalRead(runLED)));
    delay(500); Serial.print(11 - count); Serial.print(" ");
  }
  // Set Initial Values for LEDs
  digitalWrite(leftPBled,  LOW);
  digitalWrite(rightPBled, LOW);
  digitalWrite(runLED,     LOW);

   // close out setup with some information
   
   Serial.print((F("\nmaxSteps    = "))); Serial.println(maxSteps);
   
   Serial.print((F("maxTravelTime = "))); Serial.println(maxTravelTime);
   Serial.print((F("stepCounter   = "))); Serial.println(stepCounter);
   Serial.println(F("\n-----------------------------------------"));
   Serial.println(F("RESET - to get everything at known state . . . "));
   
   // run a reset at startup
  delay(1000);
  resetAll();  

  // END Flash the PB LEDs on startup
  // IO END
} // SETUP END ****************************************************************************************************

void loop() {

  // CHECK THE IO *************************************************************************************************
  if ((millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis() ;
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // check  leftPB
    if (digitalRead(leftPB) != leftPBstate) {
      leftPBstate = digitalRead(leftPB);
      if (leftPBstate == LOW) {
        Serial.write("a\n"); Bluetooth.print("a\n");
        digitalWrite(leftPBled,  HIGH); delay(2000); // leave led on for a couple of seconds
      } else {
        Serial.write("b\n"); Bluetooth.print("b\n");
        digitalWrite(leftPBled,  LOW);
      }
    }// End of check leftPB

    // check rightPB
    if (digitalRead(rightPB) != rightPBstate) {
      rightPBstate = digitalRead(rightPB);
      if (rightPBstate == LOW) {
        Serial.write("c\n"); Bluetooth.print("c\n");
        digitalWrite(rightPBled, HIGH); delay(2000); // leave led on for a couple of seconds
      } else {
        Serial.write("d\n"); Bluetooth.print("d\n");
        digitalWrite(rightPBled, LOW);
      }
    } // End of check rightPB

    // check  stepFwdPB
    if (digitalRead(stepFwdPB) != stepFwdPBstate) {
      stepFwdPBstate = digitalRead(stepFwdPB);
      if (stepFwdPBstate == LOW) {
        Serial.write("e\n"); Bluetooth.print("e\n");
      } else {
        Serial.write("f\n"); Bluetooth.print("f\n");
      }
      while ((stepFwdPBstate == LOW) && (digitalRead(stepFwdPB) == LOW)  && (stepCounter < maxSteps)) {
        // rotate( -25, .75);
        rotate(checkSteps(-(25)), .75);
      }
      Serial.print("stepCounter = "); Serial.println(stepCounter);
    } // End of check stepFwdPB

    // check  stepRevPB
    if (digitalRead(stepRevPB) != stepRevPBstate) {
      stepRevPBstate = digitalRead(stepRevPB);
      if (stepRevPBstate == LOW) {
        Serial.write("g\n"); Bluetooth.print("g\n");
      } else {
        Serial.write("h\n"); Bluetooth.print("h\n");
      }
      while ((digitalRead(stepRevPB) == LOW) && (digitalRead(stepRetLS) != LOW)) {
        //rotate( 25, .75);
        rotate(checkSteps((25)), .75);
      }
      Serial.print("stepCounter = "); Serial.println(stepCounter);
    } // End of check stepRevPB

    // check stepper retracted LS
    if (digitalRead(stepRetLS) != stepRetLSstate) {
      stepRetLSstate = digitalRead(stepRetLS);
      if (digitalRead(stepRetLS) == LOW) {
        Serial.write("i\n"); Bluetooth.print("i\n");
        stepperLSfault = false;
        stepCounter        = 0;
      } else {
        Serial.write("j\n"); Bluetooth.print("j\n");
      }
    } // if stepRetLS

    if ((stepCounter >= maxSteps) && (stepCounter != prevstepCounter)) {
      prevstepCounter = stepCounter;
      Serial.write("n\n"); Bluetooth.print("n\n");     
      } else {
      prevstepCounter = stepCounter;
      }

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  }   // End of if millis()
  // CHECK THE IO END *********************************************************************************************

  // RUN LIGHT ****************************************************************************************************
  currentMillis = millis();
  if (currentMillis - previousMillis > interval / divisor) {
    previousMillis = currentMillis;
    digitalWrite(runLED,  !digitalRead(runLED));
  }
  // RUN LIGHT END ***********************************************************************************************

  // SERIAL and BLUETOOTH READ ************************************************************************************
  // Read and process any commands that have arrived via USB
  if (Serial.available()    > 0) readSerialPort(); // serial port
  if (Bluetooth.available() > 0) readBluetooth();  // Bluetooth port
  // execute a BT character retry if necessary
  if (((millis() - badCharTimer) > badCharTimerLimit) && (badCharFlag == true)) {
    // send out the next retry request
    Bluetooth.print("s\n");                                      // send a retry character
    badCharCounter++;
    badCharTimer = millis() ;
    Serial.write("s\n");                                         // debug echo
    Serial.print("badCharCounter2: ");
    Serial.println(badCharCounter);
  }
  // SERIAL and BLUETOOTH READ END ********************************************************************************

} // of void loop()
// LOOP END *******************************************************************************************************
// LOOP END *******************************************************************************************************
// LOOP END *******************************************************************************************************

// ****************************************************************************************************************
// ************************   FUNCTIONS   **   FUNCTIONS  ** FUNCTIONS  **  FUNCTIONS  ***************************
// ****************************************************************************************************************

// processCommand **************************************************************************************************
char processCommand(char ch) {
  switch (ch) {
    case 'S':     // Stats
      stats();
      return 0;
      break;
    case 'R':     // RESET
      resetAll();
      return 0;
      break;
    case 'X':
      reboot();   // does a software reset
      return 0;
      break;
    case'Z':      // NOOP
      Serial.println("z");
      Bluetooth.print("z\n"); // NO-OP
      delay(25);
      return 0;
      break;
    case '1':
      rotate(checkSteps(-(steps_per_ml * 1)), .75);
      Serial.print("stepCounter = "); Serial.println(stepCounter);
      return 0;
      break;
    case '2':
      rotate(checkSteps(-(steps_per_ml * 2)), .75);
      Serial.print("stepCounter = "); Serial.println(stepCounter);
      return 0;
      break;
    case '3':
      rotate(checkSteps(-(steps_per_ml * 3)), .75);
      Serial.print("stepCounter = "); Serial.println(stepCounter);
      return 0;
      break;
    case '4':
      rotate(checkSteps(-(steps_per_ml * 4)), .75);
      Serial.print("stepCounter = "); Serial.println(stepCounter);
      return 0;
      break;
    case '5':
      rotate(checkSteps(-(steps_per_ml * 5)), .75);
      Serial.print("stepCounter = "); Serial.println(stepCounter);
      return 0;
      break;
    case '6':
      rotate(checkSteps(-(steps_per_ml * 6)), .75);
      Serial.print("stepCounter = "); Serial.println(stepCounter);
      return 0;
      break;
    case '7':
      rotate(checkSteps(-(steps_per_ml * 7)), .75);
      Serial.print("stepCounter = "); Serial.println(stepCounter);
      return 0;
      break;
    case '8':
      rotate(checkSteps(-(steps_per_ml * 8)), .75);
      Serial.print("stepCounter = "); Serial.println(stepCounter);
      return 0;
      break;
    case '9':
      rotate(checkSteps(-(steps_per_ml * 9)), .75 );
      Serial.print("stepCounter = "); Serial.println(stepCounter);
      return 0;
      break;
    case '?':
      Serial.print("\n\nAcuiSee-vR Version: "); Serial.println(Version);
      Serial.print("stepCounter = "); Serial.println(stepCounter);
      Bluetooth.print("\n\nAcuiSee-vR Version: "); Bluetooth.print (Version); Bluetooth.print("\n");
      return 0;
      break;
    default:
      Serial.print("skipped ");
      Serial.println(ch);
      return 0;
  } // of switch
} // end of processCommand()
// processCommand END **********************************************************************************************

// readSerialPort  *************************************************************************************************
char readSerialPort(void) {
  while (Serial.available() > 0) { // Don't read unless you know there is data
    inChar = Serial.read(); // Read a character

    // ASCII Decimal Range for A-Z, 0-9, ?, @, CR, LF
    if (((inChar >= 63) && (inChar <= 90))      // ascii A-Z, ?, and @
        || ((inChar >= 48) && (inChar <= 57))   // ascii 0-9
        || (inChar == 10)                       // ascii LF
        || (inChar == 13)) {                    // ascii CR
      if ((inChar != 10) && (inChar != 13)) processCommand(inChar); // process if NOT CR or LF
    } // of if inchar
  } // of While
} // of function readSerialPort
// readSerialPort  *************************************************************************************************

// readBluetooth  **************************************************************************************************
char readBluetooth(void) {
  while (Bluetooth.available() > 0) {
    Serial.print("Bluetooth.available() = ");
    Serial.println(Bluetooth.available());

    inChar = Bluetooth.read();  // Read a character
    Serial.print("Prefilter >>");
    Serial.print(inChar);
    Serial.print(", hex: ");
    Serial.println(inChar, HEX);   // prints value as string in hexadecimal

    // ASCII Decimal Range for A-Z, 0-9, ?, @, CR, LF
    if (((inChar >= 63) && (inChar <= 90))      // ascii A-Z, ?, and @
        || ((inChar >= 48) && (inChar <= 57))   // ascii 0-9
        || (inChar == 10)                       // ascii LF
        || (inChar == 13)) {                    // ascii CR

      // if we make it here, it is a good character that passed the filter tests
      Serial.print("Postfilter >>");
      Serial.print(inChar);
      Serial.print(", hex: ");
      Serial.println(inChar, HEX);                                  // prints value as string in hexadecimal
      if ((inChar != 10) && (inChar != 13)) processCommand(inChar); // process if NOT CR or LF
      badCharCounter = 0;
      badCharFlag = false;
    } else {

      // if we make it here, character failed the filter test, enable retries
      while (Bluetooth.available() > 0) inChar = Bluetooth.read();  // Flush the buffer
      delay(5);                                                     // Delay a slight amount
      Bluetooth.print("s\n");                                       // send a retry character
      badCharCounter++;
      badCharFlag = true;
      badCharTimer = millis() ;
      Serial.write("s\n");                                          // send a retry character
      Serial.print("badCharCounter1: ");
      Serial.println(badCharCounter);
    } // of if inchar
  } // of While
} // of function readBluetooth
// readBluetooth  **************************************************************************************************

// resetAll  *******************************************************************************************************
void resetAll(void) {
  digitalWrite(leftPBled,  LOW);
  digitalWrite(rightPBled, LOW);
  digitalWrite(runLED,     LOW);
  badCharCounter     = 0;
  stepperLSfault     = false;
  badCharFlag        = false ;
  retract();
  //UPStepper();
}
// resetAll  *******************************************************************************************************

// rotate    *******************************************************************************************************
// ROTATE A NUMBER OF STEPS ///////////////////////////////////////////////////////////////////////////
void rotate(int steps, float speed) {
  //rotate a specific number of microsteps (8 microsteps per step) - (negative for reverse movement)
  //speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
  stepCounter = stepCounter - steps;
  int dir = (steps > 0) ? HIGH : LOW;
  steps = abs(steps);
  digitalWrite(DIR_PIN, dir);
  float usDelay = (1 / speed) * delay_time; // increase for less microsteps ie 700 for single step, 70 for 1/8 step
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(usDelay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(usDelay);
  } // for

} // rotate()
// rotate    *******************************************************************************************************

// retract   *******************************************************************************************************
void retract() {
  Serial.write("k\n"); Bluetooth.print("k\n");
  accumulatedTravelTime = millis() ;
  while (digitalRead(stepRetLS) != LOW) {
    //rotate( 25, 1.0);
    rotate(checkSteps(25), 1.0);
    if ((millis() - accumulatedTravelTime) > maxTravelTime) {
      Serial.write("m - ls fault\n"); Bluetooth.print("m\n");
      stepperLSfault = true;
      break;
    }
  } // while
  if (!(stepperLSfault)) {
    stepCounter = 0;
    Serial.write("l\n"); Bluetooth.print("l\n");
    Serial.print("stepCounter = "); Serial.println(stepCounter);
  }
}
// retract   *******************************************************************************************************

// rboot     *******************************************************************************************************
void reboot() // Software restart - Restarts program from beginning but does not reset the peripherals and registers
{
  asm volatile ("  jmp 0");
}
// rboot     *******************************************************************************************************


// checkSteps ******************************************************************************************************
long checkSteps(long checkSteps) {
  if (checkSteps > 0) {
    return (checkSteps); // if positive we're moving towards LS stop, no need to check steps
  } else {
    long stepsRemaining = maxSteps - stepCounter;
    if (stepsRemaining > (abs(checkSteps))) {
    return (checkSteps);        // return value passed
    } else {
      return (-stepsRemaining); // return steps reamaining
    }
  }
} // of checkSteps
// checkSteps ******************************************************************************************************

// stats    ********************************************************************************************************
void stats() {
  if (leftPBstate == LOW) {
        Serial.write("a\n"); Bluetooth.print("a\n");
      } else {
        Serial.write("b\n"); Bluetooth.print("b\n");
      }
  
  if (rightPBstate == LOW) {
        Serial.write("c\n"); Bluetooth.print("c\n");
      } else {
        Serial.write("d\n"); Bluetooth.print("d\n");
      }    
  
  if (stepFwdPBstate == LOW) {
        Serial.write("e\n"); Bluetooth.print("e\n");
      } else {
        Serial.write("f\n"); Bluetooth.print("f\n");
      }    

  if (stepRevPBstate == LOW) {
        Serial.write("g\n"); Bluetooth.print("g\n");
      } else {
        Serial.write("h\n"); Bluetooth.print("h\n");
      }  

   if (digitalRead(stepRetLS) == LOW) {
        Serial.write("i\n"); Bluetooth.print("i\n");
        stepperLSfault = false;
        stepCounter        = 0;
      } else {
        Serial.write("j\n"); Bluetooth.print("j\n");    
      }
   if (stepCounter >= maxSteps) {
      Serial.write("n\n"); Bluetooth.print("n\n");     
      }   
      Serial.print("stepCounter = "); Serial.println(stepCounter);     
} // stats()
// stats    ********************************************************************************************************
