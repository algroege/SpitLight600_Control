#include <TimerThree.h>

#define hv2Pin                5
#define hv1Pin                6
#define delayLinePin          9
#define ledPin                13
#define latchPin              10
#define clockPin              11
#define dataPin               12
#define flSyncInterruptPin    1
#define cameraTriggerPin      20
#define internalDelay         4

#define minPumpDelay          100   //µs
#define defaultPumpDelay      150   //µs
#define maxPumpDelay          225   //µs

#define defaultPulseDivider   20    // 1Hz
#define defaultHVVoltage      0.0   // V

#define maxHvValueRaw            155 //155   // => 4.5 V

//---------------------------------------------------------------------------------------
// Low Level Teensy Parameters - LED and Serial
//---------------------------------------------------------------------------------------
bool vLED = false;
const uint32_t baudRate = 500000;

//---------------------------------------------------------------------------------------
// Serial Communication Parameters 
//---------------------------------------------------------------------------------------
float hvVoltPerDigit = 0.0298;

//---------------------------------------------------------------------------------------
// Timer Functions Parameters 
//---------------------------------------------------------------------------------------
unsigned long prev_Timer = 0; 
unsigned long period_Timer_1000 = 1000;

unsigned long triggerPulseWidth = 50;               // microseconds
long pumpDelayValue = defaultPumpDelay;


volatile bool riseDelayLine = false;
volatile bool singleShot = false;
bool singleShotRequested = false;
volatile int flCounter = 0;
volatile int pulseDivider = defaultPulseDivider;


//---------------------------------------------------------------------------------------
// UART Communication for Laser Control 
//---------------------------------------------------------------------------------------
const byte numChars = 32;

char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing  
char messageFromPC[numChars] = {0}; // variables to hold the parsed data

long singleShotFromPC = 0;
long shotFromPC = 0;
long pumpDelayFromPC = 0;
long pulseDividerFromPC = 0;
long doublePulseDelayFromPC = 0;
float hv1FromPC = 0.0;
float hv2FromPC = 0.0;
boolean newData = false;

//---------------------------------------------------------------------------------------
// Generic Setup Function
//---------------------------------------------------------------------------------------
void setup() { 
  pinMode(ledPin, OUTPUT);
  pinMode(hv1Pin, OUTPUT);
  pinMode(hv2Pin, OUTPUT);
  pinMode(delayLinePin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(flSyncInterruptPin, INPUT);
  pinMode(cameraTriggerPin, OUTPUT);
  
  digitalWrite(ledPin, HIGH);
  digitalWrite(clockPin, LOW);
  digitalWrite(latchPin, LOW);


  attachInterrupt(digitalPinToInterrupt(flSyncInterruptPin), startPumpDelayTimerISR, RISING);
  
  Timer3.initialize(pumpDelayValue);
  Timer3.attachInterrupt(triggerDelayLineISR);
  Timer3.stop();
  
  //start serial port 
  Serial.begin(9600);
  //debugging message for monitor to indicate CPU resets are occuring
  Serial.println("System Reset");

  // Initialize Trigger Box Parameters
  setDoublePulseDelay(100);
  setHVVoltage(hv1Pin, defaultHVVoltage);
  setHVVoltage(hv2Pin, defaultHVVoltage);
}

//---------------------------------------------------------------------------------------
// Main Loop
//---------------------------------------------------------------------------------------
void loop() {
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        applyParsedData();
        newData = false;
    }
}

//---------------------------------------------------------------------------------------
// Flash Lamp Sync Interrupt Service Routine:
// start Timer with 150...215µs Delay
// check Pulse Divider Settings ( 1 -> Pulse for every FL, 2 -> every second FL, ...)
//---------------------------------------------------------------------------------------
void startPumpDelayTimerISR () {
  if (singleShot) {
    if (singleShotRequested) {
      Timer3.setPeriod((pumpDelayValue - internalDelay)*2);         // ---------->>> Lib Error ???? double time
      Timer3.start();
      riseDelayLine = true;
      digitalWrite(ledPin, !digitalRead(ledPin));
      digitalWrite(cameraTriggerPin, LOW);
      singleShotRequested = false;
    }
    
  } else {
    flCounter = flCounter + 1;
    if (flCounter >= pulseDivider) {
      Timer3.setPeriod((pumpDelayValue - internalDelay)*2);         // ---------->>> Lib Error ???? double time
      Timer3.start();
      riseDelayLine = true;
      digitalWrite(ledPin, !digitalRead(ledPin));
      digitalWrite(cameraTriggerPin, LOW);
      flCounter = 0;
    }
  }
  
}

//---------------------------------------------------------------------------------------
// Delay Line Interrupt Service Routine
//     ___
//    |   |      - switches the Trigger Pulse Pin on and off (inverted by Amplifier)
//____|   |___
//---------------------------------------------------------------------------------------
void triggerDelayLineISR () {
  if(riseDelayLine) {
    digitalWrite(delayLinePin, LOW);
    Timer3.setPeriod((triggerPulseWidth - internalDelay)*2);    // ---------->>> Lib Error ???? double time
    Timer3.start();
    riseDelayLine = false;
  } else {
    Timer3.stop();
    digitalWrite(delayLinePin, HIGH);
    digitalWrite(cameraTriggerPin, HIGH);
    //digitalWrite(ledPin, !digitalRead(ledPin));
  }
}

//---------------------------------------------------------------------------------------
// Set Pulse Divider
//---------------------------------------------------------------------------------------
void setSingleShot (long _singleShot, long _shot) {
  if (_singleShot) {
    singleShot = true;
    singleShotRequested = _shot;
  } else {
    singleShot = false;  
  }
}

//---------------------------------------------------------------------------------------
// Set Pulse Divider
//---------------------------------------------------------------------------------------
void setPulseDivider (long _pulseDivider) {
  if (_pulseDivider > 0 && _pulseDivider < 100) {
    pulseDivider = _pulseDivider;
  }
}

//---------------------------------------------------------------------------------------
// Change Pump Delay Value 
// 150µs ... 215µs (max Energy at 215µs)
//---------------------------------------------------------------------------------------
void setPumpDelay (long _pumpDelayValue) {
  if (_pumpDelayValue > minPumpDelay && _pumpDelayValue < maxPumpDelay) {
    pumpDelayValue = _pumpDelayValue;
  }
}

//---------------------------------------------------------------------------------------
// Writes 8-bit value to DS1023S-500 255*5ns
//---------------------------------------------------------------------------------------
void setDoublePulseDelay (long delayBetweenPulses) {
  if (delayBetweenPulses > 255*5) delayBetweenPulses = 255*5;
  setDoublePulseDelayRaw (delayBetweenPulses / 5);
}

void setDoublePulseDelayRaw (byte delayValue) {
  digitalWrite(latchPin, HIGH);
  shiftOut(dataPin, clockPin, MSBFIRST, delayValue);
  digitalWrite(latchPin, LOW);
}

//---------------------------------------------------------------------------------------
// Set Analog Voltages HV1 and HV2
// 128 => 3.80,3.82 
// 255 = 7.58,7.61 
// ==> 0.0298V/digit
//---------------------------------------------------------------------------------------
void setHVVoltage (int hvPin, float voltageValue) {
  setHVVoltageRaw(hvPin, uint8_t(voltageValue/hvVoltPerDigit));
}

void setHVVoltageRaw (int hvPin, uint8_t voltageValue) {
  if (voltageValue > maxHvValueRaw) voltageValue = maxHvValueRaw;       // ==> 4.5V max max 5
  analogWrite(hvPin, voltageValue);
}

//---------------------------------------------------------------------------------------
// UART Communication Functions
// Try to read Data from Input Buffer
//---------------------------------------------------------------------------------------
void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            } else {
                receivedChars[ndx] = '\0';     // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
                Serial.println("Data received");
            }
        } else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//---------------------------------------------------------------------------------------
// UART Communication Functions
// Parse received Data to specific variables
// <(SingleShot),(Shot),(PumpDelay),(PulseDivider),(DoublePulseDelay),(HV1),(HV2)>
// <0,0,215,20,150,4.5,4.5>
// strcpy(messageFromPC, strtokIndx);        // copy it to messageFromPC
//---------------------------------------------------------------------------------------
void parseData() {                            // split the data into its parts
    char * strtokIndx;                        // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");       // get the first part - the string
    singleShotFromPC = atoi(strtokIndx);

    strtokIndx = strtok(NULL,",");       // get the first part - the string
    shotFromPC = atoi(strtokIndx);
 
    strtokIndx = strtok(NULL, ",");           // this continues where the previous call left off
    pumpDelayFromPC = atoi(strtokIndx);         // convert this part to an integer

    strtokIndx = strtok(NULL, ",");
    pulseDividerFromPC = atoi(strtokIndx);          

    strtokIndx = strtok(NULL, ",");
    doublePulseDelayFromPC = atoi(strtokIndx);          

    strtokIndx = strtok(NULL, ",");
    hv1FromPC = atof(strtokIndx);         

    strtokIndx = strtok(NULL, ",");
    hv2FromPC = atof(strtokIndx);                   // convert this part to a float
}

//============

void applyParsedData() {
  // Update parameters
  setSingleShot(singleShotFromPC, shotFromPC);
  setPumpDelay(pumpDelayFromPC);
  setPulseDivider(pulseDividerFromPC);
  setDoublePulseDelay(doublePulseDelayFromPC);
  setHVVoltage(hv1Pin, hv1FromPC);
  setHVVoltage(hv2Pin, hv2FromPC);
}


//---------------------------------------------------------------------------------------
// LED blink 
//---------------------------------------------------------------------------------------
//void timerTask1000 () {
//  if (millis() >= prev_Timer + period_Timer_1000) {
//      prev_Timer = millis();
//    }
//}
