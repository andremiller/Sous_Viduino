//-------------------------------------------------------------------
//
// Sous Vide Controller by Andre Miller
// https://github.com/andremiller/Sous_Viduino
// Based on Sous Vide Controller by Bill Earl from Adafruit Industries
// https://github.com/adafruit/Sous_Viduino
// Based on the Arduino PID and PID AutoTune Libraries
// by Brett Beauregard
//------------------------------------------------------------------

// PID Library
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Libraries for the DS18B20 Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// So we can save and retrieve settings
#include <EEPROM.h>

// ************************************************
// Pin definitions
// ************************************************
//------------------------------------------------------------------
// Pins Used:
// Analog 0                 DFRobot LCD Shield Keypad
// Digital 8, 9, 4, 5, 6, 7 DFRobot LCD Shield LCD
// Digital 2, 3, 11         DS18B20 Temperature Sensor (2 = Bus, 3 = VCC, 11 = GND)
// Digital 12, 13           Output control relay (12 = GND, 13 = RELAY)
//------------------------------------------------------------------

// Output Relay
// (Use GPIO pins for ground to simplify the wiring)
#define RelayPin    13
#define RelayPinGnd 12

// One-Wire Temperature Sensor
// (Use GPIO pins for power/ground to simplify the wiring)
#define ONE_WIRE_BUS 2
#define ONE_WIRE_PWR 3
#define ONE_WIRE_GND 11

// Swap these around if your relay is active low
#define RELAY_OFF LOW
#define RELAY_ON  HIGH

//#define KEYPAD_V11  // There are two different versions of the DFRobot keypad
                    // If the buttons don't work, try commenting / uncommenting this

// ************************************************
// PID Variables and constants
// ************************************************

//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;

volatile long onTime = 0;

// pid tuning parameters
double Kp;
double Ki;
double Kd;

// EEPROM addresses for persisted data
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// 10 second Time Proportional Output window
int WindowSize = 10000;
unsigned long windowStartTime;

// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember = 2;

double aTuneStep = 500;
double aTuneNoise = 1;
unsigned int aTuneLookBack = 20;

boolean tuning = false;

PID_ATune aTune(&Input, &Output);

// ************************************************
// Display Variables and constants
// ************************************************

// Libraries for LCD
#include <LiquidCrystal.h>
// Initalize the LCD shield
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

unsigned long lastInput = 0; // last button press

byte degree[8] = // define the degree symbol
{
  B00110,
  B01001,
  B01001,
  B00110,
  B00000,
  B00000,
  B00000,
  B00000
};

#define BUTTON_RIGHT 1
#define BUTTON_UP 2
#define BUTTON_DOWN 4
#define BUTTON_LEFT 8
#define BUTTON_SELECT 16

const int logInterval = 10000; // log every 10 seconds
long lastLogTime = 0;

// ************************************************
// States for state machine
// ************************************************
enum operatingState { OFF = 0, SETP, RUN, TUNE_P, TUNE_I, TUNE_D, AUTO};
operatingState opState = RUN;
bool relayState = false;

// ************************************************
// Sensor Variables and constants
// Data wire is plugged into port 2 on the Arduino

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempSensor;

// ************************************************
// Setup and diSplay initial screen
// ************************************************
void setup()
{
  Serial.begin(9600);

  // Initialize Relay Control:
  pinMode(RelayPin, OUTPUT);
  pinMode(RelayPinGnd, OUTPUT);
  digitalWrite(RelayPin, RELAY_OFF); relayState=false; // make sure it is off to start
  digitalWrite(RelayPinGnd, LOW);

  // Set up Ground & Power for the sensor from GPIO pins
  pinMode(ONE_WIRE_GND, OUTPUT);
  digitalWrite(ONE_WIRE_GND, LOW);
  pinMode(ONE_WIRE_PWR, OUTPUT);
  digitalWrite(ONE_WIRE_PWR, HIGH);

  // Initialize LCD DiSplay

  lcd.begin(16, 2);
  lcd.createChar(1, degree); // create degree symbol from the binary
  lcd.setCursor(0, 0);
  lcd.print(F("    Arduino  "));
  lcd.setCursor(0, 1);
  lcd.print(F("   Sous Vide!"));
  delay(500);

  // Start up the DS18B20 One Wire Temperature Sensor

  sensors.begin();
  while (!sensors.getAddress(tempSensor, 0))
  {
    lcd.setCursor(0, 1);
    lcd.print(F("Sensor Error"));
    delay(2000);
  }
  sensors.setResolution(tempSensor, 12);
  sensors.setWaitForConversion(false);

  // Initialize the PID and related variables
  LoadParameters();
  myPID.SetTunings(Kp, Ki, Kd);

  myPID.SetSampleTime(1000);
  myPID.SetOutputLimits(0, WindowSize);

  // Run timer2 interrupt every 15 ms
  TCCR2A = 0;
  TCCR2B = 1 << CS22 | 1 << CS21 | 1 << CS20;

  //Timer2 Overflow Interrupt Enable
  TIMSK2 |= 1 << TOIE2;

  // Start in Run state
  // Prepare to transition to the RUN state
  sensors.requestTemperatures(); // Start an asynchronous temperature reading

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  windowStartTime = millis();
  opState = RUN; // start control
}

// ************************************************
// Timer Interrupt Handler
// ************************************************
SIGNAL(TIMER2_OVF_vect)
{
  if (opState == OFF)
  {
    digitalWrite(RelayPin, RELAY_OFF); relayState=false; // make sure relay is off
  }
  else
  {
    DriveOutput();
  }
}

// ************************************************
// Main Control Loop
//
// All state changes pass through here
// ************************************************
void loop()
{
  // wait for button release before changing state
  while (ReadButtons() != 0) {}

  lcd.clear();

  switch (opState)
  {
    case OFF:
      Off();
      break;
    case SETP:
      Tune_Sp();
      break;
    case RUN:
      Run();
      break;
    case TUNE_P:
      TuneP();
      break;
    case TUNE_I:
      TuneI();
      break;
    case TUNE_D:
      TuneD();
      break;
  }
}

// ************************************************
// Initial State - press RIGHT to enter setpoint
// ************************************************
void Off()
{
  myPID.SetMode(MANUAL);
  digitalWrite(RelayPin, RELAY_OFF); relayState=false; // make sure it is off
  lcd.print(F("    OFF      "));
  lcd.setCursor(0, 1);
  lcd.print(F("   Sous Vide!"));
  uint8_t buttons = 0;

  while (!(buttons & (BUTTON_RIGHT)))
  {
    buttons = ReadButtons();
  }
  // Prepare to transition to the RUN state
  sensors.requestTemperatures(); // Start an asynchronous temperature reading

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  windowStartTime = millis();
  opState = RUN; // start control
}

// ************************************************
// Setpoint Entry State
// UP/DOWN to change setpoint
// RIGHT for tuning parameters
// LEFT for OFF
// SELECT for 10x tuning
// ************************************************
void Tune_Sp()
{
  lcd.print(F("Set Temperature:"));
  uint8_t buttons = 0;
  float increment = 0.1;
  while (true)
  {
    buttons = ReadButtons();
    if (buttons & BUTTON_SELECT)
    {
      if (increment == 0.1) {
        increment = 1.0;
      } else if (increment == 1.0) {
        increment = 0.1;
      }
      Serial.println(increment);
      delay(200);
    }
    if (buttons & BUTTON_LEFT)
    {
      opState = RUN;
      return;
    }
    if (buttons & BUTTON_RIGHT)
    {
      opState = TUNE_P;
      return;
    }
    if (buttons & BUTTON_UP)
    {
      Setpoint += increment;
      delay(200);
    }
    if (buttons & BUTTON_DOWN)
    {
      Setpoint -= increment;
      delay(200);
    }

    if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
    {
      opState = RUN;
      return;
    }
    lcd.setCursor(0, 1);
    lcd.print(Setpoint);
    lcd.print(" ");
    DoControl();
  }
}

// ************************************************
// Proportional Tuning State
// UP/DOWN to change Kp
// RIGHT for Ki
// LEFT for setpoint
// SELECT for 10x tuning
// ************************************************
void TuneP()
{
  lcd.print(F("Set Kp"));
  uint8_t buttons = 0;
  float increment = 1.0;
  while (true)
  {
    buttons = ReadButtons();
    if (buttons & BUTTON_SELECT)
    {
      if (increment == 1.0) {
        increment = 10.0;
      } else if (increment == 10.0) {
        increment = 1.0;
      }
      Serial.println(increment);
      delay(200);
    }
    if (buttons & BUTTON_LEFT)
    {
      opState = SETP;
      return;
    }
    if (buttons & BUTTON_RIGHT)
    {
      opState = TUNE_I;
      return;
    }
    if (buttons & BUTTON_UP)
    {
      Kp += increment;
      delay(200);
    }
    if (buttons & BUTTON_DOWN)
    {
      Kp -= increment;
      delay(200);
    }
    if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
    {
      opState = RUN;
      return;
    }
    lcd.setCursor(0, 1);
    lcd.print(Kp);
    lcd.print(" ");
    DoControl();
  }
}

// ************************************************
// Integral Tuning State
// UP/DOWN to change Ki
// RIGHT for Kd
// LEFT for Kp
// SELECT for 10x tuning
// ************************************************
void TuneI()
{
  lcd.print(F("Set Ki"));

  uint8_t buttons = 0;
  float increment = 0.01;
  while (true)
  {
    buttons = ReadButtons();
    if (buttons & BUTTON_SELECT)
    {
      if (increment == 0.01) {
        increment = 0.1;
      } else if (increment == 0.1) {
        increment = 1.0;
      } else if (increment == 1.0) {
        increment = 0.01;
      }
      delay(200);
    }
    if (buttons & BUTTON_LEFT)
    {
      opState = TUNE_P;
      return;
    }
    if (buttons & BUTTON_RIGHT)
    {
      opState = TUNE_D;
      return;
    }
    if (buttons & BUTTON_UP)
    {
      Ki += increment;
      delay(200);
    }
    if (buttons & BUTTON_DOWN)
    {
      Ki -= increment;
      delay(200);
    }
    if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
    {
      opState = RUN;
      return;
    }
    lcd.setCursor(0, 1);
    lcd.print(Ki);
    lcd.print(" ");
    DoControl();
  }
}

// ************************************************
// Derivative Tuning State
// UP/DOWN to change Kd
// RIGHT for setpoint
// LEFT for Ki
// SELECT for 10x tuning
// ************************************************
void TuneD()
{
  lcd.print(F("Set Kd"));
  uint8_t buttons = 0;
  float increment = 0.01;
  while (true)
  {
    buttons = ReadButtons();
    if (buttons & BUTTON_SELECT)
    {
      if (increment == 0.01) {
        increment = 0.1;
      } else if (increment == 0.1) {
        increment = 1.0;
      } else if (increment == 1.0) {
        increment = 0.01;
      }
      delay(200);
    }
    if (buttons & BUTTON_LEFT)
    {
      opState = TUNE_I;
      return;
    }
    if (buttons & BUTTON_RIGHT)
    {
      opState = RUN;
      return;
    }
    if (buttons & BUTTON_UP)
    {
      Kd += increment;
      delay(200);
    }
    if (buttons & BUTTON_DOWN)
    {
      Kd -= increment;
      delay(200);
    }
    if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
    {
      opState = RUN;
      return;
    }
    lcd.setCursor(0, 1);
    lcd.print(Kd);
    lcd.print(" ");
    DoControl();
  }
}

// ************************************************
// PID Control State
// UP for autotune
// RIGHT - Setpoint
// LEFT - OFF
// ************************************************
void Run()
{
  // set up the LCD's number of rows and columns:
  lcd.print(F("Sp: "));
  lcd.print(Setpoint);
  lcd.write(1);
  lcd.print(F("C : "));

  SaveParameters();
  myPID.SetTunings(Kp, Ki, Kd);

  uint8_t buttons = 0;
  while (true)
  {

    buttons = ReadButtons();
    if ((buttons & BUTTON_UP)
        && (abs(Input - Setpoint) < 0.5))  // Should be at steady-state
    {
      StartAutoTune();
    }
    else if (buttons & BUTTON_RIGHT)
    {
      opState = SETP;
      return;
    }
    else if (buttons & BUTTON_LEFT)
    {
      opState = OFF;
      return;
    }

    DoControl();

    lcd.setCursor(0, 1);
    lcd.print(Input);
    lcd.write(1);
    lcd.print(F("C   "));

    float pct = map(Output, 0, WindowSize, 0, 1000);
    lcd.setCursor(10, 1);
    lcd.print(F("      "));
    lcd.setCursor(10, 1);
    lcd.print(pct / 10);
    //lcd.print(Output);
    lcd.print("%");

    lcd.setCursor(14, 0);
    if (tuning)
    {
      lcd.print("T");
    }
    else
    {
      lcd.print(".");
    }
    if (relayState) {
      lcd.print("#");
    }
    else
    {
      lcd.print(".");
    }

    // periodically log to serial port in csv format
    if (millis() - lastLogTime > logInterval)
    {
      Serial.print(Input);
      Serial.print(",");
      Serial.println(Output);
    }

    delay(100);
  }
}

// ************************************************
// Execute the control loop
// ************************************************
void DoControl()
{
  // Read the input:
  if (sensors.isConversionAvailable(0))
  {
    Input = sensors.getTempC(tempSensor);
    sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
  }

  if (tuning) // run the auto-tuner
  {
    if (aTune.Runtime()) // returns 'true' when done
    {
      FinishAutoTune();
    }
  }
  else // Execute control algorithm
  {
    myPID.Compute();
  }

  // Time Proportional relay state is updated regularly via timer interrupt.
  onTime = Output;
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if (now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if ((onTime > 100) && (onTime > (now - windowStartTime)))
  {
    digitalWrite(RelayPin, RELAY_ON); relayState=true;
  }
  else
  {
    digitalWrite(RelayPin, RELAY_OFF); relayState=false;
  }
}

// ************************************************
// Start the Auto-Tuning cycle
// ************************************************

void StartAutoTune()
{
  // REmember the mode we were in
  ATuneModeRemember = myPID.GetMode();

  // set up the auto-tune parameters
  aTune.SetNoiseBand(aTuneNoise);
  aTune.SetOutputStep(aTuneStep);
  aTune.SetLookbackSec((int)aTuneLookBack);
  tuning = true;
}

// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune()
{
  tuning = false;

  // Extract the auto-tune calculated parameters
  Kp = aTune.GetKp();
  Ki = aTune.GetKi();
  Kd = aTune.GetKd();

  // Re-tune the PID and revert to normal control mode
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetMode(ATuneModeRemember);

  // Persist any changed parameters to EEPROM
  SaveParameters();
}

// ************************************************
// Check buttons and time-stamp the last press
// ************************************************
uint8_t ReadButtons()
{
  uint8_t buttons = 0;

  // Based on example from https://www.dfrobot.com/wiki/index.php/Arduino_LCD_KeyPad_Shield_(SKU:_DFR0009)
  int adc_key_in  = 0;
  // read the buttons
  adc_key_in = analogRead(0);      // read the value from the sensor
  if (adc_key_in > 1000) return 0; // We make this the 1st option for speed reasons since it will be the most likely result
  // There is a button press, timestamp it
  lastInput = millis();

  // For V1.1 use this threshold
#ifdef KEYPAD_V11
  if (adc_key_in < 50)   return BUTTON_RIGHT;
  if (adc_key_in < 250)  return BUTTON_UP;
  if (adc_key_in < 450)  return BUTTON_DOWN;
  if (adc_key_in < 650)  return BUTTON_LEFT;
  if (adc_key_in < 850)  return BUTTON_SELECT;
#else
  // For V1.0 use this threshold
  if (adc_key_in < 50)   return BUTTON_RIGHT;
  if (adc_key_in < 195)  return BUTTON_UP;
  if (adc_key_in < 380)  return BUTTON_DOWN;
  if (adc_key_in < 555)  return BUTTON_LEFT;
  if (adc_key_in < 790)  return BUTTON_SELECT;
#endif

  return 0;  // when all others fail, return this...
}

// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
  if (Setpoint != EEPROM_readDouble(SpAddress))
  {
    EEPROM_writeDouble(SpAddress, Setpoint);
  }
  if (Kp != EEPROM_readDouble(KpAddress))
  {
    EEPROM_writeDouble(KpAddress, Kp);
  }
  if (Ki != EEPROM_readDouble(KiAddress))
  {
    EEPROM_writeDouble(KiAddress, Ki);
  }
  if (Kd != EEPROM_readDouble(KdAddress))
  {
    EEPROM_writeDouble(KdAddress, Kd);
  }
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
  Setpoint = EEPROM_readDouble(SpAddress);
  Kp = EEPROM_readDouble(KpAddress);
  Ki = EEPROM_readDouble(KiAddress);
  Kd = EEPROM_readDouble(KdAddress);

  // Use defaults if EEPROM values are invalid
  if (isnan(Setpoint))
  {
    Setpoint = 60;
  }
  if (isnan(Kp))
  {
    Kp = 850;
  }
  if (isnan(Ki))
  {
    Ki = 0.5;
  }
  if (isnan(Kd))
  {
    Kd = 0.1;
  }
}


// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  {
    EEPROM.write(address++, *p++);
  }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
  double value = 0.0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  {
    *p++ = EEPROM.read(address++);
  }
  return value;
}

