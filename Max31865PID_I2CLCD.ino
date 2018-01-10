/********************************************************
   PID RelayOutput Example
  Single boiler control, based on Brett Beauregard's PID library and the Adafruit MAX31865 library
 ********************************************************/

#include <Adafruit_MAX31865.h>
#include <PID_v1.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display


// use hardware SPI, just pass in the CS pin
Adafruit_MAX31865 max = Adafruit_MAX31865(52);

// The value of the Rref resistor. Use 430.0!
#define RREF 400.0


#define PIN_INPUT 0
#define RELAY_PIN 6

//PID Variables
double Setpoint, Input, Output;

double BoilerTemp;


//Specify the links and initial tuning parameters
double Kp = 100, Ki = 12.0, Kd = 0.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 1000;
int minWindow = 10; //Should implement a minimal window size for switching SSR. Zerocross detection in Relay
unsigned long windowStartTime;

void setup()
{
  Serial.begin(115200);
  // Serial.println("Espresso Boiler Heat Control");
  pinMode(RELAY_PIN, OUTPUT);

  max.begin(MAX31865_4WIRE);  // set to 2WIRE, 3WIRE or 4WIRE as necessary
  windowStartTime = millis();

  //initialize the variables we're linked to
  Setpoint = 92.;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  lcd.init();                      // initialize the lcd
  lcd.init();
  lcd.backlight();

  //Buttons for on the fly modification of the setpoint
  pinMode(32, INPUT_PULLUP);
  pinMode(30, INPUT_PULLUP);
  pinMode(28, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
}

void loop()
{

  //Stuff to do once a second, calculate PID, display temperatures
  if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window and recalculate
    windowStartTime += WindowSize;
    BoilerTemp = (max.temperature(100, RREF));
    Input = BoilerTemp;
    myPID.Compute();
    DisplayNSerial();
    getKeys(); //very simple, needs debouncing... otherwise only one update per second
  }

  //Activate SSR via N-MOSFET. High=on
  if (Output > millis() - windowStartTime) digitalWrite(RELAY_PIN, HIGH);
  else digitalWrite(RELAY_PIN, LOW);

}

void getKeys()
{
  //Needs debouncing, at the moment only runs once per second, enough for modification of the setpoint in the limited range.
  if (digitalRead(32) == 0)
    Setpoint += 1;
  if (digitalRead(30) == 0)
    Setpoint += 0.1;
  if (digitalRead(28) == 0)
    Setpoint -= 0.1;
  if (digitalRead(26) == 0)
    Setpoint -= 1;

  if (Setpoint > 96.0) //Sensible range for Espresso maker: 85-96 deg C
    Setpoint = 96.0;
  if (Setpoint < 85.0)
    Setpoint = 85.0;

}


void DisplayNSerial()
{
  Serial.print(Setpoint);
  Serial.print(" ");
  Serial.print(BoilerTemp);
  Serial.print(" ");
  Serial.println(Output / 10); //Heater Active Time in percent (as we have a 1000 ms Window)
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Set: ");
  lcd.print(Setpoint);
  lcd.setCursor(0, 1);
  lcd.print("Temp: ");
  lcd.print(BoilerTemp);
  lcd.setCursor(0, 2);
  lcd.print("Output: ");
  lcd.print(Output / 10);
}



