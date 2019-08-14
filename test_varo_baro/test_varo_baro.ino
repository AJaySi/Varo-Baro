//Include all our required libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// BMP280 I2C address is 0x76(108) or change accordingly.
#define Addr 0x76

// Refer: https://gliding.clementallen.com/glide/
const float glide_ratio = 8;
float glide_distance = 0;

// ABORT Flag: Whenever a defined failure is met, set flag to true.
boolean abort_flag = false;
// Altimeter States
String alti_state = "GROUNDED";
float sea_level_pressure;

// NOTE: RGB LEDs will use the same pins.
const int ledGreen = 3;
const int ledRed = 5;
const int ledBlue = 6;
// NOTE: For RGB, set to true if common anode Or false if common cathode
const int buzzer = 9; //buzzer to arduino pin 9

/*
  The setup function runs once when you press reset or power the board.
  TBD: Need a push button to power on/off the vario device.
  Things to do in setup:
  1). Scan and check if barometer sensor is found/working.
  2). Put blinker and toner in start_mode
  3). Check all works and initalise the sensors.
  */
Adafruit_BMP280 bmp; // I2C
void setup()
{
  // Sets up communication with the serial port
  Serial.begin(9600);
  
  // FIXME: Sea level pressure changes everyday and needs to accurate
  // for altimeter.
  sea_level_pressure = 1003.25;

  //Debug:
  Serial.println(F("WARN: Expecting RGB LEDs on pins: "));
  Serial.print(ledRed);
  Serial.print(ledGreen);
  Serial.print(ledGreen);

  // declare LED pins as Output
  pinMode(ledGreen, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(ledBlue, OUTPUT);

  // Defines the Buzzer pin as output
  pinMode(buzzer, OUTPUT);
  Serial.println(F("WARN: Expecting buzzer on Pin: "));
  Serial.print(buzzer);

  // FIXME: Basic check for LEDs, check states and color spectrum.
  // TBD: Make a common function 'test' and pass flags as test("BMP280")
  // Check if the sound/speaker is working, by playing a tone.
  // FIXME: Check by toggling HIGH/LOW and verifying the states.
  Serial.print(F("Testing Vario Sound and Light"));
  test_sound_light();

  // Scan for sensors or fail/ABORT with alert
  // TBD: Pass flags for specific sensors to check if its working. eg: bmp280
  Serial.println(F("Scanning & Testing sensors"));
  scan_sensors();
  // Run sensor tests/calibrations for warmups.
  test_sensors();

  /* Set Default settings from BMP280 datasheet. 
      This will require change based on what is best for battery usage, flight time.*/
  Serial.println(F("BMP Operating Mode: NORMAL"));
  Serial.println(F("STANDBY time is 500ms"));

  // 8x pressure over sampling and 1x temp, 
  //Running in forced mode & reading every 20ms, 50Hz. 
  //Calibration Settings
    //bmp.setPressureOversampleRatio(2);
    //bmp.setTemperatureOversampleRatio(1);
    //bmp.setFilterRatio(4);
    //bmp.setStandby(0);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,  /* Operating Mode. */
          Adafruit_BMP280::SAMPLING_X2,  /* Temp. oversampling */
          Adafruit_BMP280::SAMPLING_X16,  /* Pressure oversampling */
          Adafruit_BMP280::FILTER_X16,    /* Filtering. */
          Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  // TBD: Need tests for battery usage percentage and flash Magenta if LOW
  Serial.println(F("TBD: Test Battery usage percentage"));
  // https://github.com/rlogiacco/BatterySense
  //test_battery();

  Serial.println(F("INFO: Setup complete"));
}

// Loop main function
void loop()
{
  // Just, test and Exit, No Loop.
}


// Needed for velocity calculations.
float get_altitude() 
{
  // #include "i2c_BMP280.h"
  float alt_meters;
  bmp.getTemperature(alt_meters);  // throw away - needed for alt.
  bmp.getPressure(alt_meters);     // throw away - needed for alt.
  bmp.getAltitude(alt_meters);
  return alt_meters;
}


// Common function for all things to do with variometer lighting/blinking etc
// Function accepts color mode dependent on altimeter & Vario states.
void light()
{
  // CHeck if Altimeter mode is GROUNDED
  if (alti_state.equalsIgnoreCase("GROUNDED"))
  {
    RGB_color(255, 0, 255); // Magenta
    // Feature: When grounded/ para waiting then reduce LED brightness to
    // save battery.
    lightness(); 
  }
  // Check if altimeter mode is SINK
  else if (alti_state.equals("SINK"))
  {
    RGB_color(255, 0, 0); // Red
    lightness();
  }
  // Check if altimeter mode is LIFT
  else if (alti_state.equals("LIFT"))
  {
    RGB_color(0, 255, 0); // Green
    lightness();
  }

  // TBD: Assign below colors for other vario functions.
  //RGB_color(0, 0, 255); // Blue
  //RGB_color(255, 255, 125); // Raspberry
  //RGB_color(0, 255, 255); // Cyan
  //RGB_color(255, 255, 0); // Yellow
  //RGB_color(255, 255, 255); // White
}


// Light Helper function
void RGB_color(int red_light_value, int green_light_value, int blue_light_value)
{
  analogWrite(ledRed, red_light_value);
  analogWrite(ledGreen, green_light_value);
  analogWrite(ledBlue, blue_light_value);
}


// Function to vary the brightness of LED on conditions/flags.
void lightness()
{
  // Brightness is maximum value of 255/HIGH/5V
  // Write brightness. analogWrite() uses PWM to send varying voltages.
  if (alti_state.equalsIgnoreCase("GROUNDED"))
  {
    analogWrite(ledGreen, 100);
    analogWrite(ledRed, 100);
    analogWrite(ledBlue, 100);
  }
  else if (alti_state.equalsIgnoreCase("SINK"))
  {
    // Set it high, for now.
    analogWrite(ledGreen, 200);
    analogWrite(ledRed, 200);
    analogWrite(ledBlue, 200);
  }
  else if (alti_state.equalsIgnoreCase("LIFT"))
  {
    analogWrite(ledGreen, 255);
    analogWrite(ledRed, 255);
    analogWrite(ledBlue, 255);
  } 
}


// Common function for sound/buzzer/beep. TBD.
void sound()
{
  if (alti_state.equalsIgnoreCase("GROUNDED"))
  {
    // Save battery when GROUNDED or gliding at constant height.
    noTone(buzzer);
  }
  else if (alti_state.equalsIgnoreCase("SINK"))
  {
    // Lower frequency for SINK
    tone(buzzer, 300);
  }
  else if (alti_state.equalsIgnoreCase("LIFT"))
  {
    // Higher frequency for LIFT
    tone(buzzer, 700);
  }
}


// Helper function to switch ON a given pin
void On(int pin, int duration)
{
  digitalWrite(pin, HIGH);
  delay(duration);
}

// Helper function to switch OFF a given pin for that duration.
void Off(int pin, int duration)
{
  digitalWrite(pin, LOW);
  delay(duration);
}

//I2C Scanner code. To test if scanner is connected/found.
void scan_sensors()
{
  // TBD: baud rate : 115200
  Serial.println();
  Serial.println(F("I2C scanner. Scanning ..."));
  Serial.print(F("Scanning address range 0x03-0x77\n\n"));
  byte count = 0;

  Wire.begin();
  for (byte i = 1; i < 120; i++)
  {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0)
    {
      Serial.print(F("Found used address: "));
      Serial.print(i, DEC);
      Serial.print(" (0x");
      Serial.print(i, HEX);
      Serial.println("");
      count++;
    }
    delay(500);
  }

  //Print scanning result.
  Serial.println(F("Scanning Complete."));
  Serial.print(F("Found following"));
  Serial.print(count, DEC);
  Serial.println(F(" device(s)."));
}

// Function to test variometer light and sound, working OK.
void test_sound_light()
{

  Serial.println(F("Testing Vario LED functions.."));
  light();
  delay(500);

  Serial.println(F("Testing Vario Light functions.."));
  sound();

  // FIXME: False positive.
  Serial.println(F("Success: Vario Sound and Light OK."));
}

// Function to run tests on the variometer sensors from initial/ON state.
// TBD: This may not be necessary in production.
void test_sensors()
{
  int ten = 0;
  float forecast = 1013.1;
  float approx = 0;
  float alti = 0;
  float store = 0;
  float start = 0;

  Serial.println(F("Setup for elevator floor change measurement."));
  
  // Check and wait till BMP280 is found and initalised.
  // This is a required sensor and other functionalities are dependent
  // on this sensor.
  if (!bmp.begin())
  {
    Serial.println(F("ERROR: No BMP280 sensor Found, waiting for connection.."));
    while (1)
      ;
  }
  else Serial.println(F("BMP280 init Success!"));

  Serial.println(F("\nINFO: Variometer BMP280 Testing..\n"));
  bmp.setPressureOversampleRatio(2);
    bmp.setTemperatureOversampleRatio(1);
    bmp.setFilterRatio(0);
    bmp.setStandby(0);

    // onetime-measure:
    bmp.setEnabled(0);
    bmp.triggerMeasurement()

  ten = 0;
  store = 0;
  start = 0;
  for (int s = 1; s <= 10; s++)
  {
    store = bmp.readAltitude(forecast);
    ten += store;
    if (s = 1)
    {
      ten = ten - store;
      s++;
    }

    Serial.println(ten);
    Serial.println(store);
    Serial.println(start);
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(store);
    Serial.println(" m");
    Serial.println(F("INFO: Calibrating sensor..."));
    store = 0;
    delay(500);
  }
  Serial.println(F("\nCalibration Complete.\n"));
  Serial.print(F("Current Altitude:  "));
  start = ten / 9;
  //start = ;
  Serial.print(start);
  Serial.println(F(" m \n\n"));
  delay(1000);
}
