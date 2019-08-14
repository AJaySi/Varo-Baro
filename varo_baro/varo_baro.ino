//Include all our required libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <i2c.h>
#include <i2c_BMP280.h>

/*
  Define all the variables and output/input pin numbers here.
  TBD: Consider moving to a config file.
*/
// BMP280 I2C address is 0x76(108) or change accordingly.
#define addr 0x76
#define BMP280_ADDRESS_ALT (0x76) /**< Alternative I2C address for the sensor. */

// TBD: Mention the paraglider glide ratio. Default is 8:1 as 8
// Knowing current altitude & sink rate, gives the following:
// 1). Maximum flight distance
// 2). Loss of height per km travelled
// 3). Flight duration
// Refer: https://gliding.clementallen.com/glide/
// NOTE: Above theoretical best case values can be improved upon with
// kalman filter, by supplying glide_distance at certain sample rate.
const float glide_ratio = 8;
float glide_distance = 0;

// ABORT Flag: Whenever a defined failure is met, set flag to true.
boolean abort_flag = false;
// Altimeter States
String alti_state = "GROUNDED";
float sea_level_pressure;

/*
  BLINKER
  Version 0.1:
  1). Turn On ledGreen when altitude is increasing (lift).
  2). Turn On ledRed when altitude is decreasing (Sink).
  3). Change the blinker rate with rate of ascent/descent.
  TBD:
  1). Use a single RGB led, instead of two leds.
*/
// Use LED pins that can be used for analog and digital output (3,5,6)
// NOTE: RGB LEDs will use the same pins.
const int ledRed = 13;
const int ledGreen = 12;
const int ledBlue = 11;
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
BMP280 bmp280;

void setup()
{
  // Sets up communication with the serial port
  Serial.begin(9600);

  // FIXME: Sea level pressure changes everyday and needs to accurate
  // for altimeter.
  sea_level_pressure = 1003.25;

  //Debug:
  Serial.println(F("WARN: Expecting RGB LEDs on Digital pins: 13, 12, 11"));

  // declare LED pins as Output
  pinMode(ledGreen, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(ledBlue, OUTPUT);

  // Defines the Buzzer pin as output
  pinMode(buzzer, OUTPUT);
  Serial.println(F("WARN: Expecting buzzer on Pin: 9"));

  // FIXME: Basic check for LEDs, check states and color spectrum.
  // TBD: Make a common function 'test' and pass flags as test("BMP280")
  // Check if the sound/speaker is working, by playing a tone.
  // FIXME: Check by toggling HIGH/LOW and verifying the states.
  Serial.print(F("\nINFO: Testing Vario Sound and Light"));
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

  // Setup for elevator floor change measurement.
  bmp280.setPressureOversampleRatio(2);
  bmp280.setTemperatureOversampleRatio(1);
  bmp280.setFilterRatio(0);
  bmp280.setStandby(0);
  bmp280.setEnabled(0);

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,  /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,  /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,  /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,    /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  Serial.println(("Setting lights in GROUNDED State: Yellow"));
  //RGB_color(255, 255, 0); // Yellow
  RGB_color(255, 0, 0); // Red
  delay(10000);

  // TBD: Need tests for battery usage percentage and flash Magenta if LOW
  Serial.println(F("TBD: Test Battery usage percentage"));
  // https://github.com/rlogiacco/BatterySense
  //test_battery();
  Serial.println(F("INFO: Setup complete"));
}

// Loop main function
void loop()
{
  /*
      ALTIMETER:
      Run the altimeter code in loop. We can put variometer functionalities,
      here in loop. Later on, multiple sensors can be manged in parallel with
      threads like GPS, IR, Camera etc.
      From BMP280, we are only interested in altitude readings.
  */
  Serial.println(F("Calling Vario ALTIMETER with 200ms delay."));
  altimeter();
  // TBD: Delay is bad, it stops all processing. Use millis()
  //delay(200);

  //Calculate glider velocity. Rate of Ascent/Descent.
  //cal_velocity();
  //delay(2000);

  // TBD: Stub for GPS
  //gps_functions();
  // TBD: Stub for Camera
  //camera();
}

// Pressure sensor usage as altimeter.
void altimeter()
{
  /* BMP280: Mode is NORMAL and Oversampling Setting is Standard resolution.
    Oversampling is simply how many times the device reads a sensor and averages out the result
    and then allows you to read it. The more oversampling that you set up the more time it takes
    for a reading to become available.

    The altitude is deduced from the difference of local pressure and
    the sea level pressure. BMP280 measured in a range from 300 hPa to
    1100hPa with ±1.0 hPa accuracy. Calculate the altitude with
    ±1 meter accuracy. The datasheet indicates that you can read
    the device 157 times a second.

    GROUNDED : No Change in altitude Or Flying at constant altitude.
    It depends on the sensor. BMP280 supports minimum of 30cm(?).
    We will then only detect altitude change above 30cm. The logic below,
    stores current altitude and check for +-30cms change in altitude. At which
    point it breaks out of the loop and altitude change variable to true.
  */
  
  boolean altitude_change = false;
  float altitude_current = bmp.readAltitude(sea_level_pressure);
  Serial.println(F("Current Altitude is: "));
  Serial.print(altitude_current);
  Serial.println(" meters");
  float altitude_diff = 0;

  // Loop until change in altitude is detected.
  while (altitude_change == false)
  {
    //bmp.awaitMeasurement();
    Serial.println(F("Waiting for change in altitude.."));
    float altitude_new = bmp.readAltitude(sea_level_pressure);
    
    delay(5000);

    // Check if present altitude reading is same before
    float alti_diff = altitude_current - altitude_new;
    Serial.println(F("Altitude Difference is "));
    Serial.print(alti_diff);
    Serial.println(" meters.");
    
    if (abs(altitude_current - altitude_new) > 0.02)
    {
      // Not equal, hence change in altitude. Set to True.
      altitude_diff = altitude_current - altitude_new;
      Serial.println(F("Change: New Altitude difference is : "));
      Serial.print(altitude_diff);
      altitude_change = true;
    }
    // The vario is GROUNDED : No tone, Magenta color.
    else {
      // Set the altimeter state to GROUNDED
      alti_state = "GROUNDED";
      // For Deubg, with battery, delay will be needed.
      delay(10);

      // No Sound needed, when no change in altitude.
      sound();
      // GROUNDED : Magenta.
      light();
    }
  }

  Serial.println(F("WHILE OUTSIDE"));
  
  // Logic for LIFT and SINK
  if (altitude_diff < 0)
  {
    /*
      LIFT logic: Consider 2 reading in the interval t
      if first altitude reading is less than next, then there is/was LIFT,
      for that duration t.
      Check if altitude_diff is negative, signifying lift.
      TBD: Define Sound and light for LIFT state.
    */
    alti_state = "LIFT";
    Serial.println(F("Vario is experiencing Lift: "));
    // TBD: convert into postive
    Serial.print(altitude_diff);
    // Set sound mode to LIFT
    sound();
    // Set ligt mode to LIFT : GREEN
    light();
  }
  else
  {
    /*
      SINK: If first altitude reading is greater than next reading then,
      we are sinking. Vario is in SINK state.
      TBD: Define light and sound for SINK state.
    */
    alti_state = "SINK";
    Serial.println(F("Vario is experiencing SinK."));
    sound();
    light();
  }

  // wait a fixed amount of time to cover the longest expected delay
  delay(100); //10 readings a second.
}


// Calculate glider velocity.
float cal_velocity()
{
  float distance, velocity;
  static uint32_t time_now, time_was = millis(), dt;
  static float alt_was = get_altitude();
  static uint32_t sample_time_was = millis();
  static uint32_t readings = 0;
  static float values = 0, avg_alt = 0;
  static float meters;

  if ((millis() - sample_time_was) > 2)
  { // read at a specific rate.

    if (!bmp280.awaitMeasurement()) Serial.println("MEASURE FAILED");

    meters = get_altitude();
    readings++;
    values += meters;

    //bmp280.setEnabled(0);
    bmp280.triggerMeasurement();
    sample_time_was = millis();
  }

  time_now = millis();
  dt = time_now - time_was;
  if (dt >= 1000)
  {

    avg_alt = values / readings;
    distance = avg_alt - alt_was;
    velocity = (distance / ((float)dt / 1000.0));
    alt_was = avg_alt;
    time_was = time_now;

    Serial.print("AVG: ");
    Serial.print(avg_alt);
    Serial.print(" m/s ; dist: ");
    Serial.print(distance);

    if (fabs(velocity) > 0.2)
    {
      Serial.print("Velocity: ");
      Serial.print(velocity);
      Serial.print(" m/s ;");
    }
    Serial.print("debug: ");
    Serial.println(readings);

    values = 0;
    readings = 0;
  }
}


// Needed for velocity calculations.
float get_altitude()
{
  // #include "i2c_BMP280.h"
  float alt_meters;
  bmp280.getTemperature(alt_meters);  // throw away - needed for alt.
  bmp280.getPressure(alt_meters);     // throw away - needed for alt.
  bmp280.getAltitude(alt_meters);
  return alt_meters;
}


// Common function for all things to do with variometer lighting/blinking etc
// Function accepts color mode dependent on altimeter & Vario states.
void light()
{
  Serial.println(F("Light: Present Altimeter State: "));
  Serial.print(alti_state);
  
  // CHeck if Altimeter mode is GROUNDED
  if (alti_state.equalsIgnoreCase("GROUNDED"))
  {
    Serial.println(F("GROUNDED: Vario State."));
    RGB_color(255, 0, 255); // Magenta
    // Feature: When grounded/ para waiting then reduce LED brightness to
    // save battery.
    lightness();
  }
  // Check if altimeter mode is SINK
  else if (alti_state.equals("SINK"))
  {
    Serial.println(F("SINK: Vario State."));
    RGB_color(255, 0, 0); // Red
    lightness();
  }
  // Check if altimeter mode is LIFT
  else if (alti_state.equals("LIFT"))
  {
    Serial.println(F("In LIFT State."));
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
    analogWrite(ledGreen, 50);
    analogWrite(ledRed, 50);
    analogWrite(ledBlue, 50);
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
      Serial.println(")");
      count++;
    }
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
  float forecast = 1013.1;
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
  else Serial.println(F("Success: BMP280 Sensor Found!"));
  Serial.println(F("\nINFO: Variometer BMP280 Testing..\n"));


  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(F("Current Altitude:  "));
  Serial.println(bmp.readAltitude());
  Serial.println(F(" m\n"));

  Serial.println(F("Current Velocity is "));
  cal_velocity();

}
