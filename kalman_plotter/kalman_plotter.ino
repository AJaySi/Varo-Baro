#include <SimpleKalmanFilter.h>
#include <Adafruit_BMP280.h>

/*
  This sample code demonstrates how the SimpleKalmanFilter object can be used with a
  pressure sensor to smooth a set of altitude measurements.
  This example needs a BMP280 barometric sensor as a data source.
  https://www.sparkfun.com/products/11824
  SimpleKalmanFilter(e_mea, e_est, q);
  e_mea: Measurement Uncertainty
  e_est: Estimation Uncertainty
  q: Process Noise
*/
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);
SimpleKalmanFilter tempKalmanFilter(1, 1, 0.01);
Adafruit_BMP280 bmp; // I2C

void setup() {

  Serial.begin(9600);

  // BMP280 Pressure sensor start
  if (!bmp.begin()) {
    Serial.println("BMP280 Pressure Sensor Error!");
    while (1); // Pause forever.
  }
}

void loop() {
  // Read raw altitude from the sensor
  float rawAltitude = bmp.readAltitude();
  // Apply Kalman filter to the raw altitude
  float filteredAltitude = pressureKalmanFilter.updateEstimate(rawAltitude);

  // Read raw temperature from the sensor
  float rawTemperature = bmp.readTemperature();
  // Apply Kalman filter to the raw temperature
  float filteredTemperature = tempKalmanFilter.updateEstimate(rawTemperature);

  // Print raw and filtered altitude
  Serial.print("Raw Altitude: ");
  Serial.print(rawAltitude, 6);
  Serial.print("\tFiltered Altitude: ");
  Serial.println(filteredAltitude, 6);

  // Print raw and filtered temperature
  Serial.print("Raw Temperature: ");
  Serial.print(rawTemperature, 6);
  Serial.print("\tFiltered Temperature: ");
  Serial.println(filteredTemperature, 6);

  delay(500);
}
