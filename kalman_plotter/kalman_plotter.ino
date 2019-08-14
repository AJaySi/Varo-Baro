#include <SimpleKalmanFilter.h>
#include <Adafruit_BMP280.h>

/*
  This sample code demonstrates how the SimpleKalmanFilter object can be used with a
  pressure sensor to smooth a set of altitude measurements.
  This example needs a BMP180 barometric sensor as a data source.
  https://www.sparkfun.com/products/11824
  SimpleKalmanFilter(e_mea, e_est, q);
  e_mea: Measurement Uncertainty
  e_est: Estimation Uncertainty
  q: Process Noise
*/
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);
SimpleKalmanFilter tempKalmanFilter(1, 1, 0.01);
SimpleKalmanFilter altiKalmanFilter(1, 1, 0.01);
Adafruit_BMP280 bmp; // I2C

void setup() {

  Serial.begin(9600);

  // BMP180 Pressure sensor start
  if (!bmp.begin()) {
    Serial.println("BMP180 Pressure Sensor Error!");
    while (1); // Pause forever.
  }
}

void loop() {
  float altitude = bmp.readAltitude();
  float kalman_altitude = pressureKalmanFilter.updateEstimate(altitude);

  float temperature = bmp.readTemperature();
  float kalman_temp = tempKalmanFilter.updateEstimate(temperature);

  Serial.print(altitude);
  Serial.print("\t");
  Serial.println(kalman_altitude); 
  Serial.print(altitude, 6);
  Serial.print(" ");
  Serial.println(kalman_altitude, 6);
  delay(500);
}
