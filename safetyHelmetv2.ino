// #include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "Wire.h" // This library allows you to communicate with I2C devices.

#define shockSensorOne A0    // Analog input pin that shock sensor one is attached to
#define shockSensorTwo A1    // Analog input pin that shock sensor two is attached to
#define shockSensorThree A2  // Analog input pin that shock sensor three is attached to
#define soundSensorPin A3    // Analog input pin that the Sound sensor is attached to

#define SHOCK_THRESHOLD 600
#define SOUND_THRESHOLD 40
#define ACCELERATION_THRESHOLD 5

// Choose two Arduino pins to use for software serial
int RXPin = 2;
int TXPin = 3;

//Default baud of NEO-6M is 9600
int GPSBaud = 9600;

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data

char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

SoftwareSerial gpsSerial(RXPin, TXPin); //for GPS

struct SensorValues {
  int soundSensorValue,
  shockSensorOneValue,
  shockSensorTwoValue,
  shockSensorThreeValue,
  accelerometer_x[2],
  accelerometer_y[2],
  accelerometer_z[2];
};

SensorValues sensorValues;

void readSensors() {
  sensorValues.soundSensorValue = analogRead(soundSensorPin);
  sensorValues.shockSensorOneValue = analogRead(shockSensorOne);
  sensorValues.shockSensorTwoValue = analogRead(shockSensorTwo);
  sensorValues.shockSensorThreeValue = analogRead(shockSensorThree);

  // stores previous value in the first position in the array
  sensorValues.accelerometer_x[0] = sensorValues.accelerometer_x[1];
  sensorValues.accelerometer_y[0] = sensorValues.accelerometer_x[1];
  sensorValues.accelerometer_z[0] = sensorValues.accelerometer_x[1];

  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers

  // stores current value in the second position in the array
  sensorValues.accelerometer_x[1] = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  sensorValues.accelerometer_y[1] = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  sensorValues.accelerometer_z[1] = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
}

double magnitude(int x, int y, int z) {
  double magnitude = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
  return magnitude;
}

bool suddenStop() {
  if ((int)magnitude(sensorValues.accelerometer_x[0], sensorValues.accelerometer_y[0], sensorValues.accelerometer_z[0]) / 
      (int)magnitude(sensorValues.accelerometer_x[1], sensorValues.accelerometer_y[1], sensorValues.accelerometer_z[1]) > ACCELERATION_THRESHOLD)
      return true;
      else
      return false;
}


void checkValues() {
  if (sensorValues.soundSensorValue > SOUND_THRESHOLD && 
     (sensorValues.shockSensorOneValue < SHOCK_THRESHOLD ||
     sensorValues.shockSensorTwoValue < SHOCK_THRESHOLD ||
     sensorValues.shockSensorThreeValue < SHOCK_THRESHOLD)) {
    digitalWrite(12, HIGH);  // LED on
    digitalWrite(13, HIGH);  // buzzer on
    delay(2000);             // wait for 2 seconds
    digitalWrite(12, LOW);   // LED off
    digitalWrite(13, LOW);   // buzzer off
    Serial.println("Impact detected");
    Serial.print("GPS Output:");
    Serial.println();
    while (gpsSerial.available() > 0)
    Serial.write(gpsSerial.read());
    Serial.println();
    delay(2000);
  } else {
    digitalWrite(12, LOW);  // LED off
    digitalWrite(13, LOW);  // buzzer off
  }
}

void printValues() {
  Serial.print("Sound sensor:  ");
  Serial.println(sensorValues.soundSensorValue);
  Serial.print("Shock Sensors: ");
  Serial.println(sensorValues.shockSensorOneValue);
  Serial.print("               ");
  Serial.println(sensorValues.shockSensorTwoValue);
  Serial.print("               ");
  Serial.println(sensorValues.shockSensorThreeValue);

  Serial.print("Accelerometer: ");
  Serial.print("aX = "); 
  Serial.println(convert_int16_to_str(sensorValues.accelerometer_x[1]));
  Serial.print("               ");
  Serial.print("aY = "); 
  Serial.println(convert_int16_to_str(sensorValues.accelerometer_y[1]));
  Serial.print("               ");
  Serial.print("aZ = "); 
  Serial.println(convert_int16_to_str(sensorValues.accelerometer_z[1]));

  Serial.println();
}

void setup() {
  pinMode(12, OUTPUT);  //LED
  pinMode(13, OUTPUT);  //buzzer
  Serial.begin(9600);
  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPSBaud);


  // accelerometer setup
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  

  Serial.println("Setup complete");

}

void loop() {
  readSensors();
  checkValues();
  printValues();
  delay(250);
}
