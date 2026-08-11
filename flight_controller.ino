#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <ESP32Servo.h>
#include <SD.h>
#include <sd_defines.h>
#include <sd_diskio.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

SFE_UBLOX_GNSS myGNSS;

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP280 bmp;

float roll, pitch, heading;
float roll_rate, pitch_rate;

float target_roll = 0;
float target_pitch = 0;

Servo myServo1;
Servo myServo2;

static float PT_rho_samples = 100;
static float Pdiff_Repeat = 10;

float kPa_sum = 0;
float rho = 0;
float R = 287.05;
float temp, pressure, lastTime;

float Kp = 0.1;
float Kd = 0;

const char* dataFile = "/Flight_Data.csv";

void setup() {

  Serial.begin(115200);

  myServo1.attach(D2);
  myServo2.attach(D1);

  while (!Serial);

  do {
    Serial.println("GNSS: trying 38400 baud");

    Serial1.begin(38400, SERIAL_8N1, D7, D6);

    if (myGNSS.begin(Serial1) == true) break;

    delay(100);

    Serial.println("GNSS: trying 9600 baud");

    Serial1.begin(9600, SERIAL_8N1, D7, D6);

    if (myGNSS.begin(Serial1) == true) {

      Serial.println("GNSS: connected at 9600 baud, switching to 38400");

      myGNSS.setSerialRate(38400);

      delay(100);

    } else {

      delay(2000);

    }

  } while(1);

  Serial.println("GNSS serial connected");

  // make non blocking
  myGNSS.setUART1Output(COM_TYPE_UBX);
  myGNSS.setI2COutput(COM_TYPE_UBX);
  myGNSS.saveConfiguration();


  if (!bmp.begin()) {

    Serial.println("Failed to initialize bmp sensor!");

    while (1);

  }


  for (int i = 0; i < PT_rho_samples; i++) {

    temp = temp + bmp.readTemperature();

    pressure = pressure + bmp.readPressure();

  }

  temp = temp / PT_rho_samples;

  pressure = pressure / PT_rho_samples;

  rho = (pressure) / ((temp + 273.15) * R);


  if (!bno.begin()) {

    Serial.println("Failed to initialize BNO055!");

    while (1);

  }


  if (!SD.begin(21)) {

    Serial.println("Card Mount Failed");

    while (1);

  }


  File file = SD.open(dataFile, FILE_WRITE);

  if (!file) {

    Serial.println("Failed to create file");

    while(1);

  }

  file.println(
    "Time, Heading, Roll, Pitch,accl x, accl y, accl z, "
    "mag x, mag y,mag z, gyro x, gyro y,gyro z, "
    "latitude, longitude, altitude, airspeed"
  );

  file.close();


  delay(1000);

  bno.setExtCrystalUse(true);

  Serial.println("Setup complete!");

}


void stabalizer_PD() {

  sensors_event_t orientationData, angVelocityData;

  bno.getEvent(
    &orientationData,
    Adafruit_BNO055::VECTOR_EULER
  );

  bno.getEvent(
    &angVelocityData,
    Adafruit_BNO055::VECTOR_GYROSCOPE
  );

  roll = orientationData.orientation.y;

  pitch = orientationData.orientation.z;

  roll_rate = angVelocityData.gyro.y;

  pitch_rate = angVelocityData.gyro.z;

  roll = Kp * (target_roll - roll) + Kd * (roll_rate) + roll;

  pitch = Kp * (target_pitch - pitch) + Kd * (pitch_rate) + pitch;

  float servo_right = pitch - roll;

  float servo_left = pitch + roll;

  float servo_right_pwm = -26.154 * servo_right + 1510;

  float servo_left_pwm = 29.615 * servo_left + 1580;

  if (servo_right_pwm > 2500)
    servo_right_pwm = 2500;

  if (servo_right_pwm < 500)
    servo_right_pwm = 500;

  if (servo_left_pwm > 2200)
    servo_left_pwm = 2200;

  if (servo_left_pwm < 1000)
    servo_left_pwm = 1000;

  myServo1.write(servo_right_pwm);

  myServo2.write(servo_left_pwm);

}


void speed_PD() {

  double Vm = 0;

  double kPa_sum = 0;

  double aR = analogRead(A0);

  for (int i = 0; i < Pdiff_Repeat; i++) {

    aR = analogRead(A0);

    Vm = aR * (3.3 / 4095);

    kPa_sum += (Vm - 0.2) / .45;

  }

  kPa_sum = kPa_sum / Pdiff_Repeat;

  if (kPa_sum < 0) {

    kPa_sum = 0;

  }

  double p = kPa_sum * 1000;

  float airSpeed = sqrt((2 * p) / rho);

}


void loop() {

  stabalizer_PD();


  sensors_event_t orientationData,
                   angVelocityData,
                   magnetometerData,
                   accelerometerData;

  bno.getEvent(
    &magnetometerData,
    Adafruit_BNO055::VECTOR_MAGNETOMETER
  );

  bno.getEvent(
    &accelerometerData,
    Adafruit_BNO055::VECTOR_ACCELEROMETER
  );

  bno.getEvent(
    &angVelocityData,
    Adafruit_BNO055::VECTOR_GYROSCOPE
  );


  unsigned long currentTime = millis();

  double Vm = 0;

  double kPa_sum = 0;

  double aR = analogRead(A0);


  for (int i = 0; i < Pdiff_Repeat; i++) {

    aR = analogRead(A0);

    Vm = aR * (3.3 / 4095);

    kPa_sum += (Vm - 0.2) / .45;

  }

  kPa_sum = kPa_sum / Pdiff_Repeat;

  if (kPa_sum < 0) {

    kPa_sum = 0;

  }

  double p = kPa_sum * 1000;

  float airSpeed = sqrt((2 * p) / rho);


  sensors_event_t eulerEvent;

  bno.getEvent(
    &eulerEvent,
    Adafruit_BNO055::VECTOR_EULER
  );

  float heading = eulerEvent.orientation.x;

  float roll = eulerEvent.orientation.y;

  float pitch = eulerEvent.orientation.z;


  float accl_x = accelerometerData.acceleration.x;

  float accl_y = accelerometerData.acceleration.y;

  float accl_z = accelerometerData.acceleration.z;


  float mag_x = magnetometerData.magnetic.x;

  float mag_y = magnetometerData.magnetic.y;

  float mag_z = magnetometerData.magnetic.z;


  float gyro_x = angVelocityData.gyro.x;

  float gyro_y = angVelocityData.gyro.y;

  float gyro_z = angVelocityData.gyro.z;


  File file = SD.open(dataFile, FILE_APPEND);


  long latitude = myGNSS.getLatitude();

  long longitude = myGNSS.getLongitude();

  long altitude = myGNSS.getAltitude();


  if (file) {

    lastTime = millis();

    file.print(currentTime);
    file.print(",");

    file.print(heading,2);
    file.print(",");

    file.print(roll,2);
    file.print(",");

    file.print(pitch,2);
    file.print(",");

    file.print(accl_x,2);
    file.print(",");

    file.print(accl_y,2);
    file.print(",");

    file.print(accl_z,2);
    file.print(",");

    file.print(mag_x,2);
    file.print(",");

    file.print(mag_y,2);
    file.print(",");

    file.print(mag_z,2);
    file.print(",");

    file.print(gyro_x,2);
    file.print(",");

    file.print(gyro_y,2);
    file.print(",");

    file.print(gyro_z,2);
    file.print(",");


    file.print(latitude,2);
    file.print(",");

    file.print(longitude,2);
    file.print(",");

    file.print(altitude,2);
    file.print(",");


    file.print(airSpeed,2);

    file.print("\n");

    file.close();

  }


  Serial.println("GPS: ");

  Serial.println(longitude,2);

  Serial.println(latitude,2);

  Serial.println(altitude,2);


  if (myGNSS.getFixType() == 0) {

    Serial.println("No GPS fix yet...");

  }


  Serial.println();

  Serial.println("Airspeed: ");

  Serial.println(airSpeed,2);


  delay(100);

}
