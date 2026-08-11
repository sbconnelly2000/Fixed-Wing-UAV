#include <SD.h>
#include <sd_defines.h>
#include <sd_diskio.h>
#include <Wire.h>
#include <SPI.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BMP280.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP280 bmp;

static float PT_rho_samples = 100;
static float Pdiff_Repeat = 10; 
float kPa_sum = 0;
float rho = 0;
float R = 287.05; 
float temp, pressure, lastTime;


const char* dataFile = "/Lab3.csv";

void setup() {
  Serial.begin(115200);
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
        //myGNSS.factoryReset();
        delay(2000); //Wait a bit before trying again to limit the Serial output
    }
  } while(1);
  Serial.println("GNSS serial connected");

  myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfiguration(); //Save the current settings to flash and BBR

  if (!bmp.begin()) { //Initialize Barometer
    Serial.println("Failed to initialize bmp sensor!");
    while (1);
  }

  for (int i = 0; i < PT_rho_samples; i++)
  {
    temp = temp + bmp.readTemperature();
    pressure = pressure + bmp.readPressure();
  }
  temp = temp/PT_rho_samples;
  pressure = pressure/PT_rho_samples;
  rho = (pressure)/((temp+273.15)*R);

  // Initialize BNO055
  if (!bno.begin()) {
    Serial.println("Failed to initialize BNO055!");
    while (1);
  }

  // Initialize SD card
  if (!SD.begin(21)) {
    Serial.println("Card Mount Failed");
    while (1);
  }

  File file = SD.open(dataFile, FILE_WRITE);
  if (!file){
    Serial.println("Failed to create file");
    while(1);
  }

  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }


  file.println("Time, Heading, Roll, Pitch,accl x, accl y, accl z, mag x, mag y,mag z, gyro x, gyro y,gyro z, latitude, longitude, altitude, airspeed");
  file.close();
  Serial.println("Setup complete!");
}


void loop() {

  sensors_event_t orientationData , angVelocityData , magnetometerData, accelerometerData;

  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

  unsigned long currentTime = millis();
  double Vm = 0;
  double kPa_sum = 0;
  double aR = analogRead(A0); // Measure analog differential pressure in binary
  
  for (int i = 0; i < Pdiff_Repeat; i++) // We measure the differential pressure multiple times and take the average (i.e. its an average filter).
  {
    aR = analogRead(A0);
    Vm = aR * (3.3/4095);   /*Your code here*/ // Convert analog differential pressure to voltage
    kPa_sum += (Vm-0.2)/.45; /*Your code here*/ // Sum and compute differential pressure measurement in kPa. Add you code to the right of the += to computer the pressure in kPA from the analog voltage. (You will use the data sheet equation).
  }
  kPa_sum = kPa_sum/Pdiff_Repeat; // Average differential pressure measurement in kPa
  if (kPa_sum < 0) // correct for random negative values
  {
    kPa_sum = 0;
  }
  double p = kPa_sum*1000; //Convert average differential pressure to Pa (1000 x kPa)
  
  float airSpeed = sqrt((2*p)/rho);
  
  
  uint8_t system, gyro, accel, mag = 0;
  
  sensors_event_t eulerEvent;
  bno.getEvent(&eulerEvent, Adafruit_BNO055::VECTOR_EULER);

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
  Serial.print("Fix type: ");
  
  Serial.println();
  Serial.println("Airspeed: ");
  Serial.println(airSpeed,2);
  
  delay(100); // Small delay between readings
}
