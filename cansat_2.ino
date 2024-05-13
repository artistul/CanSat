#include <TinyGPSPlus.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <PCF8575.h>
#include <LoRa.h>
#include <MPU9250.h>
#include <SparkFunBQ27441.h>

#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>

//include libraries

//declare variables
const int ref_velocity = 8;  //meters per second
const int ref_angle = 15;    //degrees

int angle = 15;
int velocity = 0;
int time = 0;

String dataString = "";

//updated start
int altitude = 0;
int longitude = 0;
int latitude = 0;
float ax = 0;
float ay = 0;
float az = 0;
float gx = 0;
float gy = 0;
float gz = 0;
float hx = 0;
float hy = 0;
float hz = 0;
int gas = 0;
int temperature = 0;
int humidity = 0;
int pressure = 0;
int health = 0;
int power = 0;
unsigned int capacity = 0;
unsigned int fullCapacity = 0;
int current = 0;
unsigned int volts = 0;
unsigned int soc = 0;
//updated end

bool M1_A = 0;
bool M1_B = 0;
bool M2_A = 0;
bool M2_B = 0;
bool M3_A = 0;
bool M3_B = 0;
bool M4_A = 0;
bool M4_B = 0;

//declare module pins

//expander 0
#define STBY 14
#define PWM1 17
#define PWM2 11

#define IM1A 15
#define IM1B 16
#define IM2A 13
#define IM2B 12

#define EN1_A 0
#define EN1_B 1
#define EN2_A 2
#define EN2_B 3

//expander 1
#define STBY 3
#define PWM3 6
#define PWM4 0

#define IM3A 4
#define IM3B 5
#define IM4A 2
#define IM4B 1

#define EN3_A 20
#define EN3_B 19
#define EN4_A 18
#define EN4_B 17

//BME
#define BME_SCK 29
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 29

static const int RXPin = 1, TXPin = 0;

//additional declarations

//-BME
#define SEALEVELPRESSURE_HPA (1013.25)

//-GPS
static const uint32_t GPSBaud = 4800;
SoftwareSerial ss(RXPin, TXPin);

//-Battery babysitter
const unsigned int BATTERY_CAPACITY = 850;

//declare the modules
MPU9250 mpu;
Adafruit_BME680 bme;
TinyGPSPlus gps;
PCF8575 pcf_A(0x20);
PCF8575 pcf_B(0x21);
PCF8575 PCF[2] = { pcf_A, pcf_B };
MotorDriver steer;
MotorDriver incline;


void setup() {
  //setup the coms
  Serial.begin(115200);
  while (!Serial)
    ;

  Wire.begin();
  delay(100);

  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa starting failed :(");
    while (1)
      ;
  } else {
    Serial.println("LoRa setup done :)");
  }

  //setup MPU9250
  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed :(");
      delay(5000);
    }
  }
  Serial.println("Calibrating accelerometer & gyroscope...");
  Serial.println("  >DO NOT MOVE");
  delay(1000);
  mpu.calibrateAccelGyro();
  Serial.println("Calibration 1 succesful! :)");
  Serial.println("  >MOVE IN AN 8 SHAPED MOTION");
  delay(2000);
  Serial.println("Calibrating magnetometer...");
  mpu.calibrateMag();
  Serial.println("Calibration 2 succesful! :)");
  Serial.println("MPU setup done :)");

  //setup BME680
  if (!bme.begin()) {
    Serial.println("BME connection failed :(");
    while (1)
      ;
  }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);  // 320*C for 150 ms
  Serial.println("BME setup done :)");

  //setup GPS
  ss.begin(GPSBaud);
  delay(100);
  Serial.println("GPS setup done :)");

  //setup IO Expanders
  PCF[0].begin();
  PCF[1].begin();
  Serial.println("IO Expanders setup done :)");

  //setup motors
  steer.init();
  incline.init();
  Serial.println("Motors setup done :)")

    Serial.println("CanSat online! :D");
}

void loop() {
  //update all variables for main mission
  getBME();
  getIMU();
  getGPS();
  time = millis();
  transmit(0);

  //ETA

  //set motors
}

void transmit(bool caser) {
  //prepare data packet
  dataString = "";
  //caser==1 -> only main mission
  //caser==0 -> everything
  if (!caser) {
    dataString += "Altitude: " + String(altitude) + "\n";
    dataString += "Longitude: " + String(longitude) + "\n";
    dataString += "Latitude: " + String(latitude) + "\n";
    dataString += "Accelerometer (X,Y,Z): " + String(ax) + ", " + String(ay) + ", " + String(az) + "\n";
    dataString += "Gyroscope (X,Y,Z): " + String(gx) + ", " + String(gy) + ", " + String(gz) + "\n";
    dataString += "Magnetometer (X,Y,Z): " + String(hx) + ", " + String(hy) + ", " + String(hz) + "\n";
    dataString += "Gas: " + String(gas) + "\n";
    dataString += "Temperature: " + String(temperature) + "\n";
    dataString += "Humidity: " + String(humidity) + "\n";
    dataString += "Pressure: " + String(pressure) + "\n";
    dataString += "Soc: " + String(soc) + "%\n";
    dataString += "Voltage: " + String(volts) + " mV\n";
    dataString += "Amperage " + String(current) + " mA\n";
    dataString += "Capacity " + String(capacity) + " / ";
    dataString += String(fullCapacity) + " mAh\n";
    dataString += "Power: " + String(power) + " mW\n";
    dataString += "Health: " + String(health) + "%\n";
  } else {
    dataString += "Temperature: " + String(temperature) + "\n";
    dataString += "Humidity: " + String(humidity) + "\n";
    dataString += "Pressure: " + String(pressure) + "\n";
    dataString += "Time of collection: " + String(time) + "\n";
  }
  //send the packet
  LoRa.beginPacket();
  LoRa.print(dataString);
  LoRa.endPacket();

  //debug
  Serial.println(dataString);
}

void getIMU() {
  if (mpu.update()) {
    float ax = mpu.getAccX();
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();
    float gx = mpu.getGyroX();
    float gy = mpu.getGyroY();
    float gz = mpu.getGyroZ();
    float hx = mpu.getMagX();
    float hy = mpu.getMagY();
    float hz = mpu.getMagZ();
  } else {
    Serial.print(F("(MPU) Failed to perform reading :("));
  }
}

void getBME() {
  if (!bme.performReading()) {
    Serial.println("(BME) Failed to perform reading :(");
  } else {
    humidity = bme.humidity;
    temperature = bme.temperature;
    pressure = bme.pressure;
    gas = bme.gas_resistance / 1000;
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  }
}

void getGPS() {
  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  } else {
    Serial.print(F("(GPS) Failed to perform reading :("));
  }
}
void setupBQ(void) {
  // Use lipo.begin() to initialize the BQ27441-G1A and confirm that it's
  // connected and communicating.
  if (!lipo.begin())  // begin() will return true if communication is successful
  {
    // If communication fails, print an error message and loop forever.
    Serial.println("BQ connection failed :(");
    while (1)
      ;
  }
  Serial.println("BQ setup done :)");

  // Uset lipo.setCapacity(BATTERY_CAPACITY) to set the design capacity
  // of your battery.
  lipo.setCapacity(BATTERY_CAPACITY);
}

void getBQ() {
  // Read battery stats from the BQ27441-G1A
  unsigned int soc = lipo.soc();                    // Read state-of-charge (%)
  unsigned int volts = lipo.voltage();              // Read battery voltage (mV)
  int current = lipo.current(AVG);                  // Read average current (mA)
  unsigned int fullCapacity = lipo.capacity(FULL);  // Read full capacity (mAh)
  unsigned int capacity = lipo.capacity(REMAIN);    // Read remaining capacity (mAh)
  int power = lipo.power();                         // Read average power draw (mW)
  int health = lipo.soh();                          // Read state-of-health (%)
}

void rotate(bool axis, bool motor, int direction) {
  //axis=1 -> steer | axis=0 -> incline
  //direction = 1 -> forward | direction = -1 -> reverse | direction = 0 -> brake

  //steer     motor 1 = right      (run M1)
  //steer     motor 2 = left       (run M3)
  //incline   motor 1 = speed up   (run M1 & M3)
  //incline   motor 2 = brake      (run M2 & M4)


  int inA = 0;
  int inB = 0;
  if (direction == 1 || direction == -1) {
    inA = direction;
    inB = !direction;
  }
  if (axis) {
    if (motor) {
      PCF[0].write(IM1A, inA);
      PCF[0].write(IM1B, inB);
      PCF[0].write(PWM1, 1);
    } else {
      PCF[1].write(IM3A, inA);
      PCF[1].write(IM3B, inB);
      PCF[1].write(PWM3, 1);
    }
  } else {
    if (motor) {
      PCF[0].write(IM1A, inA);
      PCF[0].write(IM1B, inB);
      PCF[0].write(PWM1, 1);

      PCF[1].write(IM3A, inA);
      PCF[1].write(IM3B, inB);
      PCF[1].write(PWM3, 1);
    } else {
      PCF[0].write(IM2A, inA);
      PCF[0].write(IM2B, inB);
      PCF[0].write(PWM2, 1);

      PCF[1].write(IM4A, inA);
      PCF[1].write(IM4B, inB);
      PCF[1].write(PWM4, 1);
    }
  }
}
