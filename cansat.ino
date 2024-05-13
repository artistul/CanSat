#include <MPU9250.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <MPU6050.h>
#include <LoRa.h>

// LoRa setup
#define LORA_SS    5  // LoRa SS pin
#define LORA_RST   14 // LoRa RST pin
#define LORA_DI0   2  // LoRa DI0 pin

// BME680 setup
Adafruit_BME680 bme;

// NEO-6M setup
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);

// MPU6050 setup
MPU9250 mpu;

void setup() {
  Serial.begin(115200);

  // LoRa setup
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DI0);
  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa initialization failed. Check your wiring.");
    while (1);
  }

  // BME680 setup
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // NEO-6M setup
  GPS_Serial.begin(9600);

  // MPU6050 setup
  Wire.begin();
  mpu.initialize();
}

void loop() {
  // BME680 data
  float temperature = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0F;
  float humidity = bme.readHumidity();
  float gasResistance = bme.readGas();

  // NEO-6M data
  float latitude = 0.0;
  float longitude = 0.0;
  float altitude = 0.0;

  while (GPS_Serial.available() > 0) {
    if (gps.encode(GPS_Serial.read())) {
      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        altitude = gps.altitude.meters();
      }
    }
  }

  // MPU6050 data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Normalize accelerometer and gyroscope values
  float accelerationX = ax / 16384.0;
  float accelerationY = ay / 16384.0;
  float accelerationZ = az / 16384.0;

  float gyroX = gx / 131.0;
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;

  // Prepare the data for transmission
  String sensorData = String(temperature) + ","
                    + String(pressure) + ","
                    + String(humidity) + ","
                    + String(gasResistance) + ","
                    + String(latitude) + ","
                    + String(longitude) + ","
                    + String(altitude) + ","
                    + String(accelerationX) + ","
                    + String(accelerationY) + ","
                    + String(accelerationZ) + ","
                    + String(gyroX) + ","
                    + String(gyroY) + ","
                    + String(gyroZ);

  // Convert the string to a char array
  char dataToSend[255];
  sensorData.toCharArray(dataToSend, 255);

  // Send the data over LoRa
  if (LoRa.beginPacket()) {
    LoRa.print(dataToSend);
    LoRa.endPacket();
  } else {
    Serial.println("Failed to begin LoRa packet transmission");
  }

  // Delay to control the transmission frequency
  delay(5000); // Adjust the delay as needed
}
