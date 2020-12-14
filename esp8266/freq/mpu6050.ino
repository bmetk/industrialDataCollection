#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "arduinoFFT.h"

#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#define WLAN_SSID       "UPC8136192"
#define WLAN_PASS       "p67davfnkwHt"

#define AIO_SERVER      "152.66.34.82"
#define AIO_SERVERPORT  1883              

#define AIO_USERNAME    "esp8266"
#define AIO_KEY         "asdasd"

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish broker = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/tempC");

Adafruit_MPU6050 mpu;


const uint16_t samples = 1024;
const double signalFrequency = 1000;
const double samplingFrequency = 5000;
const uint8_t amplitude = 100;
arduinoFFT FFT = arduinoFFT();
double vRealX[samples];
double vImagX[samples];

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}


void setup(void) {

    WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  delay(1);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);
  delay(1);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  Serial.println("");
  delay(100);
}

void PrintVector(double *vData, uint16_t bufferSize)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    Serial.println(vData[i], 4);
  }
  Serial.println();
  delay(1);
}


void loop() {
  delay(1);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  delay(1);
  for(uint16_t i = 0; i < samples; i++){
    vRealX[i] = a.acceleration.x;
    vImagX[i] = 0.0;
    delay(10);
  }
  Serial.println("Data:");
  PrintVector(vRealX, samples);
  FFT.Windowing(vRealX, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  Serial.println("Weighed data:");
  PrintVector(vRealX, samples);
  FFT.Compute(vRealX, vImagX, samples, FFT_FORWARD); /* Compute FFT */
  Serial.println("Computed Real values:");
  PrintVector(vRealX, samples);
  Serial.println("Computed Imaginary values:");
  PrintVector(vImagX, samples);
  FFT.ComplexToMagnitude(vRealX, vImagX, samples); /* Compute magnitudes */
  Serial.println("Computed magnitudes:");
  PrintVector(vRealX, (samples >> 1));
  double x = FFT.MajorPeak(vRealX, samples, samplingFrequency);
  Serial.println(x, 6);


  MQTT_connect();
 char message[100];
 sprintf(message,"kiskacsa,loc=Budapest freq=%f", x);
  if (! broker.publish(message)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
    delay(1000);

  
}
