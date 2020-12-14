#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <ESP8266WiFi.h>
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
 
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
 
void setup() 
{
  Serial.begin(9600);
  mlx.begin();  

  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

}

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
 
void loop() 
{
 MQTT_connect();
 char message[100];
 sprintf(message,"kiskacsa,loc=Budapest tempC=%f", mlx.readObjectTempC());
  if (! broker.publish(message)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  
  Serial.print("Ambient = "); 
  Serial.print(mlx.readAmbientTempC()); 
  Serial.print("*C\tObject = "); 
  Serial.print(mlx.readObjectTempC()); 
  Serial.println("*C");
  Serial.print("Ambient = "); 
  Serial.print(mlx.readAmbientTempF()); 
  Serial.print("*F\tObject = "); 
  Serial.print(mlx.readObjectTempF()); 
  Serial.println("*F");
 
  Serial.println();
  delay(1000);
}
