#include "arduinoFFT.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>


arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
/*
These values can be changed in order to evaluate the functions
*/
const uint16_t samples = 1024; //This value MUST ALWAYS be a power of 2, tegyük 1024-re valós szin jelnél
const double signalFrequency = 1000;
const double samplingFrequency = 10000;
const uint8_t amplitude = 100;
/*
mqtt wifi parameters
*/
const char* ssid = "i40tk";
const char* password = "PbKbTtKa5";
const char* mqtt_server = "192.168.33.211";
//const char* mqtt_server = "152.66.34.82";
const int mqtt_port = 1883;
/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];
double MaxValues[10] = {0};
unsigned int sampling_period_us;
unsigned long microseconds;
/*
Mqtt  wifi parameters
*/
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (100)
char msg[MSG_BUFFER_SIZE];
int value = 0;

/*
FFT time measurement
*/

unsigned long microseconds_start;
unsigned long microseconds_finished;
unsigned long ConversionTime;

void setup_wifi() {

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void SearchPeaks(uint16_t samples, double *values, double *MaxIndex)
{
  
  double MaxValue = 0;
  

for(uint16_t j = 0; j < 10; j++){
  for(uint16_t i = 1; i < ((samples >> 1) + 1); i++)
    {
      if(values[i]>MaxValue && MaxIndex[0]!=i && MaxIndex[1]!=i && MaxIndex[2]!=i && MaxIndex[3]!=i && MaxIndex[4]!=i && MaxIndex[5]!=i && MaxIndex[6]!=i && MaxIndex[7]!=i && MaxIndex[8]!=i && MaxIndex[9]!=i ){
      MaxValue = values[i];
      MaxIndex[j] = i;
      }    
    }
    MaxValue = 0;
  }
}

void PrintIndexArray(double *Array){
  uint16_t i;
    for(i = 0; i < 10; i++){
      Serial.println(Array[i]*(samplingFrequency/samples));
      
      }
  }
void setup()
{
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Ready");
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  //client.setCallback(callback); 
}

void loop()
{
  /* Build raw data */
  double cycles = (((samples-1) * signalFrequency) / samplingFrequency); //Number of signal cycles that the sampling will read
  microseconds = micros();
  for (uint16_t i = 0; i < samples; i++)
  {
    //vReal[i] = int8_t((amplitude * (sin((i * (twoPi * cycles)) / samples))) / 2.0);/* Build data with positive and negative values*/
    vReal[i] = analogRead(A0);// reads real values
    vImag[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
   /* while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;*/
  }
 // microseconds_start = micros();

FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
 
  for( int i = 2; i < (samples/2); i++){
    Serial.print((samplingFrequency/samples)*i);
    Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vReal[i], 4);
    }

  SearchPeaks(samples,vReal,MaxValues);
  PrintIndexArray(MaxValues);
  double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  
  int j;
   sprintf(msg, "eszterga");
    for( int i = 0; i < 10; i++)
     {
   
        j=MaxValues[i];
        snprintf (msg, MSG_BUFFER_SIZE, "%s, freq=%.2f", msg, (samplingFrequency/samples)*MaxValues[i]);
     }
   Serial.print("Publish message: ");
   Serial.println(msg);
   client.publish("esp8266", msg);
   
 
   delay(2000); 
}  
