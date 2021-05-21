#include <MPU9250_asukiaaa.h>
#include "arduinoFFT.h"
//#include <TaskScheduler.h>
#include <WiFi.h>
#include <PubSubClient.h>
TaskHandle_t Task1;
TaskHandle_t Task2;

// LED pins
const int led1 = 2;
const int CHANNEL = 36;
const byte interruptPin = 5;

#define SDA_PIN 21                                                      // ESP32-höz más PIN-ek kellenek
#define SCL_PIN 22

MPU9250_asukiaaa mySensor;
arduinoFFT FFT = arduinoFFT();

const int sampleNum = 1024;
unsigned long samplingFrequency_accelo = 1000;                            //[Hz]
unsigned long samplingFrequency_micro = 6000;  //10000;                          //[Hz]

double vReal_accel[sampleNum];
double vReal_micro[sampleNum];
double vImag_accel[sampleNum];
double vImag_micro[sampleNum];

unsigned long previousMicros = 0;
unsigned long samplingTime_accelo = round(1000000.0/samplingFrequency_accelo);                                                //Sampling time in microseconds
unsigned long samplingTime_micro = round(1000000.0/samplingFrequency_micro);
uint8_t samplingTimeRatio = round(samplingTime_accelo/samplingTime_micro);
#define samplingTime (((samplingTime_accelo) < (samplingTime_micro)) ? (samplingTime_accelo) : (samplingTime_micro))
unsigned long microseconds;

const uint8_t topPeakNum = 10;                                                                                               //N darab legmagasabb csúcs;       N <= sampleNum/2
double topPeak_accel[topPeakNum];
double topPeak_micro[topPeakNum];

// rpm szenzorhoz
void IRAM_ATTR handleInterrupt();
unsigned long trigger;
unsigned long lasttrigger = 0;
double rpm;
bool newValue = false;

// WIFI és mqtt


const char* ssid = "i40tk";
const char* password = "PbKbTtKa5";

const char* mqtt_server = "192.168.33.211";
//const char* mqtt_server = "152.66.34.82";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

#define MSG_BUFFER_SIZE  (150)
char msg_micro[MSG_BUFFER_SIZE];
char msg_accel[MSG_BUFFER_SIZE];
char msg_rpm[MSG_BUFFER_SIZE];

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


void reconnect (){
  while(!client.connected()){
    Serial.print("Connecting MQTT...");
    String clientID = "espressif";
    if(client.connect("espressif")){
      Serial.println("connected");
      } else {
        Serial.print("failed");
        Serial.println(client.state());
        Serial.println(" Try again in 5 sec...");
        delay(5000);
        
        }
    }
}

void callback(char *topic, byte *payload, unsigned int length) {
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);
    Serial.print("Message:");
    for (int i = 0; i < length; i++) {
        Serial.print((char) payload[i]);
    }
    Serial.println();
    Serial.println("-----------------------");
}

void topPeaks(double* vReal, double* topPeak, unsigned long samplingFrequency)
{
  double maxY = 0;
  uint16_t IndexOfMaxY = 0;

  for (uint16_t j = 0; j < topPeakNum; j++)
  {
    maxY = 0;
    IndexOfMaxY = 0;
    for (uint16_t i = 1; i < ((sampleNum >> 1) + 1); i++) 
    { 
      if ((vReal[i-1] < vReal[i]) && (vReal[i] > vReal[i+1])) 
      {
        if (vReal[i] > maxY) 
        {
          maxY = vReal[i];
          IndexOfMaxY = i;
        }
      }
      //wdt_reset();
    }
    
    double delta = 0.5 * ((vReal[IndexOfMaxY-1] - vReal[IndexOfMaxY+1]) / (vReal[IndexOfMaxY-1] - (2.0 * vReal[IndexOfMaxY]) + vReal[IndexOfMaxY+1]));
    double interpolatedX = ((IndexOfMaxY + delta)  * samplingFrequency) / (sampleNum-1);
    
    if(IndexOfMaxY==(sampleNum >> 1)) 
    {                                                      
      interpolatedX = ((IndexOfMaxY + delta)  * samplingFrequency) / (sampleNum);
    }
    
    topPeak[j] = interpolatedX;
    vReal[IndexOfMaxY] = 0.0;
  }
}

void IRAM_ATTR handleInterrupt(){
      //newValue = true;
      trigger = micros();
      unsigned long delta = trigger  - lasttrigger;
      unsigned long tper = 6*delta;
      //It its not allowed to have a Float variable inside the Callback function.
      //After replacing this with a Double variable everything works!
      //Floats use the FPU while doubles are calculated in software. 
      //For reasons, using the FPU inside an interrupt handler is currently not supported.;
      rpm = (1/((double)tper))*60000000;
      lasttrigger = trigger;
      
}



void setup() {
  Serial.begin(115200); 
  pinMode(led1, OUTPUT);
  Wire.begin(SDA_PIN, SCL_PIN);
  mySensor.setWire(&Wire);
  mySensor.beginAccel(ACC_FULL_SCALE_2_G);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  pinMode(interruptPin, INPUT);
  

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 
}

//Task1code: mikrofon és rezgésmérő
void Task1code( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    uint16_t i = 0;
  uint16_t addr_accelo = 0;
  uint16_t addr_micro = 0;
  previousMicros = micros();
  
  while (i < sampleNum )
  {
    if(micros() >= previousMicros + samplingTime) 
    {
          mySensor.accelUpdate();  
          vReal_accel[addr_accelo] = (double)mySensor.accelSqrt();
          vImag_accel[addr_accelo] = 0.0;
          addr_accelo++;
        
        previousMicros += samplingTime;

        i++;
        //wdt_reset(); 
    }
  }
  microseconds = micros();
  for (uint16_t i = 0; i < sampleNum; i++)
  {
    vReal_micro[i] = analogRead(CHANNEL);
    vImag_micro[i] = 0.0;
    while(micros() - microseconds < samplingTime_micro){
        //empty loop
      }
      microseconds += samplingTime_micro;
  }
  
  addr_accelo = 0;
  addr_micro = 0;

  //startTime = micros();

  FFT.Windowing(vReal_accel, sampleNum, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal_accel, vImag_accel, sampleNum, FFT_FORWARD); 
  FFT.ComplexToMagnitude(vReal_accel, vImag_accel, sampleNum);

  FFT.Windowing(vReal_micro, sampleNum, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal_micro, vImag_micro, sampleNum, FFT_FORWARD); 
  FFT.ComplexToMagnitude(vReal_micro, vImag_micro, sampleNum);

  //endTime = micros();
  
  //deltaTime = endTime-startTime;
  //Serial.print("Az eltelt idő: ");
  //Serial.println(deltaTime);

  topPeaks(vReal_accel, topPeak_accel, samplingFrequency_accelo);
  topPeaks(vReal_micro, topPeak_micro, samplingFrequency_micro);

  for (int i = 0 ; i < topPeakNum; i++)
  {
    Serial.print(topPeak_accel[i]); 
    Serial.print(" ; "); 
    //wdt_reset();
  }
  Serial.println();
  
  for (int i = 0; i < topPeakNum; i++)
  {
    Serial.print(topPeak_micro[i]); 
    Serial.print(" ; "); 
    //wdt_reset(); 
  }
  Serial.println();
  double mqtt_top_micro = topPeak_micro[0];
  double mqtt_top_accel = topPeak_accel[0];

  if (!client.connected()) {
    reconnect();
    Serial.println("Reconnect");
  }
  
  sprintf(msg_micro,"01,");
  sprintf(msg_accel,"02,");
  /*for( int i = 0; i < 10; i++){
    j = topPeak_micro[i];
    snprintf(msg_micro, MSG_BUFFER_SIZE, "%s %.2f",msg_micro ,topPeak_micro[i]);
    snprintf(msg_accel, MSG_BUFFER_SIZE, "%s %.2f",msg_accel ,topPeak_accel[i]);
    }*/
    // A megjeleníthetőség miatt csak a legdominánsabb frekvenciát küldjük mqtt-n
    snprintf(msg_micro, MSG_BUFFER_SIZE, " %.2f" ,mqtt_top_micro);
    snprintf(msg_accel, MSG_BUFFER_SIZE, " %.2f" ,mqtt_top_accel);
  Serial.print("Publish message: ");
  Serial.println(msg_micro);
  Serial.println(msg_accel);
  client.publish("esztergapad/frequency_microphone", msg_micro);
  client.publish("esztergapad/frequency_gyro", msg_accel);
  Serial.print("Task Freq running on core ");
  Serial.println(xPortGetCoreID());
  Serial.println("=======================================================");
  } 
}

//Task2code:fordulatszém
void Task2code( void * pvParameters ){
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);
  //Allocating an external interrupt will always allocate it on the core that does the allocation.
  //Freeing an external interrupt must always happen on the same core it was allocated on.
  //Disabling and enabling external interrupts from another core is allowed.
  Serial.print("Task RPM running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    //if(newValue){
      sprintf(msg_rpm,"03,");
      snprintf(msg_rpm, MSG_BUFFER_SIZE, "%.2f" ,rpm);
      Serial.print("Publish message: ");
      Serial.println(msg_rpm);
      client.publish("esztergapad/main_spindle_rpm", msg_rpm);
      Serial.print("Task RPM running on core ");
      Serial.println(xPortGetCoreID());
      rpm = 0;
      delay(500);
      
     // newValue = false;
    //}
  }
}

void loop() {
  
}
