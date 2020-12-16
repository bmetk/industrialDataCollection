#include"vector.h"
#include <driver/adc.h>

//#include<SoftwareSerial.h>



#include"mqttConnection.h"

#define SAMPLE_RATE 20

#define SAMPLE_VALUEDIFF 5 //in percent

#define A_PIN 36

//#define blokkos
//#define csuszoablak
#define csuszoablakidvel
//#define nullertekvizsgalat

//#define measurePin A1
unsigned long long realstarttime=0;
unsigned long long realendtime=0;

unsigned long long fullTime = 0;
unsigned long shortTime = 0;
unsigned long lastshortTime = 0;

unsigned long long lastButtonTime=0;

unsigned long long startTime = 0;
bool motvaDigitMeres=false;
double maximum=0;
double minimum=1;
int counter=0;
double offset=0;
long ido=0;
bool first=true;
int izecount=0;
bool bekapcsolva=false;
bool votma=false;

double egyMeres()
{
  double ez=0;

  double multisample[SAMPLE_RATE];
  bool multisampleB[SAMPLE_RATE];

  for(int i=0; i<SAMPLE_RATE; i++)
  {
    multisampleB[i]=true;
    multisample[i]= adc1_get_raw(ADC1_CHANNEL_0);
    ez+=multisample[i];
  }
  double atlag =ez/SAMPLE_RATE;
  int db=SAMPLE_RATE;
  

  for(int i=0; i<SAMPLE_RATE; i++)
  {
    if(multisample[i]>atlag+atlag/100*SAMPLE_VALUEDIFF||multisample[i]<atlag-(atlag/100*SAMPLE_VALUEDIFF))
    {
      multisample[i]=false;
      db--;
    }
  }

  ez=0;
  for(int i=0; i<SAMPLE_RATE; i++)
  {
    if(multisampleB[i])
    {
      ez+=multisample[i];
    }
  }
  return ez/db;
}



#ifdef blokkos//-------------------------------------

#include"blokkos.h"

#endif//--------------------------------------------------

#ifdef csuszoablak //-------------------------------------

#include"csuszoablak.h"

#endif//--------------------------------------------------

#ifdef csuszoablakidvel //--------------------------------

#include"csuszoablakidvel.h"

#endif //-------------------------------------------------

#ifdef nullertekvizsgalat//-------------------------------

#include"nullertekvizsgalat.h"

#endif //-------------------------------------------------

class event
{
  public:
    unsigned long long startTime;
    unsigned long long frequency;
    void (*func)(void);
    bool enabled;
};



void timeCucc()
{
  shortTime = micros();
  if (lastshortTime < shortTime)
  {
    fullTime += shortTime - lastshortTime;
  }
  else
  {

    fullTime += 0xffffffff - lastshortTime + shortTime;
  }
  lastshortTime = shortTime;
}



vector <event> v;

void setup()
{
  #ifdef csuszoablak
  nullaz();
  #endif
  #ifdef csuszoablakidvel
  nullaz();
  #endif

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_0);

  Serial.begin(115200);
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  //status = bme.begin();  
  
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(ledPin, OUTPUT);
  pinMode(A_PIN,INPUT);
  
  event* a = new event();
  a->startTime = 1000000;
  a->frequency = 1000;
  a->enabled = true;
  a->func = measurement_performMeasure;
  v.add(*a);
  offset=analogRead(A_PIN);
  counter=0;
  Serial.println("setup");

  Serial.print("value1: ");
  Serial.println(counter);
  
}


void loop() {

  timeCucc(); //idő számítása hogy 70 perc után is működjön (ne legyen overflow)
  for (int i = 0; i < v.size(); i++)
  {
    if (v[i].startTime < fullTime) //a v egy vektor amiben események vannak eltárolva, nem kell vele foglalkozni. A mérés egy esemény. 
    {                             //A függvénye kezdési ideje illetve frekvenciája van eltárolva benne és a meghívandó függvény pointere.
      if((fullTime-v[i].startTime)>1000ULL)
      {
        Serial.println("timeout");
      }
      else if((fullTime-v[i].startTime)<1000ULL)
      {
        
      }
      v[i].func();                            //meghívjuk a függvényt
      v[i].startTime += v[i].frequency;       //átidőzítjük a kezdést a legközelebbi időpontra
      
    }
  }
}
