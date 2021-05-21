#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Árammérő paraméterek

//static const uint8_t analogPins[] = {A3,A4,A5}; //definiálni kell a pineket, amelyekre bekötjük a 3 áramváltóból érkező vezetéket
static const uint8_t analogPins[] = {A0};
unsigned long idokezdet;
unsigned long idovege;
int minta[900];
int mintadb = 900;
double sum;
double phase1;

// Hőmérő

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
#define MSG_BUFFER_SIZE  (150)
char msg_temp[MSG_BUFFER_SIZE];
char msg_phase1[MSG_BUFFER_SIZE];
double AmbientTemp;
double ObjectTemp;

// WiFi és mqtt

const char* ssid = "i40tk";
const char* password = "PbKbTtKa5";
const char* mqtt_server = "192.168.33.211";
//const char* mqtt_server = "152.66.34.82";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

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

void setup() {
  Serial.begin(115200);
  pinMode(A0,INPUT);
  //pinMode(A4,INPUT);
  //pinMode(A5,INPUT);
  mlx.begin();  
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback); 
}

void loop() {


  Serial.print("Ambient = "); 
  AmbientTemp = mlx.readAmbientTempC();
  Serial.print(AmbientTemp); 
  Serial.print("*C\tObject = "); 
  ObjectTemp = mlx.readObjectTempC();
  Serial.print(ObjectTemp); 
  Serial.println("°C");

  if (!client.connected()) {
    reconnect();
    Serial.println("Reconnect");
  }
  
  for (int j = 0; j < 1; j++) { // Azért 1 hosszú a ciklus mert csak elérhető analóg bemenetünk volt, illetve az eszköz is egy fázist használ, ha van több írjuk át 3-ra j értékét
  for (int i = 0; i < 900; i++) {
    //analogReference(EXTERNAL); // Nano-hoz volt külső referencia fesz 8266 nem használ ilyet
    int ertek = analogRead(analogPins[j]);
    minta[i] = ertek;
  }

  if (minta[0] > 512) {
    int i = 0;
    while (minta[i] > 512) {
      minta[i] = 0;
      i += 1;
    }
    if (minta[900] > 512) {
      int i = 0;
      while (minta[900-i] >= 512) {
        minta[900-i] = 0;
        i += 1;
      }
      while (minta[900-i] < 512) {
        minta[900-i] = 0;
        i +=1;
      }
    }
    else if (minta[900] < 512) {
      int i = 0;
      while (minta[900-i] < 512) {
        minta[900-i] = 0;
        i +=1;
      } 
    }
    else if (minta[900] == 512) {
      if (minta[900] - minta[896] < 0) {
        ;
      }
      else if (minta[900] - minta[896] > 0) {
        int i = 0;
        while (minta[900-i] <= 512 ){
          minta[900-i] = 0;
          i +=1;
      }
    }
   }
  }

  else if (minta[0] < 512) {
    int i = 0;
    while (minta[i] < 512) {
      minta[i] = 0;
      i += 1;
    }
    if (minta[900] < 0) {
      int i = 0;
      while (minta[900-i] <= 512) {
        minta[900-i] = 0;
        i += 1;
      }
      while (minta[900-i] > 512) {
        minta[900-i] = 0;
        i +=1;
      }
    }
    if (minta[900] > 0) {
      int i = 0;
      while (minta[900-i] > 512) {
        minta[900-i] = 0;
        i +=1;
      } 
    }
    if (minta[900] == 512) {
      if (minta[900] - minta[896] < 0) {
        int i = 0;
        while (minta[900-i] >= 512) {
        minta[900-i] = 0;
        i +=1;
        }
      }
      else if (minta[900] - minta[896] > 0) {
        ;
      }
    }
  }
  int darabnulla = 0;
  for (int i = 0; i < 900; i++) {
    if (minta[i] == 0) {
      darabnulla += 1;
    }
  }
  sum = 0;
  for (int i = 0; i < 900; i++) {
    if (minta[i] != 0) {
      //double u = (minta[i]*4.79/1024)-2.395;
      double u = (minta[i]*3.3/1024)-1.65;
      // Ezt a refencia fesz miatt kell árírni. Van egy 2.5v-os felhúzó az áramkörben,hogy ne kerüljön negatív fesz a bemenetre.
      sum += pow(u/200*1000, 2);
    }
  }
  if (j == 0){
    Serial.print("IRMS(A): ");
    phase1 = sqrt(sum/(mintadb-darabnulla));
    Serial.println(phase1);
  }
  else if (j == 1) {
    Serial.print("IRMS(B): ");
    Serial.println(sqrt(sum/(mintadb-darabnulla)));
    }
  else {
    Serial.print("IRMS(C): ");
    Serial.println(sqrt(sum/(mintadb-darabnulla)));
    }
  }// ez zárja nagy for ciklust

  sprintf(msg_temp,"04,");
  //snprintf(msg_temp, MSG_BUFFER_SIZE, "%s %.2f",msg_temp ,ObjectTemp); Az ID mégsem kell, azt kivesszük
  snprintf(msg_temp, MSG_BUFFER_SIZE, " %.2f" ,ObjectTemp);
  Serial.print("Publish message: ");
  Serial.println(msg_temp);
  client.publish("esztergapad/temperature", msg_temp);

  sprintf(msg_phase1,"05,");
  //snprintf(msg_phase1, MSG_BUFFER_SIZE, "%s %.2f",msg_phase1 ,phase1);
  snprintf(msg_phase1, MSG_BUFFER_SIZE, "%.2f" ,phase1);
  Serial.print("Publish message: ");
  Serial.println(msg_phase1);
  client.publish("esztergapad/phase1", msg_phase1);
  phase1 = 0;
  delay(200);
  
}
