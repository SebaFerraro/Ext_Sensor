#include <Arduino.h>
#include <SPI.h>
//#include <Wire.h>
#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define DHT_DATA 23
#define DHTTYPE DHT11
#define RLY_P1 26
#define RLY_P2 27
#define ANALOG_PIN_0 35
#define TOKEN_TB "f1KmDbI5m7y5xmTqAEol"

DHT dht(DHT_DATA, DHTTYPE);
static int taskCore = 0;
int Temp=0;
int Nivel=1;
char bufferT[28]="";
int analog_value = 0;
boolean Auto[2] = {false,false};
float Lin[2] = {0,0};
const char* ssid     = "wifi";
const char* password = "secret1703secret1703";
static int Wconectado = 0;
boolean gpioState[36] = {false,false};
const char* mqtt_server = "190.2.22.61";
WiFiClient espClient;
PubSubClient client(espClient);

void WiFiEvent(WiFiEvent_t event)
{
    Serial.printf("[WiFi-event] event: %d\n", event);

    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      Wconectado=1;
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection.  Attempting to reconnect...");
      //WiFi.reconnect();
      Wconectado=0;
      break;
    case SYSTEM_EVENT_STA_START:
      Serial.println("ESP32 station start");
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("ESP32 station connected to AP");
      break;
    default:      
      Serial.println("Unhandled WiFi Event raised.");
      break;
    }
}

void Wifi_init(){ 
  WiFi.disconnect(true);
  delay(1000);
  WiFi.onEvent(WiFiEvent);
  WiFi.begin(ssid, password);
}
String get_status() {
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& data = jsonBuffer.createObject();
  data[String(RLY_P1)] = gpioState[RLY_P1] ? true : false;
  data[String(RLY_P2)] = gpioState[RLY_P2] ? true : false;
  data["AutoP1"] = Auto[0] ? true : false;
  data["AutoP2"] = Auto[1] ? true : false;
  data["LinP1"] = Lin[0];
  data["LinP2"] = Lin[1];
  char payload[256];
  data.printTo(payload, sizeof(payload));
  String strPayload = String(payload);
  Serial.print("Estado: ");
  Serial.println(strPayload);
  return strPayload;
}

String get_gpio_status() {
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& data = jsonBuffer.createObject();
  data[String(RLY_P1)] = gpioState[RLY_P1] ? true : false;
  data[String(RLY_P2)] = gpioState[RLY_P2] ? true : false;
  char payload[256];
  data.printTo(payload, sizeof(payload));
  String strPayload = String(payload);
  Serial.print("Gpio Estado: ");
  Serial.println(strPayload);
  return strPayload;
}

void set_gpio_status(int pin, boolean enabled) {
   digitalWrite(pin, enabled ? HIGH : LOW);
   // Update GPIOs state
   gpioState[pin] = enabled;
}

String get_Auto_status(int pin, String methodName) {
  //StaticJsonBuffer<200> jsonBuffer;
  //JsonObject& data = jsonBuffer.createObject();
  String data =String(Auto[pin] ? true : false);
  String payload = "{\"";
      //payload += "\"method\":";
      payload += methodName;
      payload += "\":";
      //payload += "\"params\":";
      payload += "true";
      payload += "}";

      // Send payload
  char attributes[200];
  payload.toCharArray( attributes,200);
  
  
  //char payload[256];
  //data.printTo(payload, sizeof(payload));
  //String strPayload = String(payload);
  Serial.print("Auto Pin: ");
  Serial.print(pin);
  Serial.print(" : ");
  Serial.print(attributes);
  Serial.println(".");
  //return strPayload;
  return attributes;
}

void set_Auto_status(int pin, boolean enabled) {
   Auto[pin] = enabled;
   Serial.print("Set_Auto :");
   Serial.print(Auto[pin]);
   Serial.print(" Pin:");
   Serial.println(pin);
}

String get_Lin_Value(int pin, String methodName) {
  //StaticJsonBuffer<200> jsonBuffer;
  //JsonObject& data = jsonBuffer.createObject();
  
  String data =String(Lin[pin]);
  String payload = "{";
      //payload += "\"method\":\"";
      //payload += methodName;
      //payload += "\",";
      //payload += "\"params\":";
      payload += "\"value\":\"77.50\"";
      payload += "}";

      // Send payload
  char attributes[200];
  payload.toCharArray( attributes,200);
  
  //data = Lin[pin];
  //char payload[256];
  //data.printTo(payload, sizeof(payload));
  //String strPayload = String(payload);
  Serial.print("Lin Pin: ");
  Serial.print(pin);
  Serial.print(" : ");
  //Serial.println(strPayload);
  //return strPayload;
  Serial.print(attributes);
  Serial.println(".");
  //return strPayload;
  return attributes;
}

void set_Lin_Value(int pin, float  valor) {
   Lin[pin] = valor;
   Serial.print("Set_Lin :");
   Serial.print(Lin[pin]);
   Serial.print(" Pin:");
   Serial.println(pin);
}

void on_message(char* topic, byte* payload, unsigned int length) {

  Serial.println("Mensaje Recibido");

  char json[length + 1];
  strncpy (json, (char*)payload, length);
  json[length] = '\0';

  Serial.print("Topic: ");
  Serial.println(topic);
  Serial.print("Mensaje: ");
  Serial.println(json);

  
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& data = jsonBuffer.parseObject((char*)json);

  if (!data.success())
  {
    Serial.println("parseObject() Error");
    return;
  }
  Serial.println("Data:");
  data.printTo(Serial);
  Serial.println("----");
  String methodName = String((const char*)data["method"]);

  if (methodName.equals("getGpioStatus") or methodName.equals("getValue")) {
        String responseTopic = String(topic);
        responseTopic.replace("request", "response");
        Serial.print("response :");
        Serial.println(responseTopic.c_str());
        Serial.print("datos:");
        Serial.println(get_gpio_status().c_str());
        client.publish(responseTopic.c_str(), get_gpio_status().c_str());
        
  } else if (methodName.equals("setGpioStatus")) {
        // Update GPIO status and reply
        set_gpio_status(data["params"]["pin"], data["params"]["enabled"]);
        String responseTopic = String(topic);
        responseTopic.replace("request", "response");
        client.publish(responseTopic.c_str(), get_gpio_status().c_str());
        client.publish("v1/devices/me/attributes", get_status().c_str());
        
  } else if(methodName.equals("setAutoP2") or methodName.equals("setAutoP1")) {
        int indx;
        if(methodName.equals("setAutoP1")){
          indx=0;
        }else{
          indx=1;
        }
        set_Auto_status(indx,data["params"]);        
        String responseTopic = String(topic);
        responseTopic.replace("request", "response");
        Serial.print("response :");
        Serial.println(responseTopic.c_str());
        Serial.print("datos:");
        data.printTo(Serial);
        client.publish(responseTopic.c_str(), data["params"]);
        client.publish("v1/devices/me/attributes",get_status().c_str());
  
  } else if(methodName.equals("getAutoP2") or methodName.equals("getAutoP1")) {
        int indx;
        if(methodName.equals("getAutoP1")){
          indx=0;
        }else{
          indx=1;
        }
        String responseTopic = String(topic);
        responseTopic.replace("request", "response");
        client.publish(responseTopic.c_str(), get_Auto_status(indx,methodName).c_str());
         
  }else if(methodName.equals("setLinP1") or methodName.equals("setLinP2")) {
        int indx;
        if(methodName.equals("setLinP1")){
          indx=0;
        }else{
          indx=1;
        }
        float val=data["params"];
        set_Lin_Value(indx,val);
        String responseTopic = String(topic);
        responseTopic.replace("request", "response");
        client.publish(responseTopic.c_str(), data["params"]);
        client.publish("v1/devices/me/attributes",get_status().c_str());
        
  }else if(methodName.equals("getLinP1") or methodName.equals("getLinP2")) {
        int indx;
        if(methodName.equals("getLinP1")){
          indx=0;
        }else{
          indx=1;
        }
        String responseTopic = String(topic);
        responseTopic.replace("request", "response");
        client.publish(responseTopic.c_str(), get_Lin_Value(indx,methodName).c_str());
  }     
 
}

void send_mqtt(float t, float h, float hic, float co){
  if (Wconectado == 1){
    if(!client.connected()) {
      Serial.print("Conectando ThingsBoard node ...");
      if ( client.connect("Ext_Sensor_01", TOKEN_TB, NULL) ) {
        Serial.println( "[DONE]" );
        int rsus=client.subscribe("v1/devices/me/rpc/request/+");
        Serial.print( "[SUBSCRIBE]" );
        Serial.println(rsus);
        client.publish("v1/devices/me/attributes", get_status().c_str());
        Serial.print("Enviando GPIO status ...");
        Serial.println(rsus);
      } else {
        Serial.print( "[FAILED] [ rc = " );
        Serial.print( client.state() );
      }
    }  
    if (client.connected()) {
        
      String temperatura = String(t);
      String humedad = String(h);
      String stermica = String(hic);
      String sco = String(co);
  
      String payload = "{";
      payload += "\"temperature\":";
      payload += temperatura;
      payload += ",";
      payload += "\"humedad\":";
      payload += humedad;
      payload += ",";
      payload += "\"stermica\":";
      payload += stermica;
      payload += ",";
      payload += "\"co\":";
      payload += sco;
      payload += "}";

      // Send payload
      char attributes[100];
      payload.toCharArray( attributes, 100 );
      int rsus=client.publish( "v1/devices/me/telemetry", attributes );
      Serial.print( "Publish : ");
      Serial.println(rsus);
      //client.publish(TEMP_TOPIC, msg);
      Serial.println( attributes );
    }
  }
}


void coreTask( void * pvParameters ){
 
    String taskMessage = "Corriendo en core ";
    taskMessage = taskMessage + xPortGetCoreID();
    Serial.println(taskMessage);
    while(true){
      if(Wconectado == 1){
          if(!client.connected()) {
              Serial.print("Conectando a ThingsBoard node ...");
              if ( client.connect("Ext_Sensor_01", TOKEN_TB, NULL) ) {
                 Serial.println( "[DONE]" );
                 int rsus=client.subscribe("v1/devices/me/rpc/request/+");
                 Serial.print( "[SUBSCRIBE]" );
                 Serial.println(rsus);
                 rsus=client.publish("v1/devices/me/attributes", get_status().c_str()); 
                 Serial.print("Enviando GPIO status ...");
                 Serial.println(rsus);
              } else {
                 Serial.print( "[FAILED] [ rc = " );
                 Serial.print( client.state() );
              }
          }
          //Serial.println("Client LOOP");
          //Serial.print("LOOP ");
          if(! client.loop()){
            Serial.println("Error en Loop.");  
          }
          //Serial.println(taskMessage);
      }else{
      Serial.print("No conectado wifi:");
      Serial.println(Wconectado);
    }
   delay(800);
   }
}

void setup() { 
  Serial.begin(115200);
  pinMode(RLY_P1, OUTPUT);
  pinMode(RLY_P2, OUTPUT);
  pinMode(DHT_DATA,INPUT_PULLUP);
  analogReadResolution(12); //12 bits
  analogSetAttenuation(ADC_11db);  //For all pins
  analogSetPinAttenuation(ANALOG_PIN_0, ADC_11db); //0db attenu
  delay(400);
  dht.begin();
    
  client.setServer(mqtt_server, 1883);
  client.setCallback(on_message);
  xTaskCreatePinnedToCore(coreTask, "coreTask", 10000, NULL, 0, NULL, taskCore);
  Serial.println("Hilo Creado...");  
}


void loop() {
 if (Wconectado == 0){
   Serial.println("Error No conectado wifi Wifi_init.");
   Wifi_init();
 }
    
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println("Error obteniendo los datos del sensor DHT11");
    delay(600);
    return;
  }
  float hic = dht.computeHeatIndex(t, h, false);
  int analog_value=analogRead(ANALOG_PIN_0);
  delay(60000);
  Serial.print("Valor CO:");
  Serial.println(analog_value);
  Serial.print("Humedad :");
  Serial.print(h);
  Serial.print(" Temperatura :");
  Serial.print(t);
  Serial.print(" Sensacion Termica :");
  Serial.println(hic);
  String taskMessage = "Corriendo en core ";
  taskMessage = taskMessage + xPortGetCoreID();
  Serial.println(taskMessage);
  send_mqtt(t,h,hic,analog_value);
 }
 

