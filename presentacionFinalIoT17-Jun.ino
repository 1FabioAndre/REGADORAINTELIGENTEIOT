#include <WiFiManager.h> // Incluye la librería WiFiManager
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>

// ---------------------------------------------------------------------------------------------------

const char * MQTT_BROKER_HOST = "a29cg1x4fjkk3s-ats.iot.us-east-2.amazonaws.com";

const int MQTT_BROKER_PORT = 8883;

const char * MQTT_CLIENT_ID = "ESP-32";                                               // Unique CLIENT_ID
const char * UPDATE_TOPIC = "$aws/things/thing/shadow/update";              // publish
const char * SUBSCRIBE_TOPIC = "$aws/things/thing/shadow/update/delta";     // Subscribe

WiFiClientSecure wiFiClient;
PubSubClient mqttClient(wiFiClient);

StaticJsonDocument<JSON_OBJECT_SIZE(64)> inputDoc;
StaticJsonDocument<JSON_OBJECT_SIZE(20)> outputDoc;
char outputBuffer[500]; //128

const int pinSensorHumedad = 34; 
const int pinSensorTanqueAgua = 35;
// Configuración del sensor de temperatura DHT11
#define DHTPIN 27
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);


void connectToWiFi() {
    WiFiManager wm;

    //wm.resetSettings();

    bool res;
    
    res = wm.autoConnect("AutoConnectAP","password"); // password protected ap

    if(!res) {
        Serial.println("Failed to connect");
        ESP.restart();
    } 
    else { 
        Serial.println("connected...yeey :)");
    }
}

class BombaAgua{
  private:
    int pin;
  public:
    BombaAgua(int pinBomba): pin(pinBomba){
      pinMode(pin, OUTPUT);
    }

    void encenderBomba(){
      digitalWrite(pin, HIGH);

    }

    void apagarBomba(){
      digitalWrite(pin, LOW);
    }
};

BombaAgua bomba(12);

const char AMAZON_ROOT_CA1[] PROGMEM = R"EOF(
  -----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

const char CERTIFICATE[] PROGMEM = R"KEY(
  -----BEGIN CERTIFICATE-----
MIIDWjCCAkKgAwIBAgIVANLVX2cUdH/CyPB9BS2YrDUzzetRMA0GCSqGSIb3DQEB
CwUAME0xSzBJBgNVBAsMQkFtYXpvbiBXZWIgU2VydmljZXMgTz1BbWF6b24uY29t
IEluYy4gTD1TZWF0dGxlIFNUPVdhc2hpbmd0b24gQz1VUzAeFw0yNDA0MTYwMDM1
MTFaFw00OTEyMzEyMzU5NTlaMB4xHDAaBgNVBAMME0FXUyBJb1QgQ2VydGlmaWNh
dGUwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDA4R3Q2mOVBJ0nqBqV
JESHpE/bjFYoQyR2RF28/WrIcEXgPyni1yM9Tlx8+5ahmP2fs+Xy3y1pUbcyO5rL
ACXibH2FRxdAequgsW8i4XNzEV8tZ5hNxFQBUI/ebKEzZX3BKeypNk0oXhNrGZo2
j3XNwioQAP4b3cvkU104dAzE25yCRAZhk+XY2nXYzeKdglVRPuSTDim+6eF9Y6RT
y4n3DVQyi8Snh18F7NtxOYTe5g5g7O5oCSD+GzmsonUlnZDatY5654hbVEi3pb6+
iNSLC3zXWInpoGxzNg9943GRiH7SSMy2/npfhLz6WeewbbpBgezo0iPRVcfvj7na
AlN/AgMBAAGjYDBeMB8GA1UdIwQYMBaAFC5avc3T2IFm58RhEU0azi6T+iIsMB0G
A1UdDgQWBBT2UeJ4MQXAc77owIxvJhkRA1M1JjAMBgNVHRMBAf8EAjAAMA4GA1Ud
DwEB/wQEAwIHgDANBgkqhkiG9w0BAQsFAAOCAQEAle3aUxe03mgQmXwxjjrsdBJA
OfpJRwyns0MFhBetPVHkcg+jDgLJkaKKjARPOreLQYU1ueDIh22npfxHHpQTnBXJ
ceulCyGgDLW8erJh2H8vH9AsxVL0w65VlYeNxSg0nemuR5AJ05d6mZeCbSp5/DWG
ncgDXokEK9/LQmC3wo0PUswu1AAzuenJ2XwbBH+RGkn59/aIR/00Q5quKFFCTx7U
T2RoXOxtmxdFzsdSlaf/gfT6hiqmILKcApMp+J1ZSY0Yesj2WnpxYSkTqKIGqqgF
+3BOIq4PW9nkGdl9xmDo4RB17c1I8ojm6Q7l+gr/eFHI0SDrKwPDK9U1yFMNtg==
-----END CERTIFICATE-----
)KEY";
  
const char PRIVATE_KEY[] PROGMEM = R"KEY(
  -----BEGIN RSA PRIVATE KEY-----
MIIEowIBAAKCAQEAwOEd0NpjlQSdJ6galSREh6RP24xWKEMkdkRdvP1qyHBF4D8p
4tcjPU5cfPuWoZj9n7Pl8t8taVG3MjuaywAl4mx9hUcXQHqroLFvIuFzcxFfLWeY
TcRUAVCP3myhM2V9wSnsqTZNKF4TaxmaNo91zcIqEAD+G93L5FNdOHQMxNucgkQG
YZPl2Np12M3inYJVUT7kkw4pvunhfWOkU8uJ9w1UMovEp4dfBezbcTmE3uYOYOzu
aAkg/hs5rKJ1JZ2Q2rWOeueIW1RIt6W+vojUiwt811iJ6aBsczYPfeNxkYh+0kjM
tv56X4S8+lnnsG26QYHs6NIj0VXH74+52gJTfwIDAQABAoIBABYQaoG+yc99NKEZ
uNjAylod8MizuBNpAt3ImlSJRNLjDQtHDnqVk8Fqmmftu+CGiNThFnf/EcnPgD5t
1RpNzQem04EQjddhcQqz4XvDAJr0LMgb5q9WuylfVuXh1Vr0zDrxmdINlSv9/wis
aBi/toEwKfGku2zipXVcF/jiRaerKNg50sbn29l3EOtO41TuuDiLikxPVQP+FgZC
yhgWSERgs1HAWAtuPA2tdjCoV19Ch3fWFb51cpQr7GF6+/FjZAdREvL1ZwDHfOmz
blpS8cWy3vuM+w1t8fJuoPD6Pnq+0DSQNQryDZh6/OWQx3hXQYA/cctbnJF/lJ5o
aPvaPKECgYEA3yPpVlZsrYvBZ8hivSmrkoIc382WvaDVCMQ6EsQAFqvotlfI/4QP
rzIfiNFepYptpCfYb8ORzn7ER7C6sFfR7r6Exbf55kQSBR3P/Wmb0VJ775QjPsPN
jFVhn6HtslD+fXP5HTpGb6xZfp56h0bJvbe8ZDdWRaZky+G+Mvm4HLECgYEA3Uho
uNQe5XvQV8ZTfqaLoeHjQwbEptbY6wOksMa5cVvXjqTJMlkr1G+Y4KNAY++0ulG/
28+fEW0fRExyFXJFJe1wmIt2J3Z6YMpHHxLXXC+gXO4dePWBfYVyQGiJas6UoHJF
7tMbhxRDVqcTR2D6AhZBE4qt09FEKaP+9o/dvy8CgYA1FY8COic6lh/Zt8M9qzck
8I079OXikOt9XWlPY4991UHUd0fa+ajdjfgQjXaNvUPeJJiJ5iW+0UuSnombQBky
SeK+QslRrWn4C6Kab9Bg2NWhJkXIPb6dnwZNerFYlYolgDyIZn+xO3hC9iLCIeYG
mzpXQQ7mHPKnyjl8WQoi0QKBgQCkRJuREc751sccUdsruuEPRIwb9stHe1i+Zg79
OBj0ARTtDIFbgfzakBmyMR6c0ZaddhByUheorRQ39HQAXbrdY/1hEK6erwI8Fg1k
EO2UvrpSImX4pHADSWw+ShwxELgev5YQq+DUjwNKMW9LXr9Zi0G2Cw3tn3z1WIvu
Z3Ba+wKBgHPzdG3avEDm30e6+grn/COtH+npYtLP5VXHUOxHdcisIAPqKhA0csuv
mTLlpqsKiGvmkHQUFBaYIYVHZR+xGpfdhbPN0Yu9lhcv59FLd9k5ClQgi/q7UDWC
VP//8T5WjXAUYsIVjwrY7zlgq4xVuolSg04K1e/54OjwlFsaEM/A
-----END RSA PRIVATE KEY-----
)KEY";

String modoObjeto = "X";
int builtInWaterPump = 0;
float nivelHumedadBajaConfigurada = 0;
String estadoBomba = "apagada";
String tierra = "X";
String tanque = "X";

int lecturaSensorHumedad;
int lecturaSensorAgua;

float nivelHumedad;
float nivelTanqueAgua;

void reportBuiltInWaterPump(){  // PARA EL SETUP - HACE ACTIVAR UN DELTA QUE ME AYUDA A OBTENER ESTADOS PREVIOS DE MI OBJETO
  outputDoc["state"]["reported"]["modo"] = modoObjeto;
  outputDoc["state"]["reported"]["builtInWaterPump"] = builtInWaterPump;
  outputDoc["state"]["reported"]["tierra"] = tierra;
  outputDoc["state"]["reported"]["tanque"] = tanque;
  outputDoc["state"]["reported"]["estadoBomba"] = estadoBomba;

  serializeJson(outputDoc, outputBuffer);
  mqttClient.publish(UPDATE_TOPIC, outputBuffer);
}
 
// Estas cumpliendo el desired debes reportar eso
void regandoPlanta(){
  bomba.encenderBomba();
  builtInWaterPump = 0;
  outputDoc["state"]["reported"]["builtInWaterPump"] = builtInWaterPump;
  outputDoc["state"]["desired"]["builtInWaterPump"] = builtInWaterPump;
  estadoBomba = "encencida";
  outputDoc["state"]["reported"]["estadoBomba"] = estadoBomba;

  if(modoObjeto == "automatico"){
    tierra = "siendo regada";
    outputDoc["state"]["reported"]["tierra"] = tierra;
  }
  else{
    lecturaSensorHumedad = analogRead(pinSensorHumedad);
    lecturaSensorAgua = analogRead(pinSensorTanqueAgua);
    nivelHumedad = map(lecturaSensorHumedad, 0, 4095, 0, 100);
    nivelTanqueAgua = map(lecturaSensorAgua, 0, 4095, 0, 100);

    if(nivelHumedad <= nivelHumedadBajaConfigurada){
      tierra = "seca";
    } else {
      tierra = "humeda";
    }

    if(nivelTanqueAgua < 5.00){
      tanque = "vacio";
    } else {
      tanque = "lleno";
    }

    outputDoc["state"]["reported"]["tierra"] = tierra;
    outputDoc["state"]["reported"]["tanque"] = tanque;
  }

  serializeJson(outputDoc, outputBuffer);
  mqttClient.publish(UPDATE_TOPIC, outputBuffer);

  Serial.print("Regando planta");
  int cont = 0;
  while(cont < 5){ // regando por 5 segundos
    Serial.print(".");
    cont = cont + 1;
    delay(1000);
  }

  Serial.println("Riego exitoso !");
  bomba.apagarBomba(); 

  estadoBomba = "apagada";
  outputDoc["state"]["reported"]["estadoBomba"] = estadoBomba;
  serializeJson(outputDoc, outputBuffer);
  mqttClient.publish(UPDATE_TOPIC, outputBuffer);
}

int estadoActual = 0;
int estadoAnterior = 0;

void reportInModo(){
  modoObjeto = inputDoc["state"]["modo"].as<String>();

  outputDoc["state"]["reported"]["modo"] = modoObjeto;

  serializeJson(outputDoc, outputBuffer);
  mqttClient.publish(UPDATE_TOPIC, outputBuffer);

  Serial.println("Atencion el modo del objeto es: ");Serial.println(modoObjeto);
}

void reportInBuiltInWaterPumpNEstadoBomba(){
  builtInWaterPump = inputDoc["state"]["builtInWaterPump"].as<int8_t>();

  Serial.println("El modo es: " + modoObjeto);

  if(builtInWaterPump == 1){
    regandoPlanta();
  } 
}

void callback(const char * topic, byte * payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) message += String((char) payload[i]);
  Serial.println("Message from topic " + String(topic) + ":" + message);
  DeserializationError err = deserializeJson(inputDoc, payload);
  if (!err) {
    if (String(topic) == SUBSCRIBE_TOPIC) {  
      if(inputDoc["state"].containsKey("modo")) {
        Serial.print("Modo obj: ");
        Serial.print(modoObjeto);
        reportInModo();
      } else if (inputDoc["state"].containsKey("builtInWaterPump")) {
        reportInBuiltInWaterPumpNEstadoBomba();
      } 
    }
  }
}

void connect() {
  // Bucle hasta que estemos conectados al servidor MQTT
  while (!mqttClient.connected()) {
    Serial.print("Intentando conectar al servidor MQTT...");
    
    // Intentar conectar
    if (mqttClient.connect(MQTT_CLIENT_ID)) {
      Serial.println("conectado !");
      delay(100);
      
      mqttClient.subscribe(SUBSCRIBE_TOPIC);
      
      Serial.println("Subscribed to " + String(SUBSCRIBE_TOPIC));
      
      delay(100);

    } else {
      Serial.print("Error de conexión, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" intentando de nuevo en 5 segundos...");
      // Esperar 5 segundos antes de intentar de nuevo
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Configurar el pin del sensor como entrada
  pinMode(pinSensorHumedad, INPUT);
  pinMode(pinSensorTanqueAgua, INPUT);

  //CONEXION AL WIFI ---------------------------------
  connectToWiFi();

  // Si la configuración es exitosa, se conecta a la red WiFi configurada
  Serial.println("Connected to WiFi!");


  // --------------------------------------------------
        
  // SETEANDO CERTIFICADOS XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  wiFiClient.setCACert(AMAZON_ROOT_CA1);
  wiFiClient.setCertificate(CERTIFICATE);
  wiFiClient.setPrivateKey(PRIVATE_KEY);
  // XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

  // CONECTANDO AL BROKER DE AWS WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW
  mqttClient.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
  mqttClient.setCallback(callback);
  
  Serial.print("Connecting to " + String(MQTT_BROKER_HOST));

  connect();

  reportBuiltInWaterPump(); 

  Serial.println("Objeto configurado!");
}

float temperatura = 0;

void reportEstadoSistema(String tierra, String nivelTanque, int estado){
  Serial.println("Reporte");
  outputDoc["state"]["reported"]["tierra"] = tierra;
  outputDoc["state"]["reported"]["tanque"] = nivelTanque;
  if(estado == 1){
    // Lectura de la temperatura y humedad del sensor
    float temperatura = dht.readTemperature();
    Serial.print("Hola");
    outputDoc["state"]["reported"]["temperatura"] = temperatura;
  }
  serializeJson(outputDoc, outputBuffer);
  mqttClient.publish(UPDATE_TOPIC, outputBuffer);
}

unsigned long previousPublishMillis = 0;

void loop() {
  if (mqttClient.connected()) {
    mqttClient.loop();

    if(modoObjeto == "manual"){
      Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      Serial.println("Atencion!, se activo el modo manual, el objeto dejara de leer datos de los sensores y se podra prender y apagar la bomba desde alexa."); 
      Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      delay(2000);
    } 
    else if (modoObjeto == "automatico")
    { 
      lecturaSensorHumedad = analogRead(pinSensorHumedad);
      lecturaSensorAgua = analogRead(pinSensorTanqueAgua);
      nivelHumedad = map(lecturaSensorHumedad, 0, 4095, 0, 100);
      nivelTanqueAgua = map(lecturaSensorAgua, 0, 4095, 0, 100);

      unsigned long now = millis();

      if (now - previousPublishMillis >= 8000) {
        previousPublishMillis = now;

        Serial.println("////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////");
        Serial.println("                                                   Modo automatico");
        Serial.println("////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////");

        if(nivelTanqueAgua >= 5.00 && nivelHumedad <= nivelHumedadBajaConfigurada){ estadoActual = 1; }     // tanque lleno, tierra seca
        else if(nivelTanqueAgua < 5.00 && nivelHumedad <= nivelHumedadBajaConfigurada){ estadoActual = 2; } // tanque vacio, tierra seca
        else if(nivelTanqueAgua >= 5.00 && nivelHumedad > nivelHumedadBajaConfigurada){ estadoActual = 3; } // tanque lleno, tierra humeda
        else if(nivelTanqueAgua < 5.00 && nivelHumedad > nivelHumedadBajaConfigurada){ estadoActual = 4; }  // tanque vacio, tierra humeda

        if(estadoActual != estadoAnterior  || estadoActual == 1){
          if(estadoActual == 1){ 
            Serial.print("tierra seca tanque lleno");
            // Deberias regar cuando: tanque lleno, y tierra seca FUNCION AUTOMATICA
            reportEstadoSistema("seca", "lleno", estadoActual); // estado 1
          }
          else if(estadoActual == 2){
            reportEstadoSistema("seca", "vacio", estadoActual); // estado 2
          }
          else if(estadoActual == 3){
            reportEstadoSistema("humeda", "lleno", estadoActual); // estado 3
          }
          else if(estadoActual == 4){
            reportEstadoSistema("humeda", "vacio", estadoActual); // estado 4
          }
          estadoAnterior = estadoActual;
        }


        Serial.println("*******************************************************");
        Serial.print("Estado Actual: ");Serial.println(estadoActual);
        Serial.print("Modo: ");Serial.println(modoObjeto);
        Serial.print("Humedad: "); Serial.print(nivelHumedad); Serial.println(" %");
        Serial.print("Nivel del tanque de agua: "); Serial.print(nivelTanqueAgua); Serial.println(" %");
        Serial.println("*******************************************************"); 
      }
    } 
  } 
  else 
  {
    Serial.println("-------------------------------------------------------------------------");
    Serial.println("                         ¡  MQTT no conectado  !");
    Serial.println("-------------------------------------------------------------------------");
    connect(); // reconexion a mqtt
  }
}