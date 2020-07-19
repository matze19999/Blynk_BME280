#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <WiFiUDP.h>
#include <FS.h>

Adafruit_BME280 bme;
BlynkTimer timer;
const float cToKOffset = 273.15;

const char* deviceName = "NodeMCU";
char auth[] = ""; # Blynk auth token
char ssid[] = "";
char pass[] = "";
char ip[] = ""; # IP of blynk server
int port = ; # Port of Blyn server

float absoluteHumidity(float temperature, float humidity) {
  return (13.2471*pow(EULER,17.67*temperature/(temperature+243.5))*humidity/(cToKOffset+temperature));
}

float saturationVaporPressure(float temperature) {

  if(temperature < 173 || temperature > 678) return -112;

  float svp = 0;
  if(temperature <= cToKOffset) {
    svp = exp(-5.8666426e3/temperature + 2.232870244e1 + (1.39387003e-2 + (-3.4262402e-5 + (2.7040955e-8*temperature)) * temperature) * temperature + 6.7063522e-1 * log(temperature));
  } else {

    const float th = temperature + -0.23855557567849 / (temperature - 0.65017534844798e3);
    const float a  = (th + 0.11670521452767e4) * th + -0.72421316703206e6;
    const float b  = (-0.17073846940092e2 * th + 0.12020824702470e5) * th + -0.32325550322333e7;
    const float c  = (0.14915108613530e2 * th + -0.48232657361591e4) * th + 0.40511340542057e6;

    svp = 2 * c / (-b + sqrt(b * b - 4 * a * c));
    svp *= svp;
    svp *= svp;
    svp *= 1e6;
    }

  yield();
  return svp;
}

float dewPoint(float temperature, float humidity) {
  temperature += cToKOffset; //Celsius to Kelvin

  if(humidity < 0 || humidity > 100) return -111;
  if(temperature < 173 || temperature > 678) return -112;

  humidity = humidity / 100 * saturationVaporPressure(temperature);

  byte mc = 10;

  float xNew;
  float dx;
  float z;

  do {
    dx = temperature / 1000;
    z = saturationVaporPressure(temperature);
    xNew = temperature + dx * (humidity - z) / (saturationVaporPressure(temperature + dx) - z);
    if (abs((xNew - temperature) / xNew) < 0.0001) {
        return xNew - cToKOffset;
    }
    temperature = xNew;
    mc--;
  } while(mc > 0);

  return -113;
}

void sensorDataSend() {
      float temperature = bme.readTemperature();
      float humidity_r = bme.readHumidity();
      float humidity = absoluteHumidity(temperature, humidity_r);
      float pressure = bme.readPressure() / 100.0F;
      float dew = dewPoint(temperature, humidity_r);

      Blynk.virtualWrite(V2, temperature);
      Blynk.virtualWrite(V3, humidity);
      Blynk.virtualWrite(V4, pressure);
      Blynk.virtualWrite(V5, dew);
}

BLYNK_WRITE(V1) {
  sensorDataSend();
}

void connect(){
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(2, LOW);
    WiFi.mode(WIFI_STA);
    WiFi.hostname("NodeMCU");
    WiFi.begin(ssid, pass);
    Serial.println("\nRun Setup and WIFI...");

    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }

    Serial.println();
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(2, HIGH);
    Serial.print("connected: ");
    Serial.println(WiFi.localIP());

    ArduinoOTA.setHostname(deviceName);
    delay(200);
    ArduinoOTA.begin();
    Serial.println("sys_ota_upgrade");

    ArduinoOTA.onStart([] () {
        Blynk.disconnect();
    });

    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd");
      ESP.restart();
    });

    Blynk.config(auth, ip, port);
    Serial.println("Run Blynk Setup...");
    Blynk.connect();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, OUTPUT);

  bme.begin(0x76);
  Serial.begin(115200);
  connect();
  timer.setInterval(5000L, sensorDataSend);
}


void loop() {

  if( WiFi.status() != WL_CONNECTED ) {
      connect();
      return;
  }

  ArduinoOTA.handle();
  Blynk.run();
  timer.run();
}
