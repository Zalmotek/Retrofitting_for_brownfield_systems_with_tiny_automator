#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads1;

float Voltage = 0.0;

//////////temp sensor ////////
int decimalPrecision = 2;               // decimal places for all values shown in LED Display & Serial Monitor


/* 1- Temperature Measurement */

//int ThermistorPin = A5;                 // The analog input pin for measuring temperature
float voltageDividerR1 = 5100;         // Resistor value in R1 for voltage devider method
float BValue = 3977;                    // The B Value of the thermistor for the temperature measuring range
float R1 = 10000;                        // Thermistor resistor rating at based temperature (25 degree celcius)
float t1 = 298.15;                      /* Base temperature t1 in Kelvin (default should be at 25 degree)*/
float R2 ;                              /* Resistance of Thermistor (in ohm) at Measuring Temperature*/
float t2, Temperature ;                             /* Measurement temperature t2 in Kelvin */

float a ;                               /* Use for calculation in Temperature*/
float b ;                               /* Use for calculation in Temperature*/
float c ;                               /* Use for calculation in Temperature*/
float d ;                               /* Use for calculation in Temperature*/
float e = 2.718281828 ;                 /* the value of e use for calculation in Temperature*/

float tempSampleRead  = 0;               /* to read the value of a sample including currentOffset1 value*/
float tempLastSample  = 0;               /* to count time for each sample. Technically 1 milli second 1 sample is taken */
float tempSampleSum   = 0;               /* accumulation of sample readings */
float tempSampleCount = 0;               /* to count number of sample. */
float tempMean ;                         /* to calculate the average value from all samples, in analog values*/

//////////

const char* ssid = "WifiSSid";
const char* password =  "WifiPwd";
const char* mqttServer = "IpAddressMqttTinyAutomator";
const int mqttPort = 1883;
#define MQTT_PUB_TEMP "test-topic"
//const char* mqttUser = "yourInstanceUsername";
//const char* mqttPassword = "yourInstancePassword";

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  ads1.setGain(GAIN_ONE);
  ads1.begin(0x48, &Wire);

  Wire.begin(16, 17);
  Serial.begin(115200);
  Serial.println();

  WiFi.begin(ssid, password);
  Serial.println("Connecting to WIFI…");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("After 10 seconds the first reading will be displayed");
  client.setServer(mqttServer, mqttPort);
  //
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");

    if (client.connect("ESP32Client"/*, mqttUser, mqttPassword */)) {

      Serial.println("connected");

    } else {

      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);

    }
  }

}

void loop() {
for(int i=0;i<99;i++){
  int16_t adc1, ts;
  float volts0, tm;
   if (millis() >= tempLastSample + 1)  {
    tm = ads1.readADC_SingleEnded(0);
    ts = map(tm, 0, 16000, 0, 1023);
    //Serial.print(tm);
    // Serial.println(ts);
    tempSampleRead = ts;                                                                         /* read analog value from sensor */
    tempSampleSum = tempSampleSum + tempSampleRead;                                             /* add all analog value for averaging later*/
    tempSampleCount = tempSampleCount + 1;                                                      /* keep counting the sample quantity*/
    tempLastSample = millis();                                                                  /* reset the time in order to repeat the loop again*/
   }

  if (tempSampleCount == 100)                                                                        /* after 1000 sample readings taken*/
  {
    tempMean = tempSampleSum / tempSampleCount;                                                 /* find the average analog value from those data*/
    R2 = (voltageDividerR1 * tempMean) / (1023 - tempMean);                                     /* convert the average analog value to resistance value*/
    a = 1 / t1;                                                                                 /* use for calculation */
    b = log10(R1 / R2);                                                                         /* use for calculation */
    c = b / log10(e);                                                                           /* use for calculation */
    d = c / BValue ;                                                                            /* use for calculation */
    t2 = 1 / (a - d);                                                                           /* the measured temperature value based on calculation (in Kelvin) */
    Serial.print(t2 - 298.15, decimalPrecision);                                                /* display in Serial monitor the temperature in Celcius*/
    Serial.println(" °C");
    Temperature = t2 - 298.15;
  
    tempSampleSum = 0;                                                                          /* reset all the total analog value back to 0 for the next count */
    tempSampleCount = 0;                                                                        /* reset the total number of samples taken back to 0 for the next count*/
  }
  delay(10);
}
  char buffer1[256];
  StaticJsonDocument<300> doc;
  //JsonObject& doc = doc.createObject();

  doc["device"] = "ESP32";
  doc["sensorType"] = "Temperature";
  doc["value"] = Temperature;

  char JSONmessageBuffer[100];
  serializeJson(doc, Serial);
  serializeJson(doc, buffer1);
  Serial.println("Sending message to MQTT topic..");
  Serial.println(JSONmessageBuffer);

  if (client.publish(MQTT_PUB_TEMP, buffer1) == true) {
    Serial.println("Success sending message");
  } else {
    Serial.println("Error sending message");
  }

  client.loop();
  Serial.println("-------------");


  delay(10000);

}
