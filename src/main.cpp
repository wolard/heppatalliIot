#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266Ping.h>

const char *ssid =	"ZyXEL";		// cannot be longer than 32 characters!
const char *pass =	"kopo2008";		//
WiFiEventHandler gotIpEventHandler, disconnectedEventHandler;
volatile long rpms;
long rpm;

const byte interruptPin = D4;

long measuretime=0;
 char rpmhar[5];

IPAddress server(192, 168, 1, 201);
long lastMsg = 0;
//char msg[50];
int fanvalue;
int value = 0;
float t;
float h;
float p;
long i;
long lastReconnectAttempt = 0;
Adafruit_BME280 bme1;
Adafruit_BME280 bme2;
WiFiClient wclient;
PubSubClient client(wclient);

struct linedata
{
    String measurement;
    String tag;
    String field;
};
IRAM_ATTR void blink() {

  rpms++;

}
boolean reconnect() {
  if (client.connect("arduinoClient")) {
    // Once connected, publish an announcement...
    client.publish("outTopic","hello world");
    // ... and resubscribe
    client.subscribe("inTopic");
  }
  return client.connected();
}


void reconnect2() {
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
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
   randomSeed(micros());
}
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }

}

void setup() {
 pinMode(interruptPin, INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(interruptPin), blink, RISING);

  gotIpEventHandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP& event)
  {
    Serial.print("Station connected, IP: ");
    Serial.println(WiFi.localIP());
  });


   disconnectedEventHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected& event)
  {
    Serial.println("Station disconnected");
  });
Serial.begin(9600);
delay(100);
 lastReconnectAttempt = 0;
 Serial.println();
  Serial.println();
 
  Serial.printf("Connecting to %s ...\n", ssid);
  WiFi.begin(ssid, pass);

//setup_wifi();
  client.setServer(server, 1883);
  client.setCallback(callback);
//Wire.setClockStretchLimit(2000);
 Wire.begin(D1, D2);
  Wire.setClock(100000);
  fanvalue=250;
 pinMode(D5,OUTPUT);
 analogWrite(D5,fanvalue);
    if (!bme1.begin(0x76, &Wire)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
     if (!bme2.begin(0x77, &Wire)){
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }

}

void loop() {
 if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
    Serial.println("mqtt connectiong");
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Client connected

    client.loop();
  }
i++;
Serial.println(i);

 float hud1 = bme1.readHumidity();
  // float pressure = 0;
  float h = hud1/100;
 float hud2 = bme2.readHumidity();
  float h2 = hud2/100;


 float temp1 = bme1.readTemperature();
    float t= temp1+273.15;
    float temp2 = bme2.readTemperature();
    float t2= temp2+273.15;

 p = bme1.readPressure();
  float p2 = bme2.readPressure();
 float abshum=(1320.65/t)*h*pow(10,(7.4475*(t-273.14)/(t-39.44)));
   float abshum2=(1320.65/t2)*h2*pow(10,(7.4475*(t2-273.14)/(t2-39.44)));
     Serial.print("abshub ");
    Serial.println(abshum-abshum2);
  String tempstring = String(temp1);
  String hudstring = String(hud1);
 String pressstring = String(p);
 String abshumidstring= String(abshum);

  String tempstring2 = String(temp2);
  String hudstring2 = String(hud2);
 String pressstring2 = String(p2);
 String abshumidstring2= String(abshum2);
 String rpmstring= String(rpm);
 

  //Serial.println(hum);
  Serial.println(t);
  Serial.println(p); 
  Serial.print("fanvalue");
  Serial.println(fanvalue);
  linedata linedatatemp;
  linedata linedatahumid;
  linedata linedatapressure;
  linedata linedataabshumid;

   linedata linedatatemp2;
  linedata linedatahumid2;
  linedata linedatapressure2;
  linedata linedataabshumid2;

 delay(1000);
 long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    


    char rpmchar[100];
     char tempchar[100];
     char hudchar[100];
       char presschar[100];
       char abshumchar[100];
         char tempchar2[100];
     char hudchar2[100];
       char presschar2[100];
       char abshumchar2[100];
    linedatatemp.measurement="intemp";
     linedatatemp.tag="temperature";
    linedatatemp.field=tempstring;
    linedatahumid.measurement="inhumid";
    linedatahumid.tag="Humidity";
    linedatahumid.field=hudstring;
    linedatapressure.measurement="inpressure";
    linedatapressure.tag="pressure";
    linedatapressure.field=pressstring;
    linedataabshumid.measurement="Absolute_Humidity_Inside";
    linedataabshumid.tag="Absolute_Humidity";
    linedataabshumid.field=abshumidstring;

     linedatatemp2.measurement="outtemp";
     linedatatemp2.tag="temperature";
    linedatatemp2.field=tempstring2;
    linedatahumid2.measurement="outhumid";
    linedatahumid2.tag="Humidity";
    linedatahumid2.field=hudstring2;
    linedatapressure2.measurement="outpressure";
    linedatapressure2.tag="pressure";
    linedatapressure2.field=pressstring2;
    linedataabshumid2.measurement="Absolute_Humidity_Outside";
    linedataabshumid2.tag="Absolute_Humidity";
    linedataabshumid2.field=abshumidstring2;
    
    String tempmsg=linedatatemp.measurement+",type="+ linedatatemp.tag+" value="+linedatatemp.field;
    String humidmsg=linedatahumid.measurement+",type="+ linedatahumid.tag+" value="+linedatahumid.field;
    String pressmsg=linedatapressure.measurement+",type="+ linedatapressure.tag+" value="+linedatapressure.field;
    String abshummsg=linedataabshumid.measurement+",type="+ linedataabshumid.tag+" value="+linedataabshumid.field;

     String tempmsg2=linedatatemp2.measurement+",type="+ linedatatemp2.tag+" value="+linedatatemp2.field;
    String humidmsg2=linedatahumid2.measurement+",type="+ linedatahumid2.tag+" value="+linedatahumid2.field;
    String pressmsg2=linedatapressure2.measurement+",type="+ linedatapressure2.tag+" value="+linedatapressure2.field;
    String abshummsg2=linedataabshumid2.measurement+",type="+ linedataabshumid2.tag+" value="+linedataabshumid2.field;
   // snprintf (msg, 50, temperature, value);
    tempmsg.toCharArray(tempchar,100);
     humidmsg.toCharArray(hudchar,100);
     pressmsg.toCharArray(presschar,100);
  abshummsg.toCharArray(abshumchar,100);

   tempmsg2.toCharArray(tempchar2,100);
     humidmsg2.toCharArray(hudchar2,100);
     pressmsg2.toCharArray(presschar2,100);
  abshummsg2.toCharArray(abshumchar2,100);
  rpmstring.toCharArray(rpmchar,100);
 char intemp[6];
 char inhud[6];
 char outtemp[6];
 char outhud[6];
 char press[10];
 
dtostrf(temp1, 1, 2, intemp);
dtostrf(hud1, 1, 2, inhud);
dtostrf(temp2, 1, 2, outtemp);
dtostrf(hud2, 1, 2, outhud);
dtostrf(p2, 1, 2, press);

    client.publish("heppatalli/temp/in", intemp);
    client.publish("heppatalli/humidity/in", inhud);
   // client.publish("heppatalli/pressure/in", presschar);
     client.publish("heppatalli/abshumid/in", abshumchar);

      client.publish("heppatalli/temp/out", outtemp);
    client.publish("heppatalli/humidity/out", outhud);
    client.publish("heppatalli/pressure/out", press);
     client.publish("heppatalli/abshumid/out", abshumchar2);
 
  Serial.print("rpm ");
  rpm=rpms*15;
  Serial.println(rpm);
  rpms=0;
  client.publish("heppatalli/fanrpm", rpmchar);
 
 
  }
if(temp1<3)
{
   Serial.print(temp1);
  Serial.println("low temp");
  fanvalue=250;
  analogWrite(D5,fanvalue);
}
if ((abshum-abshum2<1.5)&&(fanvalue<250))
{

 
Serial.println("fan slower");
fanvalue++;
analogWrite(D5, fanvalue);


}
if  ((abshum-abshum2>1.5)&&(fanvalue>0))
{
 Serial.println("fan faster");
  fanvalue--;
  analogWrite(D5, fanvalue);
}

  if (millis() - lastMsg > 5000)
  {
    lastMsg = millis();
      if(Ping.ping(server,1)) {
    Serial.println("Success!!");
  } else {
    Serial.println("Error :(");
     ESP.restart();
  }

  }
  // put your main code here, to run repeatedly:
}