#include <Arduino.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

const char *ssid =	"TP-LINK_BF33";		// cannot be longer than 32 characters!
const char *pass =	"82447410";		//
IPAddress server(192, 168, 0, 3);
long lastMsg = 0;
//char msg[50];
int value = 0;
float t;
float h;
float p;
Adafruit_BME280 bme1;
WiFiClient wclient;
PubSubClient client(wclient);
struct linedata
{
    String measurement;
    String tag;
    String field;
};




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
Serial.begin(9600);
delay(100);
 Serial.println();
  Serial.println();
setup_wifi();
  client.setServer(server, 1883);
  client.setCallback(callback);
Wire.setClockStretchLimit(2000);
 Wire.begin(D1, D2);
  Wire.setClock(100000);
 pinMode(D3,OUTPUT);
 if (!bme1.begin(0x76, &Wire)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1); 
    }

}

void loop() {
 if (!client.connected()) {
    reconnect();
  }
  client.loop();
  float hud1 = bme1.readHumidity();
  // float pressure = 0;
  float h = hud1/100;


 float temp1 = bme1.readTemperature();
    float t= temp1+273.15;

 p = bme1.readPressure();
 float hum=(1320.65/t)*h*pow(10,(7.4475*(t-273.14)/(t-39.44)));
  
  String tempstring = String(temp1);
  String hudstring = String(hud1);
 String pressstring = String(p);
 

  Serial.println(hum);
  Serial.println(t);
  Serial.println(p); 
  linedata linedatatemp;
  linedata linedatahumid;
  linedata linedatapressure;

 delay(1000);
 long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    ++value;
     char tempchar[100];
     char hudchar[100];
       char presschar[100];
    linedatatemp.measurement="intemp";
     linedatatemp.tag="temperature";
    linedatatemp.field=tempstring;
    linedatahumid.measurement="inhumid";
    linedatahumid.tag="Humidity";
    linedatahumid.field=hudstring;
    linedatapressure.measurement="inpressure";
    linedatapressure.tag="pressure";
    linedatapressure.field=pressstring;
    String tempmsg=linedatatemp.measurement+",type="+ linedatatemp.tag+" value="+linedatatemp.field;
    String humidmsg=linedatahumid.measurement+",type="+ linedatahumid.tag+" value="+linedatahumid.field;
    String pressmsg=linedatapressure.measurement+",type="+ linedatapressure.tag+" value="+linedatapressure.field;
   // snprintf (msg, 50, temperature, value);
    tempmsg.toCharArray(tempchar,100);
     humidmsg.toCharArray(hudchar,100);
     pressmsg.toCharArray(presschar,100);
  
  
    client.publish("heppatalli/temp/in", tempchar);
    client.publish("heppatalli/humidity/in", hudchar);
    client.publish("heppatalli/pressure/in", presschar);
  }
  // put your main code here, to run repeatedly:
}