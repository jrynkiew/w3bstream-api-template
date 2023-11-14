#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <ArduinoMqttClient.h>
#include "secrets.h"
#include "src/w3bstream.pb.h"
#include "pb_encode.h"

//Enter your sensitive data in the secrets.h
int status                    = WL_IDLE_STATUS;   // the Wifi radio's status
char ssid[]                   = SECRET_SSID;      // your network SSID (name)
char pass[]                   = SECRET_PASS;      // your network password (use for WPA, or use as key for WEP)
const char broker[]           = "192.168.0.208";  // MQTT broker IP
int port                      = 1883;             // MQTT port
const char topic[]            = "eth_0x794f72d75ded3903dea907f420ad85b4813f6385_asd"; // w3bstream MQTT topic
const char* token             = "w3b_MV8xNjk3MjEzMzY1X2cyUj98STpkbWQnXA"; // w3bstream token
const char* payload           = "Hello World!";   // message to send to w3bstream
const char* event_type        = "DEFAULT";        // w3bstream event type
const char* pub_id            = "1";              // w3bstream publisher id
const char* event_id          = "1";              // w3bstream event id
unsigned int localPort        = 2390;             // local port to listen for UDP packets
const int NTP_PACKET_SIZE     = 48;               // NTP timestamp is in the first 48 bytes of the message
const long interval           = 10000;            // interval at which to send message (milliseconds)
unsigned long previousMillis  = 0;                // will store last time message was sent
const char username[]         = "admin";          // MQTT username
const char password[]         = "iotex.W3B.admin";// MQTT password

byte packetBuffer[NTP_PACKET_SIZE];               //buffer to hold incoming and outgoing packets
IPAddress timeServer(162, 159, 200, 123);         // pool.ntp.org NTP server
WiFiUDP Udp;                                      // A UDP instance to let us send and receive packets over UDP
WiFiClient wifiClient;                            // A WiFi instance to let connect to WiFi
MqttClient mqttClient(wifiClient);                // A MQTT Client instance to let us publish and subscribe to topics
uint8_t buffer[1024];                             // A buffer to hold the encoded protobuf message
pb_ostream_t stream;                              // A stream to write the encoded protobuf message to the buffer
Event message = Event_init_zero;                  // A protobuf message to send to the MQTT broker

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

int64_t getUnixTimestamp() {
  sendNTPpacket(timeServer); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);
  if (Udp.parsePacket()) {
    // We've received a packet, read the data from it
    Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
    //the timestamp starts at byte 40 of the received packet and is four bytes,
    //or two words, long. First, extract the two words:
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    // convert NTP time into Unix time. It starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // return Unix time:
    return epoch;
  }
  else {
    Serial.println("no NTP packet received");
    return 0;
  }
}

bool encode_string(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
    char *str = (char*)(*arg);
    
    if (!pb_encode_tag_for_field(stream, field))
        return false;
    
    return pb_encode_string(stream, (uint8_t*)str, strlen(str));
}

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  // Check if WiFi firmware version is the latest version
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 3 seconds for connection:
    delay(3000);
  }

  // you're connected now, so print out the data:
  Serial.println("Connected to WiFi");
  printWifiStatus();

  // Start UDP listener for NTP packets
  Udp.begin(localPort);

  // You can provide a username and password for authentication
  mqttClient.setUsernamePassword(username, password);

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  stream = pb_ostream_from_buffer(buffer, 1024);

  message.has_header = true;
  message.header.pub_time = getUnixTimestamp();

  message.header.event_type.arg = (void*)event_type;
  message.header.pub_id.arg = (void*)pub_id;
  message.header.token.arg = (void*)token;
  message.header.event_id.arg = (void*)event_id;
  message.payload.arg = (void*)payload;

  message.header.event_type.funcs.encode = &encode_string;
  message.header.pub_id.funcs.encode = &encode_string;
  message.header.token.funcs.encode = &encode_string;
  message.header.event_id.funcs.encode = &encode_string;
  message.payload.funcs.encode = &encode_string;
  
  pb_encode(&stream, Event_fields, &message);
}
 
void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    // save the last time a message was sent
    previousMillis = currentMillis;

    mqttClient.poll();
    publishProtobufToMQTT();
  }
}

void publishProtobufToMQTT() {
  // Publish it to the MQTT broker using the Print interface
  Serial.print("Raw message is:\t");
  Serial.println((char*)buffer);

  mqttClient.beginMessage(topic);
  mqttClient.print((char*)buffer);
  mqttClient.endMessage();

  Serial.println();
}