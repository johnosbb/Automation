#define BLYNK_TEMPLATE_ID "TMPL4SKGygDVT"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "wnE8HvPlM2KhuS6Xw6u_VH4DwydxOgwV"



// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "bregenz_hb_2.4GHz";
char pass[] = "aa81990034ae";

/******* MQTT Broker Connection Details *******/
const char* mqtt_server = "192.168.1.151";
const char* mqtt_username = "Mqttuser";
const char* mqtt_password = "Mqtt3738";
const int mqtt_server_port =1883;

IPAddress device_ip  (192, 168,   1, 205);
IPAddress dns_ip_1    (  8,   8,   8,   8);
IPAddress dns_ip_2    (  4,   4,   4,   4 );
IPAddress gateway_ip (192, 168,   1,   254);
IPAddress subnet_mask(255, 255, 255,   0);