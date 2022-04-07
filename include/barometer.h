
#define LED_ON LOW
#define LED_OFF HIGH
#define WIFI_CONNECTION_ATTEMPTS 15
#define VALID_SETTINGS_FLAG 0xDAB0
#define SSID_SIZE 100
#define PASSWORD_SIZE 50
#define ADDRESS_SIZE 30
#define USERNAME_SIZE 50

#define DEFAULT_ALTITUDE 23.86f //Lady Lake, FL
#define DEFAULT_SAMPLE_RATE 15 //seconds between samples

#define MQTT_CLIENTID_SIZE 25
#define DEFAULT_MQTT_BROKER_PORT 1883
#define MQTT_MAX_TOPIC_SIZE 50
#define MQTT_MAX_MESSAGE_SIZE 150
#define DEFAULT_MQTT_TOPIC_ROOT "esp8266/barometer/"
#define MQTT_CLIENT_ID_ROOT "barometer"
#define MQTT_TOPIC_RSSI "rssi"
#define MQTT_TOPIC_STATUS "status"
#define DEFAULT_MQTT_RUN_MESSAGE "started"
#define DEFAULT_MQTT_LWT_MESSAGE "Barometer Offline"
#define MQTT_TOPIC_COMMAND_REQUEST "command"
#define TEMPERATURE_TOPIC "temperature"
#define ABSOLUTE_PRESSURE_TOPIC "absolutePressure"
#define RELATIVE_PRESSURE_TOPIC "relativePressure"
#define COMPUTED_ALTITUDE_TOPIC "computedAltitude"
#define JSON_TOPIC "json"

//prototypes
void incomingMqttHandler(char*, byte*, unsigned int);
unsigned long myMillis();
char* fixup(char*, const char*, float);
char* fixup(char*, const char*, const char*);
bool processCommand(String);
void checkForCommand();
bool connectToWiFi();
void showSettings();
void reconnect(); 
void showSub(char*, bool);
void initializeSettings();
void loadSettings();
bool saveSettings();
void serialEvent(); 
void setup();
void barometerSetup(); 
void loop();
bool getValues();
