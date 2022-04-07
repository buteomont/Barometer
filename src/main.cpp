/*
 * Program to report the barometric pressure and temperature via MQTT.
 * By David E. Powell 
 *
 * Configuration is done via serial port or mqtt topic "command". 
 * Program updates can be done OTA.
 * 
 * 
 * **** to erase the entire flash chip in PlatformIO, open
 * **** a terminal and type "pio run -t erase"
 */ 
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <pgmspace.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "SFE_BMP180.h"
#include <Wire.h>

#include "barometer.h"

char *stack_start;// initial stack size
SFE_BMP180 pressure;// Create an SFE_BMP180 object

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// These are the settings that get stored in EEPROM.  They are all in one struct which
// makes it easier to store and retrieve.
typedef struct 
  {
  unsigned int validConfig=0; 
  char ssid[SSID_SIZE] = "";
  char wifiPassword[PASSWORD_SIZE] = "";
  char brokerAddress[ADDRESS_SIZE]="";
  int brokerPort=DEFAULT_MQTT_BROKER_PORT;
  char mqttUsername[USERNAME_SIZE]="";
  char mqttUserPassword[PASSWORD_SIZE]="";
  char mqttTopicRoot[MQTT_MAX_TOPIC_SIZE]="";
  char mqttRunMessage[MQTT_MAX_MESSAGE_SIZE]="";
  char mqttTimeoutMessage[MQTT_MAX_MESSAGE_SIZE]="";
  char mqttLWTMessage[MQTT_MAX_MESSAGE_SIZE]="";
  float altitude=DEFAULT_ALTITUDE; // Altitude in meters
  int sampleRate=DEFAULT_SAMPLE_RATE;
  boolean debug=false;
  char mqttClientId[MQTT_CLIENTID_SIZE]=""; //will be the same across reboots
  } conf;

  typedef struct
    {
    float temperature; //centigrade
    float absPressure; //millibars
    float relPressure; //millibars
    float compElevation;//meters
    } values;

conf settings; //all settings in one struct makes it easier to store in EEPROM
boolean settingsAreValid=false;

String commandString = "";     // a String to hold incoming commands from serial
bool commandComplete = false;  // goes true when enter is pressed

unsigned long timeoutCount=0; //milliseconds
boolean timeoutMessageSent=false;

double temperature, absolutePressure, relativePressure,computedAltitude;
char json[180];

void printStackSize(char id)
  {
  char stack;
  Serial.print(id);
  Serial.print (F(": stack size "));
  Serial.println (stack_start - &stack);
  }

char* fixup(char* rawString, const char* field, float value)
  {
  char strVal[10];
  sprintf(strVal,"%.2f",value);
  return fixup(rawString, field, strVal);
  }

char* fixup(char* rawString, const char* field, const char* value)
  {
  String rs=String(rawString);
  rs.replace(field,String(value));
  strcpy(rawString,rs.c_str());
  if (settings.debug)
    printStackSize('F');
  return rawString;
  }

/************************
 * Do the MQTT thing
 ************************/

boolean publish(char* topic, const char* reading, boolean retain)
  {
  Serial.print(topic);
  Serial.print(" ");
  Serial.println(reading);
  return mqttClient.publish(topic,reading,retain); 
  }

/**
 * Handler for incoming MQTT messages.  The payload is the command to perform. 
 * The MQTT message topic sent is the topic root plus the command.
 * Implemented commands are: 
 * MQTT_PAYLOAD_SETTINGS_COMMAND: sends a JSON payload of all user-specified settings
 * MQTT_PAYLOAD_REBOOT_COMMAND: Reboot the controller
 * MQTT_PAYLOAD_VERSION_COMMAND Show the version number
 * MQTT_PAYLOAD_STATUS_COMMAND Show the most recent flow values
 */
void incomingMqttHandler(char* reqTopic, byte* payload, unsigned int length) 
  {
  if (settings.debug)
    {
    Serial.println("====================================> Callback works.");
    }
  payload[length]='\0'; //this should have been done in the caller code, shouldn't have to do it here
  char charbuf[100];
  sprintf(charbuf,"%s",payload);
  const char* response;
  char settingsResp[400];
  char tempbuf[10];

  if (strcmp(charbuf,"settings")==0) //special case, send all settings
    {
    strcpy(settingsResp,"\nssid=");
    strcat(settingsResp,settings.ssid);
    strcat(settingsResp,"\n");
    strcat(settingsResp,"wifipass=");
    strcat(settingsResp,settings.wifiPassword);
    strcat(settingsResp,"\n");
    strcat(settingsResp,"broker=");
    strcat(settingsResp,settings.brokerAddress);
    strcat(settingsResp,"\n");
    strcat(settingsResp,"brokerPort=");
    strcat(settingsResp,String(settings.brokerPort).c_str());
    strcat(settingsResp,"\n");
    strcat(settingsResp,"userName=");
    strcat(settingsResp,settings.mqttUsername);
    strcat(settingsResp,"\n");
    strcat(settingsResp,"userPass=");
    strcat(settingsResp,settings.mqttUserPassword);
    strcat(settingsResp,"\n");
    strcat(settingsResp,"topicRoot=");
    strcat(settingsResp,settings.mqttTopicRoot);
    strcat(settingsResp,"\n");
    strcat(settingsResp,"runMessage=");
    strcat(settingsResp,settings.mqttRunMessage);
    strcat(settingsResp,"\n");
    strcat(settingsResp,"lwtMessage=");
    strcat(settingsResp,settings.mqttLWTMessage);
    strcat(settingsResp,"\n");
    strcat(settingsResp,"timeoutMessage=");
    strcat(settingsResp,settings.mqttTimeoutMessage);
    strcat(settingsResp,"\n");
    strcat(settingsResp,"altitude=");
    sprintf(tempbuf,"%.2f",settings.altitude);
    strcat(settingsResp,tempbuf);
    strcat(settingsResp,"\n");
    strcat(settingsResp,"sampleRate=");
    strcat(settingsResp,String(settings.sampleRate).c_str());
    strcat(settingsResp,"\n");
    strcat(settingsResp,"debug=");
    strcat(settingsResp,settings.debug?"true":"false");
    strcat(settingsResp,"\n");
    strcat(settingsResp,"MQTT client ID=");
    strcat(settingsResp,settings.mqttClientId);
    strcat(settingsResp,"\n");
    strcat(settingsResp,"IP Address=");
    strcat(settingsResp,WiFi.localIP().toString().c_str());
    response=settingsResp;
    }
  else if (processCommand(charbuf))
    {
    response="OK";
    }
  else
    {
    char badCmd[18];
    strcpy(badCmd,"(empty)");
    response=badCmd;
    }

  //prepare the response topic
  char topic[MQTT_MAX_TOPIC_SIZE];
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,charbuf); //the incoming command becomes the topic suffix

  if (!publish(topic,response,false)) //do not retain
    Serial.println("************ Failure when publishing status response!");
  }

boolean sendMessage(char* topic, char* value)
  { 
  boolean success=false;
  if (!mqttClient.connected())
    {
    Serial.println("Not connected to MQTT broker!");
    }
  else
    {
    char topicBuf[MQTT_MAX_TOPIC_SIZE+MQTT_MAX_MESSAGE_SIZE];
    
    //publish the message
    strcpy(topicBuf,settings.mqttTopicRoot);
    strcat(topicBuf,topic);
    success=publish(topicBuf,value,true); //retain
    if (!success)
      Serial.println("************ Failed publishing "+String(topic)+"! ("+String(success)+")");
    }
  return success;
  }

void otaSetup()
  {
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() 
    {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
    });

    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
  }

void setup() 
  {
  //init record of stack
  char stack;
  stack_start = &stack;  
  
  Serial.begin(115200);
  Serial.setTimeout(10000);
  Serial.println();
  
  while (!Serial); // wait here for serial port to connect.
  Serial.println(F("Running."));

  barometerSetup(); //Initialize the BMP180

  pinMode(LED_BUILTIN,OUTPUT);// The blue light on the board shows WiFi activity
  digitalWrite(LED_BUILTIN,LED_OFF);

  EEPROM.begin(sizeof(settings)); //fire up the eeprom section of flash
  commandString.reserve(200); // reserve 200 bytes of serial buffer space for incoming command string

  if (settings.debug)
    Serial.println(F("Loading settings"));
  loadSettings(); //set the values from eeprom

  Serial.print("Performing settings sanity check...");
  if ((settings.validConfig!=0 && 
      settings.validConfig!=VALID_SETTINGS_FLAG) || //should always be one or the other
      settings.brokerPort<0 ||
      settings.brokerPort>65535)
    {
    Serial.println("\nSettings in eeprom failed sanity check, initializing.");
    initializeSettings(); //must be a new board or flash was erased
    }
  else
    Serial.println("passed.");

  if (settings.debug)
    Serial.println(F("Connecting to WiFi"));
  
  if (settings.validConfig==VALID_SETTINGS_FLAG)
    connectToWiFi(); //connect to the wifi
  
  if (WiFi.status() == WL_CONNECTED)
    {
    sendMessage(MQTT_TOPIC_STATUS, settings.mqttRunMessage); //running!
    otaSetup(); //initialize the OTA stuff
    }
  }

void loop()
  {
  checkForCommand(); // Check for input in case something needs to be changed to work
  connectToWiFi(); //make sure we're connected
  if (WiFi.status() == WL_CONNECTED)
    {
    mqttClient.loop(); //This has to happen every so often or we get disconnected for some reason

    ArduinoOTA.handle(); //Check for new version

    //see if it's time for a sample
    static unsigned long nextSample=millis()+settings.sampleRate*1000;
    if (millis()>nextSample)
      {
      bool ok=getValues();
      if (ok)
        {
        char buf[10];
        sprintf(buf,"%.2f",temperature);
        sendMessage(TEMPERATURE_TOPIC,buf);
        sprintf(buf,"%.2f",absolutePressure);
        sendMessage(ABSOLUTE_PRESSURE_TOPIC,buf);
        sprintf(buf,"%.2f",relativePressure);
        sendMessage(RELATIVE_PRESSURE_TOPIC,buf);
        sprintf(buf,"%.2f",computedAltitude);
        sendMessage(COMPUTED_ALTITUDE_TOPIC,buf);
        sprintf(buf,"%d",WiFi.RSSI());
        sendMessage(MQTT_TOPIC_RSSI,buf);
        sendMessage(JSON_TOPIC,json);
        }
      nextSample=millis()+settings.sampleRate*1000; //next sample time
      }
    }
  }


/*
 * If not connected to wifi, connect.
 */
boolean connectToWiFi()
  {
  yield();
  static boolean retval=true; //assume connection to wifi is ok
  if (WiFi.status() != WL_CONNECTED)
    {
    if (settings.debug)
      {
      Serial.print(F("Attempting to connect to WPA SSID \""));
      Serial.print(settings.ssid);
      Serial.print("\" with passphrase \"");
      Serial.print(settings.wifiPassword);
      Serial.println("\"");
      }

    WiFi.mode(WIFI_STA); //station mode, we are only a client in the wifi world
    WiFi.begin(settings.ssid, settings.wifiPassword);

    //try for a few seconds to connect to wifi
    for (int i=0;i<WIFI_CONNECTION_ATTEMPTS;i++)  
      {
      if (WiFi.status() == WL_CONNECTED)
        {
        digitalWrite(LED_BUILTIN,LED_ON); //show we're connected
        break;  // got it
        }
      if (settings.debug)
        Serial.print(".");
      checkForCommand(); // Check for input in case something needs to be changed to work
      ESP.wdtFeed(); //feed the watchdog timers.
      delay(500);
      }

    if (WiFi.status() == WL_CONNECTED)
      {
      digitalWrite(LED_BUILTIN,LED_ON); //show we're connected
      if (settings.debug)
        {
        Serial.println(F("Connected to network."));
        Serial.println();
        }
      //show the IP address
      Serial.println(WiFi.localIP());
      retval=true;
      }     
    else //can't connect to wifi, try again next time
      {
      retval=false;
      Serial.print("Wifi status is ");
      Serial.println(WiFi.status());
      Serial.println(F("WiFi connection unsuccessful."));
      digitalWrite(LED_BUILTIN,LED_OFF); //stay off until we connect
      }
    }
  if (WiFi.status() == WL_CONNECTED)
    {
    reconnect(); // go ahead and connect to the MQTT broker
    }
  return retval;
  }

void showSub(char* topic, bool subgood)
  {
  if (settings.debug)
    {
    Serial.print("++++++Subscribing to ");
    Serial.print(topic);
    Serial.print(":");
    Serial.println(subgood);
    }
  }


/*
 * Reconnect to the MQTT broker
 */
void reconnect() 
  {
  // Loop until we're reconnected
  if (!mqttClient.connected()) 
    {      
    Serial.print("Attempting MQTT connection...");

    mqttClient.setBufferSize(500); //default (256) isn't big enough
    mqttClient.setServer(settings.brokerAddress, settings.brokerPort);
    mqttClient.setCallback(incomingMqttHandler);
    
    // Attempt to connect
    char willTopic[MQTT_MAX_TOPIC_SIZE]="";
    strcpy(willTopic,settings.mqttTopicRoot);
    strcat(willTopic,MQTT_TOPIC_STATUS);


    if (mqttClient.connect(settings.mqttClientId,
                          settings.mqttUsername,
                          settings.mqttUserPassword,
                          willTopic,
                          0,                  //QOS
                          true,               //retain
                          settings.mqttLWTMessage))
      {
      Serial.println("connected to MQTT broker.");

      //resubscribe to the incoming message topic
      char topic[MQTT_MAX_TOPIC_SIZE];
      strcpy(topic,settings.mqttTopicRoot);
      strcat(topic,MQTT_TOPIC_COMMAND_REQUEST);
      bool subgood=mqttClient.subscribe(topic);
      showSub(topic,subgood);
      }
    else 
      {
      Serial.print("failed, rc=");
      Serial.println(mqttClient.state());
      Serial.println("Will try again in a second");
      
      // Wait a second before retrying
      // In the meantime check for input in case something needs to be changed to make it work
      checkForCommand(); 
      
      delay(1000);
      }
    }
  mqttClient.loop(); //This has to happen every so often or we get disconnected for some reason
  }

//Generate an MQTT client ID.  This should not be necessary very often
char* generateMqttClientId(char* mqttId)
  {
  strcpy(mqttId,strcat(MQTT_CLIENT_ID_ROOT,String(random(0xffff), HEX).c_str()));
  if (settings.debug)
    {
    Serial.print("New MQTT userid is ");
    Serial.println(mqttId);
    }
  return mqttId;
  }

void showSettings()
  {
  Serial.print("ssid=<wifi ssid> (");
  Serial.print(settings.ssid);
  Serial.println(")");
  Serial.print("wifipass=<wifi password> (");
  Serial.print(settings.wifiPassword);
  Serial.println(")");
  Serial.print("broker=<address of MQTT broker> (");
  Serial.print(settings.brokerAddress);
  Serial.println(")");
  Serial.print("brokerPort=<port number MQTT broker> (");
  Serial.print(settings.brokerPort);
  Serial.println(")");
  Serial.print("userName=<user ID for MQTT broker> (");
  Serial.print(settings.mqttUsername);
  Serial.println(")");
  Serial.print("userPass=<user password for MQTT broker> (");
  Serial.print(settings.mqttUserPassword);
  Serial.println(")");
  Serial.print("topicRoot=<MQTT topic base to which status or other topics will be added> (");
  Serial.print(settings.mqttTopicRoot);
  Serial.println(")");
  Serial.print("runMessage=<status message to send when power is applied> (");
  Serial.print(settings.mqttRunMessage);
  Serial.println(")");
  Serial.print("lwtMessage=<status message to send when power is removed> (");
  Serial.print(settings.mqttLWTMessage);
  Serial.println(")");
  Serial.print("timeoutMessage=<status message to send when runtime is exceeded> (");
  Serial.print(settings.mqttTimeoutMessage);
  Serial.println(")");
  Serial.print("altitude=<ground elevation in meters> (");
  Serial.print(settings.altitude);
  Serial.println(")");
  Serial.print("sampleRate=<seconds between samples> (");
  Serial.print(settings.sampleRate);
  Serial.println(")");
  Serial.print("debug=<print debug messages to serial port> (");
  Serial.print(settings.debug?"true":"false");
  Serial.println(")");
  Serial.print("MQTT client ID=<automatically generated client ID> (");
  Serial.print(settings.mqttClientId);
  Serial.println(") **Use \"resetmqttid=yes\" to regenerate");
  Serial.println("\n*** Use \"factorydefaults=yes\" to reset all settings ***");
  Serial.print("\nIP Address=");
  Serial.println(WiFi.localIP());
  }

/*
 * Check for configuration input via the serial port.  Return a null string 
 * if no input is available or return the complete line otherwise.
 */
String getConfigCommand()
  {
  if (commandComplete) 
    {
    String newCommand=commandString;

    commandString = "";
    commandComplete = false;
    return newCommand;
    }
  else return "";
  }

bool processCommand(String cmd)
  {
  const char *str=cmd.c_str();
  char *val=NULL;
  char *nme=strtok((char *)str,"=");
  if (nme!=NULL)
    val=strtok(NULL,"=");

  //Get rid of the carriage return
  if (val!=NULL && strlen(val)>0 && val[strlen(val)-1]==13)
    val[strlen(val)-1]=0; 

  if (nme==NULL || val==NULL || strlen(nme)==0 || strlen(val)==0)
    {
    showSettings();
    return false;   //not a valid command, or it's missing
    }
  else if (strcmp(nme,"ssid")==0)
    {
    strcpy(settings.ssid,val);
    saveSettings();
    }
  else if (strcmp(nme,"wifipass")==0)
    {
    strcpy(settings.wifiPassword,val);
    saveSettings();
    }
  else if (strcmp(nme,"broker")==0)
    {
    strcpy(settings.brokerAddress,val);
    saveSettings();
    }
  else if (strcmp(nme,"brokerPort")==0)
    {
    settings.brokerPort=atoi(val);
    saveSettings();
    }
  else if (strcmp(nme,"userName")==0)
    {
    strcpy(settings.mqttUsername,val);
    saveSettings();
    }
  else if (strcmp(nme,"userPass")==0)
    {
    strcpy(settings.mqttUserPassword,val);
    saveSettings();
    }
  else if (strcmp(nme,"lwtMessage")==0)
    {
    strcpy(settings.mqttLWTMessage,val);
    saveSettings();
    }
  else if (strcmp(nme,"runMessage")==0)
    {
    strcpy(settings.mqttRunMessage,val);
    saveSettings();
    }
  else if (strcmp(nme,"timeoutMessage")==0)
    {
    strcpy(settings.mqttTimeoutMessage,val);
    saveSettings();
    }
  else if (strcmp(nme,"topicRoot")==0)
    {
    strcpy(settings.mqttTopicRoot,val);
    saveSettings();
    }
  else if (strcmp(nme,"altitude")==0)
    {
    settings.altitude=(float)atof(val);
    saveSettings();
    }
  else if (strcmp(nme,"sampleRate")==0)
    {
    settings.sampleRate=atoi(val);
    saveSettings();
    }
  else if ((strcmp(nme,"resetmqttid")==0)&& (strcmp(val,"yes")==0))
    {
    generateMqttClientId(settings.mqttClientId);
    saveSettings();
    }
  else if (strcmp(nme,"debug")==0)
    {
    settings.debug=strcmp(val,"false")==0?false:true;
    saveSettings();
    }
  else if ((strcmp(nme,"factorydefaults")==0) && (strcmp(val,"yes")==0)) //reset all eeprom settings
    {
    Serial.println("\n*********************** Resetting EEPROM Values ************************");
    initializeSettings();
    saveSettings();
    delay(2000);
    ESP.restart();
    }
  else if ((strcmp(nme,"reset")==0) && (strcmp(val,"yes")==0)) //reset the device
    {
    Serial.println("\n*********************** Resetting Device ************************");
    delay(1000);
    ESP.restart();
    }
  else
    {
    showSettings();
    return false; //command not found
    }
  return true;
  }

void initializeSettings()
  {
  settings.validConfig=0; 
  strcpy(settings.ssid,"");
  strcpy(settings.wifiPassword,"");
  settings.altitude=DEFAULT_ALTITUDE;
  settings.sampleRate=DEFAULT_SAMPLE_RATE;
  strcpy(settings.brokerAddress,"");
  settings.brokerPort=DEFAULT_MQTT_BROKER_PORT;
  strcpy(settings.mqttLWTMessage,DEFAULT_MQTT_LWT_MESSAGE);
  strcpy(settings.mqttRunMessage,DEFAULT_MQTT_RUN_MESSAGE);
  strcpy(settings.mqttTopicRoot,DEFAULT_MQTT_TOPIC_ROOT);
  strcpy(settings.mqttUsername,"");
  strcpy(settings.mqttUserPassword,"");
  generateMqttClientId(settings.mqttClientId);
  settings.debug=false;
  saveSettings();
  }

void checkForCommand()
  {
  if (Serial.available())
    {
    serialEvent();
    String cmd=getConfigCommand();
    if (cmd.length()>0)
      {
      processCommand(cmd);
      }
    }
  }
  
/*
*  Initialize the settings from eeprom and determine if they are valid
*/
void loadSettings()
  {
  EEPROM.get(0,settings);
  if (settings.validConfig==VALID_SETTINGS_FLAG)    //skip loading stuff if it's never been written
    {
    settingsAreValid=true;
    if (settings.debug)
      Serial.println("Loaded configuration values from EEPROM");
    }
  else
    {
    Serial.println("Skipping load from EEPROM, device not configured.");    
    settingsAreValid=false;
    }
  }

/*
 * Save the settings to EEPROM. Set the valid flag if everything is filled in.
 */
boolean saveSettings()
  {
  if (strlen(settings.ssid)>0 &&
    strlen(settings.ssid)<=SSID_SIZE &&
    strlen(settings.wifiPassword)>0 &&
    strlen(settings.wifiPassword)<=PASSWORD_SIZE &&
    strlen(settings.brokerAddress)>0 &&
    strlen(settings.brokerAddress)<ADDRESS_SIZE &&
    strlen(settings.mqttLWTMessage)>0 &&
    strlen(settings.mqttLWTMessage)<MQTT_MAX_MESSAGE_SIZE &&
    strlen(settings.mqttRunMessage)>0 &&
    strlen(settings.mqttRunMessage)<MQTT_MAX_MESSAGE_SIZE &&
    strlen(settings.mqttTimeoutMessage)>0 &&
    strlen(settings.mqttTimeoutMessage)<MQTT_MAX_MESSAGE_SIZE &&
    strlen(settings.mqttTopicRoot)>0 &&
    strlen(settings.mqttTopicRoot)<MQTT_MAX_TOPIC_SIZE &&
    settings.brokerPort>0 &&
    settings.brokerPort<65535 &&
    settings.altitude>0 &&
    settings.sampleRate>0)
    {
    Serial.println("Settings deemed complete");
    settings.validConfig=VALID_SETTINGS_FLAG;
    settingsAreValid=true;
    }
  else
    {
    Serial.println("Settings still incomplete");
    settings.validConfig=0;
    settingsAreValid=false;
    }

  //The mqttClientId is not set by the user, but we need to make sure it's set  
  if (strlen(settings.mqttClientId)==0)
    {
    generateMqttClientId(settings.mqttClientId);
    }

  EEPROM.put(0,settings);
  return EEPROM.commit();
  }

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() 
  {
  while (Serial.available()) 
    {
    // get the new byte
    char inChar = (char)Serial.read();
    Serial.print(inChar);

    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it 
    if (inChar == '\n') 
      {
      commandComplete = true;
      }
    else
      {
      // add it to the inputString 
      commandString += inChar;
      }
    }
  }

/* 
Like most pressure sensors, the BMP180 measures absolute pressure.
This is the actual ambient pressure seen by the device, which will
vary with both altitude and weather.

Before taking a pressure reading you must take a temperature reading.
This is done with startTemperature() and getTemperature().
The result is in degrees C.

Once you have a temperature reading, you can take a pressure reading.
This is done with startPressure() and getPressure().
The result is in millibar (mb) aka hectopascals (hPa).

If you'll be monitoring weather patterns, you will probably want to
remove the effects of altitude. This will produce readings that can
be compared to the published pressure readings from other locations.
To do this, use the sealevel() function. You will need to provide
the known altitude at which the pressure was measured.

If you want to measure altitude, you will need to know the pressure
at a baseline altitude. This can be average sealevel pressure, or
a previous pressure reading at your altitude, in which case
subsequent altitude readings will be + or - the initial baseline.
This is done with the altitude() function.

The SFE_BMP180 library uses floating-point equations developed by the
Weather Station Data Logger project: http://wmrx00.sourceforge.net/
*/

void barometerSetup()
  {
  // Initialize the sensor (it is important to get calibration values stored on the device).

  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
    {
    // Oops, something went wrong, this is usually a connection problem
    Serial.println("BMP180 init fail\n\n");
    Serial.println("Rebooting in 10 seconds.");
    delay(10000); //wait 10 seconds
    ESP.reset(); //reboot
    }
  }

bool getValues()
  {
  bool ok=true;
  char status;
  double T,P,p0=0.0,a=0.0;
  static int failureCount=0;

  // Get pressure readings.

  // For sea-level-compensated pressure, as used in weather reports,
  // we will need to know the altitude at which the measurements are taken.
  // We're using a constant called settings.altitude in this sketch.
  // To measure altitude instead of pressure, we need
  // to provide a known baseline pressure. 
  
  Serial.println();
  Serial.print("provided altitude: ");
  Serial.print(settings.altitude,2);
  Serial.print(" meters, ");
  Serial.print(settings.altitude*3.28084,2);
  Serial.println(" feet");
  

  // We must get a temperature measurement before getting a pressure reading.
  // Start a temperature measurement. The call will return the number of ms to wait
  // if it is successful, and 0 if it is not.
  status = pressure.startTemperature();
  if (status != 0)
    {
    delay(status);// Wait for the measurement to complete

    // Retrieve the completed temperature measurement and
    // store it in the variable T.
    // Function returns 1 if successful, 0 if failure.
    status = pressure.getTemperature(T);
    if (status != 0)
      {
      temperature=T; //global access

      // Print out the measurement:
      Serial.print("temperature: ");
      Serial.print(T,2);
      Serial.print(" deg C, ");
      Serial.print((9.0/5.0)*T+32.0,2);
      Serial.println(" deg F");
      
      // Start a pressure measurement.
      // The parameter is the oversampling setting, from 0 to 3.
      // (3 is highest resolution and the longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.
      status = pressure.startPressure(3);
      if (status != 0)
        {
        delay(status);// Wait for the measurement to complete

        // Retrieve the completed pressure measurement and 
        // store it in the variable P.
        // This call requires the previous temperature measurement (T).
        // Function returns 1 if successful, 0 if failure.
        status = pressure.getPressure(P,T);
        if (status != 0)
          {
          absolutePressure=P;

          // Print out the measurement:
          Serial.print("absolute pressure: ");
          Serial.print(P,2);
          Serial.print(" mb, ");
          Serial.print(P*0.0295333727,2);
          Serial.println(" inHg");

          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and the current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in millibars, settings.altitude = 
          // current altitude in meters.
          // Result: p0 = sea-level compensated pressure in millibars
          p0 = pressure.sealevel(P,settings.altitude); 
          relativePressure=p0;

          Serial.print("relative (sea-level) pressure: ");
          Serial.print(p0,2);
          Serial.print(" mb, ");
          Serial.print(p0*0.0295333727,2);
          Serial.println(" inHg");

          // To determine the altitude from the pressure reading,
          // use the altitude function along with a baseline pressure (sea-level or other).
          // Parameters: P = absolute pressure in millibars, p0 = baseline 
          // pressure in millibars.
          // Result: a = altitude in meters.
          a = pressure.altitude(P,p0);
          computedAltitude=a;

          Serial.print("computed altitude: ");
          Serial.print(a,2);
          Serial.print(" meters, ");
          Serial.print(a*3.28084,2);
          Serial.println(" feet");
          failureCount=0;
          }
        else 
          {
          Serial.println("error retrieving pressure measurement\n");
          ok=false;
          failureCount++;
          }
        }
      else 
        {
        Serial.println("error starting pressure measurement\n");
        ok=false;
        failureCount++;
        }
      }
    else 
      {
      Serial.println("error retrieving temperature measurement\n");
      ok=false;
      failureCount++;
      };
    }
  else
    {
    Serial.println("error starting temperature measurement\n");
    ok=false;
    failureCount++;
    }

  if (failureCount==0)
    {
    strcpy(json,"{\"results\":{\"base_altitude\":<settings.altitude>,\"temperature\":<Temp>,\"absolute_pressure\":<absPress>,\"relative_pressure\":<relPress>,\"computed_altitude\":<compAlt>}}");

    fixup(json,"<settings.altitude>",settings.altitude);
    fixup(json,"<Temp>",T);
    fixup(json,"<absPress>",P);
    fixup(json,"<relPress>",p0);
    fixup(json,"<compAlt>",a);
    Serial.println(json);
    }

  if (failureCount>4)
    ESP.reset();

  return ok;
  }