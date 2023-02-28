enum inverterModelT {ME3000, HYBRID};
inverterModelT inverterModel = ME3000; //default to ME3000

// The device name is used as the MQTT base topic. If you need more than one Sofar2mqtt on your network, give them unique names.
const char* version = "v3.1";

bool tftModel = true; //true means 2.8" color tft, false for oled version

#include <DoubleResetDetect.h>
#define DRD_TIMEOUT 0.1
#define DRD_ADDRESS 0x00
DoubleResetDetect drd(DRD_TIMEOUT, DRD_ADDRESS);


#include <WiFiManager.h>
#include <EEPROM.h>
#define WIFI_TIMEOUT 30000

// * To be filled with EEPROM data
char deviceName[64] = "Sofar";
char MQTT_HOST[64] = "";
char MQTT_PORT[6]  = "1883";
char MQTT_USER[32] = "";
char MQTT_PASS[32] = "";
#define MQTTRECONNECTTIMER 30000 //it takes 30 secs for each mqtt server reconnect attempt
unsigned long lastMqttReconnectAttempt = 0;



/*****
  Sofar2mqtt is a remote control interface for Sofar solar and battery inverters.
  It allows remote control of the inverter and reports the invertor status, power usage, battery state etc for integration with smart home systems such as Home Assistant and Node-Red vi MQTT.
  For read only mode, it will send status messages without the inverter needing to be in passive mode.
  It's designed to run on an ESP8266 microcontroller with a TTL to RS485 module such as MAX485 or MAX3485.
  Designed to work with TTL modules with or without the DR and RE flow control pins. If your TTL module does not have these pins then just ignore the wire from D5.

  Subscribe your MQTT client to:

  Sofar2mqtt/state

  Which provides:

  running_state
  grid_voltage
  grid_current
  grid_freq
  systemIO_power (AC side of inverter)
  battery_power  (DC side of inverter)
  battery_voltage
  battery_current
  batterySOC
  battery_temp
  battery_cycles
  grid_power
  consumption
  solarPV
  today_generation
  today_exported
  today_purchase
  today_consumption
  inverter_temp
  inverterHS_temp
  solarPVAmps

  With the inverter in Passive Mode, send MQTT messages to:

  Sofar2mqtt/set/standby   - send value "true"
  Sofar2mqtt/set/auto   - send value "true" or "battery_save"
  Sofar2mqtt/set/charge   - send values in the range 0-3000 (watts)
  Sofar2mqtt/set/discharge   - send values in the range 0-3000 (watts)

  Each of the above will return a response on:
  Sofar2mqtt/response/<function>, the message containing the response from
  the inverter, which has a result code in the lower byte and status in the upper byte.

  The result code will be 0 for success, 1 means "Invalid Work Mode" ( which possibly means
  the inverter isn't in passive mode, ) and 3 means "Inverter busy." 2 and 4 are data errors
  which shouldn't happen unless there's a cable issue or some such.

  The status bits in the upper byte indicate the following:
  Bit 0 - Charge enabled
  Bit 1 - Discharge enabled
  Bit 2 - Battery full, charge prohibited
  Bit 3 - Battery flat, discharge prohibited

  For example, a publish to Sofar2mqtt/set/charge will result in one on Sofar2mqtt/response/charge.
  AND the message with 0xff to get the result code, which should be 0.

  battery_save is a hybrid auto mode that will charge from excess solar but not discharge.

  There will also be messages published to Sofar2mqtt/response/<type> when things happen
  in the background, such as setting auto mode on startup and switching modes in battery_save mode.

  (c)Colin McGerty 2021 colin@mcgerty.co.uk
  Major version 2.0 rewrite by Adam Hill sidepipeukatgmaildotcom
  Thanks to Rich Platts for hybrid model code and testing.
  calcCRC by angelo.compagnucci@gmail.com and jpmzometa@gmail.com
*****/
#include <Arduino.h>

#define SOFAR_SLAVE_ID          0x01

#define MAX_POWER		3000 //maybe change in further models

#define RS485_TRIES 8       // x 50mS to wait for RS485 input chars.
// Wifi parameters.
#include <ESP8266WiFi.h>
WiFiClient wifi;

#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

char jsonstring[1000];


// MQTT parameters
#include <PubSubClient.h>
PubSubClient mqtt(wifi);

// SoftwareSerial is used to create a second serial port, which will be deidcated to RS485.
// The built-in serial port remains available for flashing and debugging.
#include <SoftwareSerial.h>
#define RS485_TX HIGH
#define RS485_RX LOW
#define RXPin        3  // Serial Receive pin
#define TXPin        1  // Serial Transmit pin
SoftwareSerial RS485Serial(RXPin, TXPin);


unsigned int INVERTER_RUNNINGSTATE;

#define MAX_FRAME_SIZE          64
#define MODBUS_FN_READSINGLEREG 0x03
#define MODBUS_FN_WRITESINGLEREG 0x10
#define SOFAR_FN_PASSIVEMODE    0x42
#define SOFAR_PARAM_STANDBY     0x5555

//newer models have passive control at normal holding regs, writing 6x 32-byte integers
#define SOFAR_V2_REG_PASSIVEMODE  0x1187

// Battery Save mode is a hybrid mode where the battery will charge from excess solar but not discharge.
bool BATTERYSAVE = false;

// SoFar Information Registers
#define SOFAR_REG_RUNSTATE	0x0200
#define SOFAR_REG_GRIDV		0x0206
#define SOFAR_REG_GRIDA		0x0207
#define SOFAR_REG_GRIDFREQ	0x020c
#define SOFAR_REG_BATTW		0x020d
#define SOFAR_REG_BATTV		0x020e
#define SOFAR_REG_BATTA		0x020f
#define SOFAR_REG_BATTSOC	0x0210
#define SOFAR_REG_BATTTEMP	0x0211
#define SOFAR_REG_GRIDW		0x0212
#define SOFAR_REG_LOADW		0x0213
#define SOFAR_REG_SYSIOW	0x0214
#define SOFAR_REG_PVW		0x0215
#define SOFAR_REG_PVDAY		0x0218
#define SOFAR_REG_EXPDAY	0x0219
#define SOFAR_REG_IMPDAY	0x021a
#define SOFAR_REG_LOADDAY	0x021b
#define SOFAR_REG_CHARGDAY  0x0224
#define SOFAR_REG_DISCHDAY  0x0225
#define SOFAR_REG_BATTCYC	0x022c
#define SOFAR_REG_PVA		0x0236
#define SOFAR_REG_INTTEMP	0x0238
#define SOFAR_REG_HSTEMP	0x0239
#define SOFAR_REG_PV1		0x0252
#define SOFAR_REG_PV2		0x0255

#define SOFAR_FN_STANDBY	0x0100
#define SOFAR_FN_DISCHARGE	0x0101
#define SOFAR_FN_CHARGE		0x0102
#define SOFAR_FN_AUTO		0x0103

struct mqtt_status_register
{
  inverterModelT inverter;
  uint16_t regnum;
  String    mqtt_name;
};

static struct mqtt_status_register  mqtt_status_reads[] =
{
  { ME3000, SOFAR_REG_RUNSTATE, "running_state" },
  { ME3000, SOFAR_REG_GRIDV, "grid_voltage" },
  { ME3000, SOFAR_REG_GRIDA, "grid_current" },
  { ME3000, SOFAR_REG_GRIDFREQ, "grid_freq" },
  { ME3000, SOFAR_REG_GRIDW, "grid_power" },
  { ME3000, SOFAR_REG_BATTW, "battery_power" },
  { ME3000, SOFAR_REG_BATTV, "battery_voltage" },
  { ME3000, SOFAR_REG_BATTA, "battery_current" },
  { ME3000, SOFAR_REG_SYSIOW, "systemIO_power" },
  { ME3000, SOFAR_REG_BATTSOC, "batterySOC" },
  { ME3000, SOFAR_REG_BATTTEMP, "battery_temp" },
  { ME3000, SOFAR_REG_BATTCYC, "battery_cycles" },
  { ME3000, SOFAR_REG_LOADW, "consumption" },
  { ME3000, SOFAR_REG_PVW, "solarPV" },
  { ME3000, SOFAR_REG_PVA, "solarPVAmps" },
  { ME3000, SOFAR_REG_EXPDAY, "today_exported" },
  { ME3000, SOFAR_REG_IMPDAY, "today_purchase" },
  { ME3000, SOFAR_REG_PVDAY, "today_generation" },
  { ME3000, SOFAR_REG_LOADDAY, "today_consumption" },
  { ME3000, SOFAR_REG_CHARGDAY, "today_charged" },
  { ME3000, SOFAR_REG_DISCHDAY, "today_discharged" },
  { ME3000, SOFAR_REG_INTTEMP, "inverter_temp" },
  { ME3000, SOFAR_REG_HSTEMP, "inverter_HStemp" },
  { HYBRID, SOFAR_REG_RUNSTATE, "running_state" },
  { HYBRID, SOFAR_REG_GRIDV, "grid_voltage" },
  { HYBRID, SOFAR_REG_GRIDA, "grid_current" },
  { HYBRID, SOFAR_REG_GRIDFREQ, "grid_freq" },
  { HYBRID, SOFAR_REG_GRIDW, "grid_power" },
  { HYBRID, SOFAR_REG_BATTW, "battery_power" },
  { HYBRID, SOFAR_REG_BATTV, "battery_voltage" },
  { HYBRID, SOFAR_REG_BATTA, "battery_current" },
  { HYBRID, SOFAR_REG_SYSIOW, "systemIO_power" },
  { HYBRID, SOFAR_REG_BATTSOC, "batterySOC" },
  { HYBRID, SOFAR_REG_BATTTEMP, "battery_temp" },
  { HYBRID, SOFAR_REG_BATTCYC, "battery_cycles" },
  { HYBRID, SOFAR_REG_LOADW, "consumption" },
  { HYBRID, SOFAR_REG_PVW, "solarPV" },
  { HYBRID, SOFAR_REG_PVA, "solarPVAmps" },
  { HYBRID, SOFAR_REG_PV1, "Solarpv1" },
  { HYBRID, SOFAR_REG_PV2, "Solarpv2" },
  { HYBRID, SOFAR_REG_PVDAY, "today_generation" },
  { HYBRID, SOFAR_REG_LOADDAY, "today_consumption" },
  { HYBRID, SOFAR_REG_INTTEMP, "inverter_temp" },
  { HYBRID, SOFAR_REG_HSTEMP, "inverter_HStemp" },
};

// This is the return object for the sendModbus() function. Since we are a modbus master, we
// are primarily interested in the responses to our commands.
struct modbusResponse
{
  uint8_t errorLevel;
  uint8_t data[MAX_FRAME_SIZE];
  uint8_t dataSize;
  char* errorMessage;
};

bool modbusError = true;

// These timers are used in the main loop.
#define HEARTBEAT_INTERVAL 9000
#define RUNSTATE_INTERVAL 5000
#define SEND_INTERVAL 10000
#define BATTERYSAVE_INTERVAL 3000

// Wemos OLED Shield set up. 64x48, pins D1 and D2
#include <SPI.h>
#include <Wire.h>

#include "Sofar2mqtt.h"

//for the tft
#include <Adafruit_ILI9341.h>   // include Adafruit ILI9341 TFT library
#define TFT_CS    D1     // TFT CS  pin is connected to arduino pin 8
#define TFT_DC    D2     // TFT DC  pin is connected to arduino pin 10
#define TFT_LED   D8
// initialize ILI9341 TFT library
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

//for the oled
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 0  // GPIO0
Adafruit_SSD1306 display(OLED_RESET);


#include <ArduinoOTA.h>

/**
   Check to see if the elapsed interval has passed since the passed in
   millis() value. If it has, return true and update the lastRun. Note
   that millis() overflows after 50 days, so we need to deal with that
   too... in our case we just zero the last run, which means the timer
   could be shorter but it's not critical... not worth the extra effort
   of doing it properly for once in 50 days.
*/
bool checkTimer(unsigned long *lastRun, unsigned long interval)
{
  unsigned long now = millis();

  if (*lastRun > now)
    *lastRun = 0;

  if (now >= *lastRun + interval)
  {
    *lastRun = now;
    return true;
  }

  return false;
}

// Update the OLED. Use "NULL" for no change or "" for an empty line.
String oledLine1;
String oledLine2;
String oledLine3;
String oledLine4;

void updateOLED(String line1, String line2, String line3, String line4)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  if (line1 != "NULL")
  {
    display.println(line1);
    oledLine1 = line1;
  }
  else
    display.println(oledLine1);

  display.setCursor(0, 12);

  if (line2 != "NULL")
  {
    display.println(line2);
    oledLine2 = line2;
  }
  else
    display.println(oledLine2);

  display.setCursor(0, 24);

  if (line3 != "NULL")
  {
    display.println(line3);
    oledLine3 = line3;
  }
  else
    display.println(oledLine3);

  display.setCursor(0, 36);

  if (line4 != "NULL")
  {
    display.println(line4);
    oledLine4 = line4;
  }
  else
    display.println(oledLine4);

  display.display();
}

// **********************************
// * EEPROM helpers                 *
// **********************************

String read_eeprom(int offset, int len)
{
  String res = "";
  for (int i = 0; i < len; ++i)
  {
    res += char(EEPROM.read(i + offset));
  }
  return res;
}

void write_eeprom(int offset, int len, String value)
{
  for (int i = 0; i < len; ++i)
  {
    if ((unsigned)i < value.length())
    {
      EEPROM.write(i + offset, value[i]);
    }
    else
    {
      EEPROM.write(i + offset, 0);
    }
  }
}

// * Gets called when WiFiManager enters configuration mode
void configModeCallback(WiFiManager *myWiFiManager)
{
  if (tftModel) {
    tft.println(F("Entering config mode"));
    tft.println(F("Connect your phone to WiFi: "));
    tft.println(myWiFiManager->getConfigPortalSSID());
    tft.println(F("And browse to: "));
    tft.println(WiFi.softAPIP());
  } else {
    updateOLED("NULL", "hotspot", "no config", "NULL");
  }

}


bool shouldSaveConfig = false;

// * Callback notifying us of the need to save config
void save_wifi_config_callback ()
{
  shouldSaveConfig = true;
}

void saveToEeprom() {
  write_eeprom(0, 1, "1");           // * 0 --> always "1"
  write_eeprom(1, 64, deviceName);   // * 1-64
  write_eeprom(65, 64, MQTT_HOST);   // * 65-128
  write_eeprom(129, 6, MQTT_PORT);   // * 129-134
  write_eeprom(135, 32, MQTT_USER);  // * 135-166
  write_eeprom(167, 32, MQTT_PASS);  // * 167-198
  EEPROM.write(199, inverterModel); // * 199
  EEPROM.write(200, tftModel); // * 200
  EEPROM.commit();
  ESP.reset(); // reset after save to activate new settings
}

bool loadFromEeprom() {
  // * Get MQTT Server settings
  String settings_available = read_eeprom(0, 1);

  if (settings_available == "1")
  {
    read_eeprom(1, 64).toCharArray(deviceName, 64);  // * 1-64
    read_eeprom(65, 64).toCharArray(MQTT_HOST, 64);   // * 65-128
    read_eeprom(129, 6).toCharArray(MQTT_PORT, 6);    // * 129-134
    read_eeprom(135, 32).toCharArray(MQTT_USER, 32);  // * 135-166
    read_eeprom(167, 32).toCharArray(MQTT_PASS, 32); // * 167 -198
    if (EEPROM.read(199)) inverterModel = HYBRID;
    tftModel = false;
    if (EEPROM.read(200)) tftModel = true;
    WiFi.hostname(deviceName);
    return true;
  }
  return false;
}

void setup_wifi()
{

  WiFiManagerParameter CUSTOM_MY_HOST("device", "My hostname", deviceName, 64);
  WiFiManagerParameter CUSTOM_MQTT_HOST("mqtt", "MQTT hostname", MQTT_HOST, 64);
  WiFiManagerParameter CUSTOM_MQTT_PORT("port", "MQTT port",     MQTT_PORT, 6);
  WiFiManagerParameter CUSTOM_MQTT_USER("user", "MQTT user",     MQTT_USER, 32);
  WiFiManagerParameter CUSTOM_MQTT_PASS("pass", "MQTT pass",     MQTT_PASS, 32);

  const char *bufferStr = R"(
  <br/>
  <p>Select LCD screen type:</p>
  <input style='display: inline-block;' type='radio' id='TFT' name='lcd_selection' onclick='setHiddenValueLCD()'>
  <label for='TFT'>TFT</label><br/>
  <input style='display: inline-block;' type='radio' id='OLED' name='lcd_selection' onclick='setHiddenValueLCD()'>
  <label for='OLED'>OLED</label><br/>
  <br/>  
  <p>Select inverter type:</p>
  <input style='display: inline-block;' type='radio' id='ME3000' name='inverter_selection' onclick='setHiddenValueInverter()'>
  <label for='ME3000'>ME3000SP</label><br/>
  <input style='display: inline-block;' type='radio' id='HYBRID' name='inverter_selection' onclick='setHiddenValueInverter()'>
  <label for='HYBRID'>HYDxxxxES</label><br/>
  <br/>
  <script>
  function setHiddenValueLCD() {
    var checkBox = document.getElementById('OLED');
    var hiddenvalue = document.getElementById('key_custom_lcd');
    if (checkBox.checked == true){
      hiddenvalue.value=0
    } else {
      hiddenvalue.value=1
    }
  }
  function setHiddenValueInverter() {
    var checkBox = document.getElementById('ME3000');
    var hiddenvalue = document.getElementById('key_custom_inverter');
    if (checkBox.checked == true){
      hiddenvalue.value=0
    } else {
      hiddenvalue.value=1
    }
  }
  if (document.getElementById("key_custom_lcd").value === "1") {
    document.getElementById("TFT").checked = true  
  } else {
    document.getElementById("OLED").checked = true  
  }
  document.querySelector("[for='key_custom_lcd']").hidden = true;
  document.getElementById('key_custom_lcd').hidden = true;
  if (document.getElementById("key_custom_inverter").value === "1") {
    document.getElementById("HYBRID").checked = true  
  } else {
    document.getElementById("ME3000").checked = true  
  }
  document.querySelector("[for='key_custom_inverter']").hidden = true;
  document.getElementById('key_custom_inverter').hidden = true;
  </script>
  )";
  WiFiManagerParameter custom_html_inputs(bufferStr);
  char lcdModelString[6];
  sprintf(lcdModelString, "%u", uint8_t(tftModel));
  WiFiManagerParameter custom_hidden_lcd("key_custom_lcd", "LCD type hidden", lcdModelString, 2);
  char inverterModelString[6];
  sprintf(inverterModelString, "%u", uint8_t(inverterModel));
  WiFiManagerParameter custom_hidden_inverter("key_custom_inverter", "Inverter type hidden", inverterModelString, 2);


  WiFiManager wifiManager;
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setConfigPortalTimeout(WIFI_TIMEOUT);
  wifiManager.setSaveConfigCallback(save_wifi_config_callback);
  wifiManager.addParameter(&custom_hidden_lcd);
  wifiManager.addParameter(&custom_hidden_inverter);
  wifiManager.addParameter(&custom_html_inputs);
  wifiManager.addParameter(&CUSTOM_MY_HOST);
  wifiManager.addParameter(&CUSTOM_MQTT_HOST);
  wifiManager.addParameter(&CUSTOM_MQTT_PORT);
  wifiManager.addParameter(&CUSTOM_MQTT_USER);
  wifiManager.addParameter(&CUSTOM_MQTT_PASS);



  if (!wifiManager.autoConnect("Sofar2Mqtt"))
  {
    if (tftModel) {
      tft.println(F("Failed to connect to WIFI and hit timeout"));
    } else {
      updateOLED("NULL", "NULL", "WiFi.!.", "NULL");
    }
    // * Reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(WIFI_TIMEOUT);
  }

  // * Read updated parameters
  strcpy(deviceName, CUSTOM_MY_HOST.getValue());
  strcpy(MQTT_HOST, CUSTOM_MQTT_HOST.getValue());
  strcpy(MQTT_PORT, CUSTOM_MQTT_PORT.getValue());
  strcpy(MQTT_USER, CUSTOM_MQTT_USER.getValue());
  strcpy(MQTT_PASS, CUSTOM_MQTT_PASS.getValue());
  if (atoi(custom_hidden_lcd.getValue()) == 0) tftModel = false;
  if (atoi(custom_hidden_inverter.getValue())) inverterModel = HYBRID;

  // * Save the custom parameters to FS which will also initiate a reset to activate other lcd screen if necessary
  if (shouldSaveConfig) saveToEeprom();
  if (tftModel) {
    tft.println(F("Connected to WIFI..."));
    tft.println(WiFi.localIP());
  } else {
    updateOLED("NULL", "NULL", "WiFi...", "NULL");
  }
  delay(500);

}



int addStateInfo(String &state, uint16_t reg, String human)
{
  unsigned int	val;
  modbusResponse	rs;

  if (readSingleReg(SOFAR_SLAVE_ID, reg, &rs))
    return -1;

  val = ((rs.data[0] << 8) | rs.data[1]);

  if (!( state == "{"))
    state += ",";

  state += "\"" + human + "\":" + String(val);
  return 0;
}

void sendData()
{
  static unsigned long	lastRun = 0;

  // Update all parameters and send to MQTT.
  if (checkTimer(&lastRun, SEND_INTERVAL))
  {
    String	state = "{\"uptime\":" + String(millis()) + ",\"deviceName\": \"" + String(deviceName) + "\"";
    //String  state = "{\"uptime\":" + String(millis());

    for (int l = 0; l < sizeof(mqtt_status_reads) / sizeof(struct mqtt_status_register); l++)
      if (mqtt_status_reads[l].inverter == inverterModel) {
        addStateInfo(state, mqtt_status_reads[l].regnum, mqtt_status_reads[l].mqtt_name);
        loopRuns(); //handle some other requests while building the state info
      }
    state = state + "}";

    //Prefix the mqtt topic name with deviceName.
    String topic(deviceName);
    topic += "/state";
    sendMqtt(const_cast<char*>(topic.c_str()), state);
    state.toCharArray(jsonstring, sizeof(jsonstring));
  }
}

// This function is executed when an MQTT message arrives on a topic that we are subscribed to.
void mqttCallback(String topic, byte *message, unsigned int length)
{
  if (!topic.startsWith(String(deviceName) + "/set/"))
    return;

  String messageTemp;
  uint16_t fnCode = 0, fnParam = 0;
  String cmd = topic.substring(topic.lastIndexOf("/") + 1);

  for (int i = 0; i < length; i++)
  {
    messageTemp += (char)message[i];
  }

  int   messageValue = messageTemp.toInt();
  bool  messageBool = ((messageTemp != "false") && (messageTemp != "battery_save"));

  if (cmd == "standby")
  {
    if (messageBool)
    {
      fnCode = SOFAR_FN_STANDBY;
      fnParam = SOFAR_PARAM_STANDBY;
    }
  }
  else if (cmd == "auto")
  {
    if (messageBool)
      fnCode = SOFAR_FN_AUTO;
    else if (messageTemp == "battery_save")
      BATTERYSAVE = true;
  }
  else if ((messageValue > 0) && (messageValue <= MAX_POWER))
  {
    fnParam = messageValue;

    if (cmd == "charge")
      fnCode = SOFAR_FN_CHARGE;
    else if (cmd == "discharge")
      fnCode = SOFAR_FN_DISCHARGE;
  }

  if (fnCode)
  {
    BATTERYSAVE = false;
    sendPassiveCmd(SOFAR_SLAVE_ID, fnCode, fnParam, cmd);
  }
}

void batterySave()
{
  static unsigned long	lastRun = 0;

  if (checkTimer(&lastRun, BATTERYSAVE_INTERVAL) && BATTERYSAVE)
  {
    modbusResponse  rs;

    //Get grid power
    unsigned int	p = 0;

    if (readSingleReg(SOFAR_SLAVE_ID, SOFAR_REG_GRIDW, &rs)) {
      p = ((rs.data[0] << 8) | rs.data[1]);

      // Switch to auto when any power flows to the grid.
      // We leave a little wriggle room because once you start charging the battery,
      // gridPower should be floating just above or below zero.
      if ((p < 65535 / 2 || p > 65525) && (((inverterModel == ME3000) && (INVERTER_RUNNINGSTATE != 4)) || ((inverterModel == HYBRID) && (INVERTER_RUNNINGSTATE != 6))) )
      {
        //exporting to the grid
        sendPassiveCmd(SOFAR_SLAVE_ID, SOFAR_FN_AUTO, 0, "bsave_auto");

      }
      else
      {
        //importing from the grid
        sendPassiveCmd(SOFAR_SLAVE_ID, SOFAR_FN_STANDBY, SOFAR_PARAM_STANDBY, "bsave_standby");
      }
    }
  }
}

// This function reconnects the ESP8266 to the MQTT broker
void mqttReconnect()
{
  unsigned long now = millis();
  if ((lastMqttReconnectAttempt == 0) || ((unsigned long)(now - lastMqttReconnectAttempt) > MQTTRECONNECTTIMER)) { //only try reconnect each MQTTRECONNECTTIMER seconds or on boot when lastMqttReconnectAttempt is still 0
    lastMqttReconnectAttempt = now;
    if (tftModel) {
      tft.fillCircle(220, 290, 10, ILI9341_RED);
    } else {
      updateOLED("NULL", "Offline", "NULL", "NULL");
    }
    mqtt.disconnect();		// Just in case.
    // Attempt to connect
    if (mqtt.connect(deviceName, MQTT_USER, MQTT_PASS))
    {
      if (tftModel) {
        tft.fillCircle(220, 290, 10, ILI9341_GREEN);
      } else {
        updateOLED("NULL", "Online", "NULL", "NULL");
      }
      //Set topic names to include the deviceName.
      String standbyMode(deviceName);
      standbyMode += "/set/standby";
      String autoMode(deviceName);
      autoMode += "/set/auto";
      String chargeMode(deviceName);
      chargeMode += "/set/charge";
      String dischargeMode(deviceName);
      dischargeMode += "/set/discharge";

      // Subscribe or resubscribe to topics.
      mqtt.subscribe(const_cast<char*>(standbyMode.c_str()));
      mqtt.subscribe(const_cast<char*>(autoMode.c_str()));
      mqtt.subscribe(const_cast<char*>(chargeMode.c_str()));
      mqtt.subscribe(const_cast<char*>(dischargeMode.c_str()));
    }
  }
}

/**
   Flush the RS485 buffers in both directions. The doc for Serial.flush() implies it only
   flushes outbound characters now... I assume RS485Serial is the same.
*/
void flushRS485()
{
  RS485Serial.flush();
  delay(200);

  while (RS485Serial.available())
    RS485Serial.read();
}

int sendModbus(uint8_t frame[], byte frameSize, modbusResponse *resp)
{
  //Calculate the CRC and overwrite the last two bytes.
  calcCRC(frame, frameSize);

  // Make sure there are no spurious characters in the in/out buffer.
  flushRS485();

  //Send
  RS485Serial.write(frame, frameSize);

  return listen(resp);
}

// Listen for a response.
int listen(modbusResponse *resp)
{
  uint8_t		inFrame[64];
  uint8_t		inByteNum = 0;
  uint8_t		inFrameSize = 0;
  uint8_t		inFunctionCode = 0;
  uint8_t		inDataBytes = 0;
  int		done = 0;
  modbusResponse	dummy;

  if (!resp)
    resp = &dummy;      // Just in case we ever want to interpret here.

  resp->dataSize = 0;
  resp->errorLevel = 0;

  while ((!done) && (inByteNum < sizeof(inFrame)))
  {
    int tries = 0;

    while ((!RS485Serial.available()) && (tries++ < RS485_TRIES))
      delay(50);

    if (tries >= RS485_TRIES)
    {
      break;
    }

    inFrame[inByteNum] = RS485Serial.read();

    //Process the byte
    switch (inByteNum)
    {
      case 0:
        if (inFrame[inByteNum] != SOFAR_SLAVE_ID)  //If we're looking for the first byte but it dosn't match the slave ID, we're just going to drop it.
          inByteNum--;          // Will be incremented again at the end of the loop.
        break;

      case 1:
        //This is the second byte in a frame, where the function code lives.
        inFunctionCode = inFrame[inByteNum];
        break;

      case 2:
        //This is the third byte in a frame, which tells us the number of data bytes to follow.
        if ((inDataBytes = inFrame[inByteNum]) > sizeof(inFrame))
          inByteNum = -1;       // Frame is too big?
        break;

      default:
        if (inByteNum < inDataBytes + 3)
        {
          //This is presumed to be a data byte.
          resp->data[inByteNum - 3] = inFrame[inByteNum];
          resp->dataSize++;
        }
        else if (inByteNum > inDataBytes + 3)
          done = 1;
    }

    inByteNum++;
  }

  inFrameSize = inByteNum;

  /**
    Now check to see if the last two bytes are a valid CRC.
    If we don't have a response pointer we don't care.
  **/
  if (inFrameSize < 5)
  {
    resp->errorLevel = 2;
    resp->errorMessage = "Response too short";
  }
  else if (checkCRC(inFrame, inFrameSize))
  {
    resp->errorLevel = 0;
    resp->errorMessage = "Valid data frame";
  }
  else
  {
    resp->errorLevel = 1;
    resp->errorMessage = "Error: invalid data frame";
  }

  return -resp->errorLevel;
}

int readSingleReg(uint8_t id, uint16_t reg, modbusResponse *rs)
{
  uint8_t	frame[] = { id, MODBUS_FN_READSINGLEREG, reg >> 8, reg & 0xff, 0, 0x01, 0, 0 };

  return sendModbus(frame, sizeof(frame), rs);
}

int sendPassiveCmdV2(uint8_t id, uint16_t cmd, uint16_t param, String pubTopic) {
  /*SOFAR_V2_REG_PASSIVEMODE
  *need to be finished and checked
  *writes to 4487 - 4492 with 6x 32-bit integers
  *4487 = desired PPC passive power
  *4489 = min passive power
  *4491 = max passive power
  *but 4487 isn't for forced passive mode. Set min and max to same value for that. Negative is discharging
  */
  modbusResponse  rs;
  uint8_t frame[] = { id, MODBUS_FN_WRITESINGLEREG, SOFAR_V2_REG_PASSIVEMODE >> 8, SOFAR_V2_REG_PASSIVEMODE & 0xff, 0, 6, 12, 0, 0, 0, 0, 0, 0, param >> 8, param & 0xff, 0, 0, param >> 8, param & 0xff, 0, 0 };
  int   err = -1;
  String    retMsg;

  if (sendModbus(frame, sizeof(frame), &rs))
    retMsg = rs.errorMessage;
  else if (rs.dataSize != 2)
    retMsg = "Reponse is " + String(rs.dataSize) + " bytes?";
  else
  {
    retMsg = String((rs.data[0] << 8) | (rs.data[1] & 0xff));
    err = 0;
  }

  String topic(deviceName);
  topic += "/response/" + pubTopic;
  sendMqtt(const_cast<char*>(topic.c_str()), retMsg);
  return err;
}

int sendPassiveCmd(uint8_t id, uint16_t cmd, uint16_t param, String pubTopic)
{
  modbusResponse	rs;
  uint8_t	frame[] = { id, SOFAR_FN_PASSIVEMODE, cmd >> 8, cmd & 0xff, param >> 8, param & 0xff, 0, 0 };
  int		err = -1;
  String		retMsg;

  if (sendModbus(frame, sizeof(frame), &rs))
    retMsg = rs.errorMessage;
  else if (rs.dataSize != 2)
    retMsg = "Reponse is " + String(rs.dataSize) + " bytes?";
  else
  {
    retMsg = String((rs.data[0] << 8) | (rs.data[1] & 0xff));
    err = 0;
  }

  String topic(deviceName);
  topic += "/response/" + pubTopic;
  sendMqtt(const_cast<char*>(topic.c_str()), retMsg);
  return err;
}

void sendMqtt(char* topic, String msg_str)
{
  char	msg[1000];

  mqtt.setBufferSize(1024);
  msg_str.toCharArray(msg, msg_str.length() + 1); //packaging up the data to publish to mqtt
  if (!(mqtt.publish(topic, msg)))
    printScreen("MQTT publish failed");
}


void heartbeat()
{
  static unsigned long  lastRun = 0;

  //Send a heartbeat
  if (checkTimer(&lastRun, HEARTBEAT_INTERVAL))
  {
    uint8_t	sendHeartbeat[] = {SOFAR_SLAVE_ID, 0x49, 0x22, 0x01, 0x22, 0x02, 0x00, 0x00};
    int	ret;

    if (!(ret = sendModbus(sendHeartbeat, sizeof(sendHeartbeat), NULL))) //ret=0 is good
    { if (modbusError) { //fixed previous modbus error
        modbusError = false;
        if (tftModel) {
          tft.fillCircle(20, 290, 10, ILI9341_GREEN);
        } else {
          updateOLED("NULL", "NULL", "RS485", "OK");
        }
      }
    }
    else
    {
      modbusError = true;
      if (tftModel) {
        tft.fillCircle(20, 290, 10, ILI9341_RED);
      } else {
        updateOLED("NULL", "NULL", "RS485", "ERROR");
      }

    }

  }
}

void updateRunstate()
{
  static unsigned long	lastRun = 0;

  //Check the runstate
  if (checkTimer(&lastRun, RUNSTATE_INTERVAL))
  {
    modbusResponse  response;

    if (!readSingleReg(SOFAR_SLAVE_ID, SOFAR_REG_RUNSTATE, &response))
    {
      INVERTER_RUNNINGSTATE = ((response.data[0] << 8) | response.data[1]);

      if (inverterModel == ME3000) {
        switch (INVERTER_RUNNINGSTATE)
        {
          case 0:
            printScreen("Standby");
            if (BATTERYSAVE)
              if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_LIGHTGREY);
              else if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_WHITE);
            break;

          case 1:
            printScreen("Check");
            if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_YELLOW );
            break;

          case 2:
            printScreen("Charging", String(batteryWatts()) + "W");
            if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_BLUE);
            break;

          case 3:
            printScreen("Check dis");
            if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_GREEN);
            break;

          case 4:
            printScreen("Discharging", String(-1 * batteryWatts()) + "W");
            if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_GREEN);
            break;

          case 5:
            if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_PURPLE);
            break;

          case 6:
            printScreen("EPS state");
            if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_RED);
            break;

          case 7:
            printScreen("FAULT");
            if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_RED);
            break;

          default:
            printScreen("?");
            if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_BLACK);
            break;
        }
      } else if (inverterModel == HYBRID) {
        switch (INVERTER_RUNNINGSTATE)
        {
          case 0:
            printScreen("Standby");
            if (BATTERYSAVE)
              if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_LIGHTGREY);
              else if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_WHITE);
            break;

          case 1:
            printScreen("Check");
            if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_YELLOW );
            break;

          case 2:
            {
              int w = batteryWatts();
              if (w == 0) {
                printScreen("Normal");
                if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_WHITE);
              } else if (w > 0) {
                printScreen("Charging", String(w) + "W");
                if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_BLUE);
              } else {
                printScreen("Discharging", String(w * -1) + "W");
                if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_GREEN);
              }
            }
            break;

          case 3:
            printScreen("EPS state");
            if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_PURPLE);
            break;

          case 4:
            printScreen("FAULT");
            if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_RED);
            break;

          default:
            printScreen("?");
            if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_BLACK);
            break;
        }

      }
    }
    else
    {
      printScreen("CRC fault");
      if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_RED);
    }
  }
}

int batteryWatts()
{
  if ( ((inverterModel == ME3000) && (INVERTER_RUNNINGSTATE == 2 || INVERTER_RUNNINGSTATE == 4)) || ((inverterModel == HYBRID) && (INVERTER_RUNNINGSTATE == 2)) )
  {
    modbusResponse  response;

    if (!readSingleReg(SOFAR_SLAVE_ID, SOFAR_REG_BATTW, &response))
    {
      unsigned int w = ((response.data[0] << 8) | response.data[1]);

      if (w < 32768) {
        w = w * 10;
      }
      else {
        w = (65535 - w) * -10;
      }
      return w;
    }
  }

  return 0;
}


void drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) {

  int16_t i, j, byteWidth = (w + 7) / 8;
  uint8_t byte;

  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++) {
      if (i & 7) byte <<= 1;
      else      byte   = pgm_read_byte(bitmap + j * byteWidth + i / 8);
      if (byte & 0x80) tft.drawPixel(x + i, y + j, color);
    }
  }
}

void printScreen(String text) {
  if (text.length() > 10) {
    int index = text.lastIndexOf(' ');
    String text1 = text.substring(0, index);
    String text2 = text.substring(index + 1);
    printScreen(text1, text2);
  } else {
    if (tftModel) {
      tft.fillRect(40, 135, 159, 64, ILI9341_CYAN);
      int pos = 115 - 12 * (text.length() / 2);
      tft.setCursor(pos, 160);
      tft.setTextSize(2);
      tft.setTextColor(ILI9341_BLACK, ILI9341_CYAN);
      tft.println(text);
    } else {
      updateOLED("NULL", "NULL", text, "NULL");
    }
  }
}

void printScreen(String text1, String text2) {
  if (tftModel) {
    tft.fillRect(40, 135, 159, 64, ILI9341_CYAN);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_BLACK, ILI9341_CYAN);
    { int pos = 115 - 12 * (text1.length() / 2);
      tft.setCursor(pos, 145);
      tft.println(text1);
    }
    { int pos = 115 - 12 * (text2.length() / 2);
      tft.setCursor(pos, 175);
      tft.println(text2);
    }
  } else {
    updateOLED("NULL", "NULL", text1, text2);
  }
}

void setupOTA() {
  ArduinoOTA.setHostname(deviceName);
  // ArduinoOTA.setPassword("admin");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    //Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    //Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    //Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      //Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      //Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      //Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      //Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      //Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
}

// Webserver root page
void handleRoot() {
  httpServer.send_P(200, "text/html", index_html);
}

void handleJson()
{
  httpServer.send(200, "application/json", jsonstring);
}

void handleCommand() {
  int num = httpServer.args();
  bool saveEeprom = false;
  String message = "";
  for (int i = 0 ; i < num ; i++) {
    if ((httpServer.argName(i) == "reset") || (httpServer.argName(i) == "restart") || (httpServer.argName(i) == "reboot") || ((httpServer.argName(i) == "reload"))) {
      httpServer.send(200, "text/plain", "Restarting!\r\n");
      delay(1000);
      ESP.reset();
    } else if (httpServer.argName(i) == "factoryreset") {
      httpServer.send(200, "text/plain", "Factory reset! Please restart manually.\r\n");
      delay(1000);
      resetConfig();
    } else if (httpServer.argName(i) == "devicename") {
      String value =  httpServer.arg(i);
      message += "Setting devicename to: " + value + "\r\n";
      value.toCharArray(deviceName, sizeof(deviceName));
      saveEeprom = true;
    } else if (httpServer.argName(i) == "mqtthost") {
      String value =  httpServer.arg(i);
      message += "Setting MQTT host to: " + value + "\r\n";
      value.toCharArray(MQTT_HOST, sizeof(MQTT_HOST));
      saveEeprom = true;
    } else if (httpServer.argName(i) == "mqttport") {
      String value =  httpServer.arg(i);
      message += "Setting MQTT port to: " + value + "\r\n";
      value.toCharArray(MQTT_PORT, sizeof(MQTT_PORT));
      saveEeprom = true;
    } else if (httpServer.argName(i) == "mqttuser") {
      String value =  httpServer.arg(i);
      message += "Setting MQTT username to: " + value + "\r\n";
      value.toCharArray(MQTT_USER, sizeof(MQTT_USER));
      saveEeprom = true;
    } else if (httpServer.argName(i) == "mqttpass") {
      String value =  httpServer.arg(i);
      message += "Setting MQTT password to: " + value + "\r\n";
      value.toCharArray(MQTT_PASS, sizeof(MQTT_PASS));
      saveEeprom = true;
    }
  }
  httpServer.send(200, "text/plain", message);
  if (saveEeprom) saveToEeprom();
}


void resetConfig() {
  //initiate debug led indication for factory reset
  pinMode(2, FUNCTION_0); //set it as gpio
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW); //blue led on
  if (tftModel) {
    analogWrite(TFT_LED, 32); //PWM on led pin to dim screen
    tft.fillScreen(ILI9341_RED);
    tft.fillScreen(ILI9341_BLACK);
    tft.setScrollMargins(1, 10);
    tft.setTextColor(ILI9341_RED, ILI9341_BLACK); // Red on black
    tft.println("Double reset detected, clearing config.");
  }
  WiFi.persistent(true);
  WiFi.disconnect();
  WiFi.persistent(false);
  WiFiManager wifiManager;
  wifiManager.resetSettings();
  EEPROM.begin(512);
  write_eeprom(0, 1, "0");
  EEPROM.commit();

  if (tftModel) {
    tft.println("Config cleared. Please reset to configure this device...");
  }

  while (true) {
    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
    delay(100);
  }
}

void doubleResetDetect() {
  if (drd.detect()) {
    if (tftModel) {
      tft.begin();
      tft.setRotation(2);
    }
    resetConfig();
  }
}

void setup()
{
  // * Configure EEPROM an get initial settings
  EEPROM.begin(512);
  if (!loadFromEeprom()) { //we don't have config yet, switch between lcd models after each reset
    tftModel = true;
    if (EEPROM.read(200)) tftModel = false; //previous reboot we selected TFT model, now switch to OLED
    EEPROM.write(200, tftModel); // * 200
    EEPROM.commit();
  }
  doubleResetDetect(); //detect factory reset first

  if (tftModel) {
    tft.begin();
    tft.setRotation(2);
    analogWrite(TFT_LED, 32); //PWM on led pin to dim screen
    tft.fillScreen(ILI9341_CYAN);
    tft.fillScreen(ILI9341_BLACK);
    tft.setScrollMargins(1, 10);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK); // White on black

    tft.println("Sofar2mqtt starting...");
  } else {
    //Turn on the OLED
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize OLED with the I2C addr 0x3C (for the 64x48)
    display.clearDisplay();
    display.display();
    updateOLED(deviceName, "starting", "WiFi..", version);
  }

  RS485Serial.begin(9600);
  delay(500);
  setup_wifi(); //set wifi and get settings, so first thing to do

  if (tftModel) {
    tft.print("Running inverter model: ");
    if (inverterModel == ME3000) {
      tft.println("ME3000");
    } else {
      tft.println("HYBRID");
    }
  }
  delay(1000);

  mqtt.setServer(MQTT_HOST, atoi(MQTT_PORT));
  mqtt.setCallback(mqttCallback);

  setupOTA();
  MDNS.begin(deviceName);
  httpUpdater.setup(&httpServer);
  httpServer.begin();
  MDNS.addService("http", "tcp", 80);
  httpServer.on("/", handleRoot);
  httpServer.on("/json", handleJson);
  httpServer.on("/command", handleCommand);

  //Wake up the inverter and put it in auto mode to begin with.
  if (tftModel) {
    tft.fillScreen(ILI9341_BLACK);
    drawBitmap(0, 0, background, 240, 320, ILI9341_WHITE);
    printScreen("Started");
  }
  heartbeat();
  mqttReconnect();
  if (!modbusError) sendPassiveCmd(SOFAR_SLAVE_ID, SOFAR_FN_AUTO, 0, "startup_auto");
}

void loopRuns() {
  ArduinoOTA.handle();
  httpServer.handleClient();
  MDNS.update();
}

void loop()
{
  loopRuns();

  //Send a heartbeat to keep the inverter awake and update modbusError boolean
  heartbeat();

  //Check and display the runstate
  if (!modbusError) updateRunstate();

  //make sure mqtt is still connected
  if ((!mqtt.connected()) || !mqtt.loop())
  {
    mqttReconnect();
  } else {
    //Transmit all data to MQTT
    if (!modbusError) sendData();
  }

  //Set battery save state
  if (!modbusError) batterySave();
}

//calcCRC and checkCRC are based on...
//https://github.com/angeloc/simplemodbusng/blob/master/SimpleModbusMaster/SimpleModbusMaster.cpp

void calcCRC(uint8_t frame[], byte frameSize)
{
  unsigned int temp = 0xffff, flag;

  for (unsigned char i = 0; i < frameSize - 2; i++)
  {
    temp = temp ^ frame[i];

    for (unsigned char j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>= 1;

      if (flag)
        temp ^= 0xA001;
    }
  }

  // Bytes are reversed.
  frame[frameSize - 2] = temp & 0xff;
  frame[frameSize - 1] = temp >> 8;
}

bool checkCRC(uint8_t frame[], byte frameSize)
{
  unsigned int calculated_crc, received_crc;

  received_crc = ((frame[frameSize - 2] << 8) | frame[frameSize - 1]);
  calcCRC(frame, frameSize);
  calculated_crc = ((frame[frameSize - 2] << 8) | frame[frameSize - 1]);
  return (received_crc = calculated_crc);
}
