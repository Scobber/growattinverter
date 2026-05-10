/*

Add ESP8266 compiler to arduino IDE
  - In your Arduino IDE, go to File -> Preferences
  - Enter http://arduino.esp8266.com/stable/package_esp8266com_index.json into the "Additional Boards Manager URLs"

Used Libs
  - WiFiManager         by tzapu           https://github.com/tzapu/WiFiManager
  - PubSubClient        by Nick O´Leary    https://github.com/knolleary/pubsubclient
  - DoubleResetDetector by Khai Hoang      https://github.com/khoih-prog/ESP_DoubleResetDetector
  - ModbusMaster        by Doc Walker      https://github.com/knolleary/pubsubclient
  - ArduinoJson         by Benoit Blanchon https://github.com/bblanchon/ArduinoJson

To install the used libraries, use the embedded library manager (Sketch -> Include Library -> Manage Libraries),
or download them from github (Sketch -> Include Library -> Add .ZIP Library)

Thanks to Jethro Kairys
https://github.com/jkairys/growatt-esp8266

File -> "Show verbose output during:" "compilation".
This will show the path to the binary during compilation
e.g. C:\Users\<username>\AppData\Local\Temp\arduino_build_533155


*/
// ---------------------------------------------------------------
// User configuration area start
// ---------------------------------------------------------------

// Configuration file that contains the individual configuration and secret data (wifi password...)
// Rename the Config.h.example from the repo to Config.h and add all your config data to it
// The Config.h has been added to the .gitignore, so that your secrets will be kept
#include "Config.h"
#ifndef __CONFIG_H__
#error Please rename config.h.example to config.h
#endif

// Provide sensible defaults if the compile-time flags are not set
#ifndef ENABLE_MODBUS_COMMUNICATION
#define ENABLE_MODBUS_COMMUNICATION 0
#endif

#ifndef ENABLE_WEB_DEBUG
#define ENABLE_WEB_DEBUG 0
#endif



#ifdef ESP8266
#include <ESP8266HTTPUpdateServer.h>
#elif ESP32
#include <ESPHTTPUpdateServer.h>
#endif



#if ENABLE_DOUBLE_RESET == 1
#define ESP_DRD_USE_LITTLEFS    true
#define ESP_DRD_USE_EEPROM      false
#define DRD_TIMEOUT             10
#define DRD_ADDRESS             0
#include <ESP_DoubleResetDetector.h>
DoubleResetDetector* drd;
#endif

#if ENABLE_WEB_DEBUG == 1
char acWebDebug[1024] = "";
uint16_t u16WebMsgNo = 0;
#define WEB_DEBUG_PRINT(s) {if( (strlen(acWebDebug)+strlen(s)+50) < sizeof(acWebDebug) ) sprintf(acWebDebug, "%s#%i: %s\n", acWebDebug, u16WebMsgNo++, s);}
#else
#undef WEB_DEBUG_PRINT
#define WEB_DEBUG_PRINT(s) ;
#endif

// User configuration area end
// ---------------------------------------------------------------

#include "LittleFS.h"

#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#elif ESP32
#include <WiFi.h>
#include <WebServer.h>
#endif

#if MQTT_SUPPORTED == 1
#include <PubSubClient.h>
#endif
#include <ArduinoJson.h>


#include "Growatt.h"
bool StartedConfigAfterBoot = false;
#define CONFIG_PORTAL_MAX_TIME_SECONDS 300
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include "index.h"

#if PINGER_SUPPORTED == 1
#include <Pinger.h>
#include <PingerResponse.h>
#endif
#include <time.h>

#define LED_GN 0  // GPIO0
#define LED_RT 2  // GPIO2
#define LED_BL 16 // GPIO16

#define FORMAT_LITTLEFS_IF_FAILED true

byte btnPressed = 0;

#define NUM_OF_RETRIES 5
char u8RetryCounter = NUM_OF_RETRIES;

const char* update_path = "/firmware";
uint16_t u16PacketCnt = 0;
#if PINGER_SUPPORTED == 1
Pinger pinger;
#endif

WiFiClient   espClient;
#if MQTT_SUPPORTED == 1
PubSubClient MqttClient(espClient);

long previousConnectTryMillis = 0;
#endif
Growatt      Inverter;
#ifdef ESP8266
ESP8266WebServer httpServer(80);
#elif ESP32
WebServer httpServer(80);
#endif

#ifdef ESP8266
ESP8266HTTPUpdateServer httpUpdater;
#elif ESP32
ESPHTTPUpdateServer httpUpdater;
#endif
WiFiManager wm;

typedef enum {
    BOOT_CONNECTING,
    NORMAL_MODE,
    RECONNECTING,
    CONFIG_PORTAL_MODE
} RuntimeMode_t;

RuntimeMode_t gRuntimeMode = BOOT_CONNECTING;
bool gConfigPortalRequested = false;
uint8_t gReconnectFailures = 0;
unsigned long gLastConfigPortalMs = 0;
bool gButtonPortalLatched = false;

const uint8_t WIFI_RECONNECT_FAILS_BEFORE_PORTAL = 3;
const unsigned long CONFIG_PORTAL_COOLDOWN_MS = 30000;
const unsigned long WIFI_RECONNECT_ATTEMPT_MS = 4000;
const uint16_t FAST_CONNECT_TIMEOUT_SECONDS = 12;

const static char* serverfile = "/mqtts";
const static char* portfile = "/mqttp";
const static char* topicfile = "/mqttt";
const static char* userfile = "/mqttu";
const static char* secretfile = "/mqttw";

String mqttserver = "";
String mqttport = "";
String mqtttopic = "";
String mqttuser = "";
String mqttpwd = "";

char JsonPayload[MQTT_MAX_PACKET_SIZE] = "{\"InverterStatus\": -1 }";

void updateMqttClientServer()
{
#if MQTT_SUPPORTED == 1
    uint16_t port = mqttport.toInt();
    if (port == 0)
        port = 1883;

    MqttClient.setServer(mqttserver.c_str(), port);
#endif
}

bool enterConfigPortal(const char* reason)
{
    gRuntimeMode = CONFIG_PORTAL_MODE;
    gLastConfigPortalMs = millis();
    gConfigPortalRequested = false;
    StartedConfigAfterBoot = false;

    digitalWrite(LED_BL, 1);

#if ENABLE_DEBUG_OUTPUT == 1
    Serial.print(F("Entering WiFiManager portal: "));
    Serial.println(reason);
#endif

    wm.setAPStaticIPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
    wm.setConfigPortalTimeout(CONFIG_PORTAL_MAX_TIME_SECONDS);

    bool res = wm.startConfigPortal("GrowattConfig");

    digitalWrite(LED_BL, 0);

    if (res && WiFi.status() == WL_CONNECTED)
    {
        gRuntimeMode = NORMAL_MODE;
        gReconnectFailures = 0;
#if ENABLE_DEBUG_OUTPUT == 1
        Serial.println(F("Portal connected, back to normal mode"));
#endif
        return true;
    }

    gRuntimeMode = RECONNECTING;
#if ENABLE_DEBUG_OUTPUT == 1
    Serial.println(F("Portal exited without WiFi, continuing reconnect mode"));
#endif
    return false;
}

void SendSettingsSite(void)
{
    StaticJsonDocument<384> doc;
    doc["mqttserver"] = mqttserver;
    doc["mqttport"] = mqttport;
    doc["mqtttopic"] = mqtttopic;
    doc["mqttuser"] = mqttuser;
    doc["mqttpwd"] = mqttpwd;
    doc["wifiConnected"] = (WiFi.status() == WL_CONNECTED);

    JsonPayload[0] = '\0';
    serializeJson(doc, JsonPayload, sizeof(JsonPayload));
    httpServer.send(200, "application/json", JsonPayload);
}

void HandleSettingsPost(void)
{
    if (!httpServer.hasArg("mqttserver") || !httpServer.hasArg("mqttport") || !httpServer.hasArg("mqtttopic") ||
        !httpServer.hasArg("mqttuser") || !httpServer.hasArg("mqttpwd"))
    {
        httpServer.send(400, "text/plain", "Missing settings fields");
        return;
    }

    String newServer = httpServer.arg("mqttserver");
    String newPort = httpServer.arg("mqttport");
    String newTopic = httpServer.arg("mqtttopic");
    String newUser = httpServer.arg("mqttuser");
    String newPwd = httpServer.arg("mqttpwd");

    if (newServer.length() > 40 || newPort.length() > 6 || newTopic.length() > 64 ||
        newUser.length() > 40 || newPwd.length() > 40)
    {
        httpServer.send(400, "text/plain", "Settings value too long");
        return;
    }

    uint16_t port = newPort.toInt();
    if (port == 0)
    {
        httpServer.send(400, "text/plain", "Invalid MQTT port");
        return;
    }

    mqttserver = newServer;
    mqttport = newPort;
    mqtttopic = newTopic;
    mqttuser = newUser;
    mqttpwd = newPwd;

    bool saveOk = true;
    saveOk &= write_to_file(serverfile, mqttserver);
    saveOk &= write_to_file(portfile, mqttport);
    saveOk &= write_to_file(topicfile, mqtttopic);
    saveOk &= write_to_file(userfile, mqttuser);
    saveOk &= write_to_file(secretfile, mqttpwd);

    updateMqttClientServer();

#if MQTT_SUPPORTED == 1
    if (MqttClient.connected())
    {
        MqttClient.disconnect();
    }
#endif

    if (!saveOk)
    {
        httpServer.send(500, "text/plain", "Failed to persist settings");
        return;
    }

    httpServer.send(200, "text/plain", "Settings saved");
}

// -------------------------------------------------------
// Check the WiFi status and reconnect if necessary
// -------------------------------------------------------
void WiFi_Reconnect()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        gRuntimeMode = RECONNECTING;
        digitalWrite(LED_GN, 0);

        uint32_t start = millis();
        WiFi.reconnect();

        while ((WiFi.status() != WL_CONNECTED) && (millis() - start < WIFI_RECONNECT_ATTEMPT_MS))
        {
            delay(200);
#if ENABLE_DEBUG_OUTPUT == 1
            Serial.print("x");
#endif
            digitalWrite(LED_RT, !digitalRead(LED_RT)); // toggle red led on WiFi (re)connect
        }

        if (WiFi.status() == WL_CONNECTED)
        {
#if ENABLE_DEBUG_OUTPUT == 1
            Serial.println("");
            WiFi.printDiag(Serial);
            Serial.print("local IP:");
            Serial.println(WiFi.localIP());
            Serial.print("Hostname: ");
            Serial.println(HOSTNAME);
#endif

            WEB_DEBUG_PRINT("WiFi reconnected")

            digitalWrite(LED_RT, 1);
            gReconnectFailures = 0;
            gRuntimeMode = NORMAL_MODE;
        }
        else
        {
            if (gReconnectFailures < 255)
                gReconnectFailures++;

            if (gReconnectFailures >= WIFI_RECONNECT_FAILS_BEFORE_PORTAL && (millis() - gLastConfigPortalMs) > CONFIG_PORTAL_COOLDOWN_MS)
            {
                gReconnectFailures = 0;
                gConfigPortalRequested = true;
#if ENABLE_DEBUG_OUTPUT == 1
                Serial.println();
                Serial.println(F("WiFi unavailable, scheduling WiFiManager portal"));
#endif
            }
        }
    }
}

// Conection can fail after sunrise. The stick powers up before the inverter.
// So the detection of the inverter will fail. If no inverter is detected, we have to retry later (s. loop() )
// The detection without running inverter will take several seconds, because the ModBus-Lib has a timeout of 2s 
// for each read access (and we do several of them). The WiFi can crash during this function. Perhaps we can fix 
// this by using the callback function of the ModBus-Lib
void InverterReconnect(void)
{
    // Baudrate will be set here, depending on the version of the stick
    Inverter.begin(Serial);

    #if ENABLE_WEB_DEBUG == 1
        if (Inverter.GetWiFiStickType() == ShineWiFi_S)
            WEB_DEBUG_PRINT("ShineWiFi-S (Serial) found")
        else if (Inverter.GetWiFiStickType() == ShineWiFi_X)
            WEB_DEBUG_PRINT("ShineWiFi-X (USB) found")
        else
            WEB_DEBUG_PRINT("Error: Unknown Shine Stick")
    #endif
}



// -------------------------------------------------------
// Check the Mqtt status and reconnect if necessary
// -------------------------------------------------------
#if MQTT_SUPPORTED == 1
bool MqttReconnect()
{
    if (mqttserver.length() == 0)
    {
        //No server configured
        return false;
    }

    if (WiFi.status() != WL_CONNECTED)
        return false;

    if (MqttClient.connected())
        return true;

    if (millis() - previousConnectTryMillis >= (5000))
    {
        #if ENABLE_DEBUG_OUTPUT == 1
            Serial.print("MqttServer: "); Serial.println(mqttserver);
            Serial.print("MqttUser: "); Serial.println(mqttuser);
            Serial.print("MqttTopic: "); Serial.println(mqtttopic);
            Serial.print("Attempting MQTT connection...");
        #endif

        //Run only once every 5 seconds
        previousConnectTryMillis = millis();
        // Attempt to connect with last will
        if (MqttClient.connect(getId().c_str(), mqttuser.c_str(), mqttpwd.c_str(), mqtttopic.c_str(), 1, 1, "{\"InverterStatus\": -1 }"))
        {
            #if ENABLE_DEBUG_OUTPUT == 1
                Serial.println("connected");
                return true;
            #endif
        }
        else
        {
            #if ENABLE_DEBUG_OUTPUT == 1
                Serial.print("failed, rc=");
                Serial.print(MqttClient.state());
                Serial.println(" try again in 5 seconds");
            #endif
            WEB_DEBUG_PRINT("MQTT Connect failed")
            previousConnectTryMillis = millis();
        }
    }
    return false;
}
#endif

String load_from_file(const char* file_name, String defaultvalue) {
    String result = "";

    File this_file = LittleFS.open(file_name, "r");
    if (!this_file) { // failed to open the file, return defaultvalue
        return defaultvalue;
    }

    while (this_file.available()) {
        result += (char)this_file.read();
    }

    this_file.close();
    return result;
}

bool write_to_file(const char* file_name, String contents) {
    File this_file = LittleFS.open(file_name, "w");
    if (!this_file) { // failed to open the file, return false
        return false;
    }

    int bytesWritten = this_file.print(contents);

    if (bytesWritten == 0) { // write failed
        return false;
    }

    this_file.close();
    return true;
}

String getId()
{
    #ifdef ESP8266
    uint64_t id = ESP.getChipId();
    #elif ESP32
    uint64_t id = ESP.getEfuseMac();
    #endif

    return String("Growatt") + String(id);
}

void setup()
{
    #if ENABLE_DEBUG_OUTPUT == 1
        Serial.begin(115200);
        Serial.println(F("Setup()"));
    #endif
    WEB_DEBUG_PRINT("Setup()");

    #if ENABLE_DOUBLE_RESET == 1
    drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);
    #endif

    pinMode(LED_GN, OUTPUT);
    pinMode(LED_RT, OUTPUT);
    pinMode(LED_BL, OUTPUT);

    #ifdef ESP8266
    LittleFS.begin();
    #elif ESP32
    LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED);
    #endif

    #if MQTT_SUPPORTED == 1
        mqttserver = load_from_file(serverfile, "10.1.2.3");
        mqttport = load_from_file(portfile, "1883");
        mqtttopic = load_from_file(topicfile, "energy/solar");
        mqttuser = load_from_file(userfile, "");
        mqttpwd = load_from_file(secretfile, "");
    #endif

    #if ENABLE_DOUBLE_RESET == 1
    if (drd->detectDoubleReset()) {
        #if ENABLE_DEBUG_OUTPUT == 1
            Serial.println(F("Double reset detected"));
        #endif
        StartedConfigAfterBoot = true;
    }
    #endif

    WiFi.hostname(HOSTNAME);
    WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP

    #if MQTT_SUPPORTED == 1
        // make sure the packet size is set correctly in the library
        MqttClient.setBufferSize(MQTT_MAX_PACKET_SIZE);
    #endif

    std::vector<const char*> menu = { "wifi", "wifinoscan", "erase", "restart" };
    wm.setMenu(menu);

    digitalWrite(LED_BL, 1);
    // First try fast STA connect only. If WiFi is unavailable, then move to WiFiManager portal.
    wm.setAPStaticIPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
    wm.setConfigPortalTimeout(CONFIG_PORTAL_MAX_TIME_SECONDS);
    wm.setConnectTimeout(FAST_CONNECT_TIMEOUT_SECONDS);
    wm.setEnableConfigPortal(false);
    bool res = wm.autoConnect("GrowattConfig");

    if (res && WiFi.status() == WL_CONNECTED)
    {
        #if ENABLE_DEBUG_OUTPUT == 1
            Serial.println(F("WiFi connected in STA mode"));
        #endif
        gRuntimeMode = NORMAL_MODE;
    }
    else
    {
        gRuntimeMode = RECONNECTING;
        #if ENABLE_DEBUG_OUTPUT == 1
            Serial.println(F("No WiFi available, opening WiFiManager portal"));
        #endif
        enterConfigPortal("Initial WiFi unavailable");
    }

    digitalWrite(LED_BL, 0);

    // Initialize time via NTP for proper timestamp generation
    if (WiFi.status() == WL_CONNECTED)
    {
        configTime(0, 0, "pool.ntp.org");
        time_t now = time(nullptr);
        for (uint8_t i = 0; i < 10 && now < 100000; i++) {
            delay(500);
            now = time(nullptr);
        }
    }

    #if MQTT_SUPPORTED == 1
        updateMqttClientServer();
        #if ENABLE_DEBUG_OUTPUT == 1
            Serial.print(F("MqttServer: ")); Serial.println(mqttserver);
            Serial.print(F("MqttPort: ")); Serial.println(mqttport);
            Serial.print(F("MqttTopic: ")); Serial.println(mqtttopic);
        #endif
    #endif
    

    httpServer.on("/status", SendJsonSite);
    httpServer.on("/uistatus", SendUiJsonSite);
    httpServer.on("/solar_api/v1/GetInverterRealtimeData.cgi", SendFroniusSite);
    httpServer.on("/solar_api/v1/GetPowerFlowRealtimeData.fcgi", SendPowerFlowSite);
    httpServer.on("/solar_api/v1/GetDeviceInfo.cgi", SendDeviceInfoSite);
    httpServer.on("/solar_api/v1/GetInverterInfo.cgi", SendInverterInfoSite);
    httpServer.on("/solar_api/v1/GetLoggerInfo.cgi", SendLoggerInfoSite);
    httpServer.on("/solar_api/v1/GetActiveDeviceInfo.cgi", SendActiveDeviceInfoSite);
    httpServer.on("/StartAp", StartConfigAccessPoint);
    httpServer.on("/settings", HTTP_GET, SendSettingsSite);
    httpServer.on("/settings", HTTP_POST, HandleSettingsPost);
    httpServer.on("/postCommunicationModbus", SendPostSite);
    httpServer.on("/postCommunicationModbus_p", HTTP_POST, handlePostData);
    httpServer.on("/", MainPage);
    #if ENABLE_WEB_DEBUG == 1
        httpServer.on("/debug", SendDebug);
    #endif

    Inverter.InitProtocol();
    InverterReconnect();
#if GROWATT_MODBUS_VERSION == 125
    Inverter.ConfigureExportLimit(100);
#endif

    httpUpdater.setup(&httpServer, update_path);
    httpServer.begin();
}

void SendJsonSite(void)
{
    JsonPayload[0] = '\0';
    Inverter.CreateJson(JsonPayload, WiFi.macAddress().c_str());
    httpServer.send(200, "application/json", JsonPayload);
}

void SendUiJsonSite(void)
{
    JsonPayload[0] = '\0';
    Inverter.CreateUIJson(JsonPayload);
    httpServer.send(200, "application/json", JsonPayload);
}

void SendFroniusSite(void)
{
    JsonPayload[0] = '\0';
    Inverter.CreateFroniusJson(JsonPayload);
    httpServer.send(200, "application/json", JsonPayload);
}

void SendPowerFlowSite(void)
{
    JsonPayload[0] = '\0';
    Inverter.CreatePowerFlowJson(JsonPayload);
    httpServer.send(200, "application/json", JsonPayload);
}

void SendDeviceInfoSite(void)
{
    JsonPayload[0] = '\0';
    Inverter.CreateDeviceInfoJson(JsonPayload);
    httpServer.send(200, "application/json", JsonPayload);
}

void SendInverterInfoSite(void)
{
    JsonPayload[0] = '\0';
    Inverter.CreateInverterInfoJson(JsonPayload);
    httpServer.send(200, "application/json", JsonPayload);
}

void SendLoggerInfoSite(void)
{
    JsonPayload[0] = '\0';
    Inverter.CreateLoggerInfoJson(JsonPayload);
    httpServer.send(200, "application/json", JsonPayload);
}

void SendActiveDeviceInfoSite(void)
{
    JsonPayload[0] = '\0';
    Inverter.CreateActiveDeviceInfoJson(JsonPayload);
    httpServer.send(200, "application/json", JsonPayload);
}

void StartConfigAccessPoint(void)
{
    String Text;
    Text = "Configuration access point requested ...\r\nConnect to Wifi: \"GrowattConfig\" and visit 192.168.4.1\r\nThis portal is for WiFi/network configuration only.";
    httpServer.send(200, "text/plain", Text);
    wm.setAPStaticIPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
    StartedConfigAfterBoot = true;
}

#if ENABLE_WEB_DEBUG == 1
void SendDebug(void)
{
    httpServer.send(200, "text/plain", acWebDebug);
}
#endif

void MainPage(void)
{
    httpServer.send(200, "text/html", MAIN_page);
}

void SendPostSite(void)
{
    httpServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    httpServer.send(200, "text/html", "");

    httpServer.sendContent("<form action=\"/postCommunicationModbus_p\" method=\"POST\">");
    httpServer.sendContent("<input type=\"text\" name=\"reg\" placeholder=\"Register ID\"></br>");
    httpServer.sendContent("<input type=\"text\" name=\"val\" placeholder=\"Input Value (16bit only!)\"></br>");
    httpServer.sendContent("<select name=\"type\"><option value=\"16b\" selected>16b</option><option value=\"32b\">32b</option></select></br>");
    httpServer.sendContent("<select name=\"operation\"><option value=\"R\" selected>Read</option><option value=\"W\">Write</option></select></br>");
    httpServer.sendContent("<select name=\"registerType\"><option value=\"I\" selected>Input Register</option><option value=\"H\">Holding Register</option></select></br>");
    httpServer.sendContent("<input type=\"submit\" value=\"Go\"></form>");


    httpServer.sendContent("<h3>Input Registers</h3><table border=\"1\"><tr><th>Name</th><th>Address</th><th>Value</th></tr>");
    for (int i = 0; i < Inverter._Protocol.InputRegisterCount; i++)
    {
        httpServer.sendContent("<tr><td>" + String(Inverter._Protocol.InputRegisters[i].name) + "</td><td>" +
                               String(Inverter._Protocol.InputRegisters[i].address) + "</td><td>" +
                               String(Inverter._Protocol.InputRegisters[i].value) + "</td></tr>");
    }
    httpServer.sendContent("</table>");

    httpServer.sendContent("<h3>Holding Registers</h3><table border=\"1\"><tr><th>Name</th><th>Address</th><th>Value</th></tr>");
    for (int i = 0; i < Inverter._Protocol.HoldingRegisterCount; i++)
    {
        httpServer.sendContent("<tr><td>" + String(Inverter._Protocol.HoldingRegisters[i].name) + "</td><td>" +
                               String(Inverter._Protocol.HoldingRegisters[i].address) + "</td><td>" +
                               String(Inverter._Protocol.HoldingRegisters[i].value) + "</td></tr>");
    }
    httpServer.sendContent("</table>");
}

void handlePostData()
{
    char* msg;
    uint16_t u16Tmp;
    uint32_t u32Tmp;

    msg = JsonPayload;
    msg[0] = 0;

    if (!httpServer.hasArg("reg") || !httpServer.hasArg("val"))
    {
        // If the POST request doesn't have data
        httpServer.send(400, "text/plain", "400: Invalid Request"); // The request is invalid, so send HTTP status 400
        return;
    }
    else
    {
        if (httpServer.arg("operation") == "R")
        {
            if (httpServer.arg("registerType") == "I")
            {
                if (httpServer.arg("type") == "16b")
                {
                    if (Inverter.ReadInputReg(httpServer.arg("reg").toInt(), &u16Tmp))
                    {
                        sprintf(msg, "Read 16b Input register %ld with value %d", httpServer.arg("reg").toInt(), u16Tmp);
                    }
                    else
                    {
                        sprintf(msg, "Read 16b Input register %ld impossible - not connected?", httpServer.arg("reg").toInt());
                    }
                }
                else
                {
                    if (Inverter.ReadInputReg(httpServer.arg("reg").toInt(), &u32Tmp))
                    {
                        sprintf(msg, "Read 32b Input register %ld with value %d", httpServer.arg("reg").toInt(), u32Tmp);
                    }
                    else
                    {
                        sprintf(msg, "Read 32b Input register %ld impossible - not connected?", httpServer.arg("reg").toInt());
                    }
                }
            }
            else
            {
                if (httpServer.arg("type") == "16b")
                {
                    if (Inverter.ReadHoldingReg(httpServer.arg("reg").toInt(), &u16Tmp))
                    {
                        sprintf(msg, "Read 16b Holding register %ld with value %d", httpServer.arg("reg").toInt(), u16Tmp);
                    }
                    else
                    {
                        sprintf(msg, "Read 16b Holding register %ld impossible - not connected?", httpServer.arg("reg").toInt());
                    }
                }
                else
                {
                    if (Inverter.ReadHoldingReg(httpServer.arg("reg").toInt(), &u32Tmp))
                    {
                        sprintf(msg, "Read 32b Holding register %ld with value %d", httpServer.arg("reg").toInt(), u32Tmp);
                    }
                    else
                    {
                        sprintf(msg, "Read 32b Holding register %ld impossible - not connected?", httpServer.arg("reg").toInt());
                    }
                }
            }
        }
        else
        {
            if (httpServer.arg("registerType") == "H")
            {
                if (httpServer.arg("type") == "16b")
                {
                    if (Inverter.WriteHoldingReg(httpServer.arg("reg").toInt(), httpServer.arg("val").toInt()))
                    {
                        sprintf(msg, "Wrote Holding Register %ld to a value of %ld!", httpServer.arg("reg").toInt(), httpServer.arg("val").toInt());
                    }
                    else
                    {
                        sprintf(msg, "Read 16b Holding register %ld impossible - not connected?", httpServer.arg("reg").toInt());
                    }
                }
                else
                {
                    sprintf(msg, "Writing to double (32b) registers not supported");
                }
            }
            else
            {
                sprintf(msg, "It is not possible to write into Input Registers");
            }
        }
        httpServer.send(200, "text/plain", msg);
        return;
    }
}

// -------------------------------------------------------
// Main loop
// -------------------------------------------------------
long ButtonTimer = 0;
long LEDTimer = 0;
long RefreshTimer = 0;
long WifiRetryTimer = 0;
uint8_t refreshCycle = 0;

void loop()
{
    #if ENABLE_DOUBLE_RESET == 1
    drd->loop();
    #endif

    long now = millis();
    char readoutSucceeded;

    if ((now - ButtonTimer) > BUTTON_TIMER)
    {
        ButtonTimer = now;

        if( AP_BUTTON_PRESSED )
        {
            if (btnPressed < 255)
                btnPressed++;

            if (btnPressed > 5 && !gButtonPortalLatched)
            {
                #if ENABLE_DEBUG_OUTPUT == 1
                    Serial.println("Handle press");
                #endif
                StartedConfigAfterBoot = true;
                gButtonPortalLatched = true;
            }
            #if ENABLE_DEBUG_OUTPUT == 1
                Serial.print("Btn pressed");
            #endif
        }
        else
        {
            btnPressed = 0;
            gButtonPortalLatched = false;
        }
    }

    if ((StartedConfigAfterBoot || gConfigPortalRequested) && ((millis() - gLastConfigPortalMs) > CONFIG_PORTAL_COOLDOWN_MS))
    {
        digitalWrite(LED_BL, 1);
        httpServer.stop();
        #if ENABLE_DEBUG_OUTPUT == 1
            Serial.println("Config after boot started");
        #endif
        enterConfigPortal("Requested from runtime");
        httpServer.begin();
    }

    WiFi_Reconnect();

    #if MQTT_SUPPORTED == 1
        if (MqttReconnect())
        {
            MqttClient.loop();
        }
    #endif

    httpServer.handleClient();

    // Toggle green LED with 1 Hz (alive)
    // ------------------------------------------------------------
    if ((now - LEDTimer) > LED_TIMER)
    {
        if (WiFi.status() == WL_CONNECTED)
            digitalWrite(LED_GN, !digitalRead(LED_GN));
        else
            digitalWrite(LED_GN, 0);

        LEDTimer = now;
    }

    // InverterReconnect() takes a long time --> wifi will crash
    // Do it only every two minutes
    if ((now - WifiRetryTimer) > WIFI_RETRY_TIMER)
    {
        if (Inverter.GetWiFiStickType() == Undef_stick)
            InverterReconnect();
        WifiRetryTimer = now;
    }

    // Read Inverter every REFRESH_TIMER ms [defined in config.h]
    // ------------------------------------------------------------
    if ((now - RefreshTimer) > REFRESH_TIMER)
    {
        if ((WiFi.status() == WL_CONNECTED) && (Inverter.GetWiFiStickType()))
        {
            readoutSucceeded = 0;
            bool fullRead = (refreshCycle % FULL_READ_INTERVAL) == 0;
            refreshCycle++;
            while ((u8RetryCounter) && !(readoutSucceeded))
            {
                #if SIMULATE_INVERTER == 1
                if (1) // do it always
                #else
                if (Inverter.ReadData(fullRead)) // get new data from inverter
                #endif
                {
                    WEB_DEBUG_PRINT("ReadData() successful")
                    u16PacketCnt++;
                    u8RetryCounter = NUM_OF_RETRIES;


                    // Create JSON string
                    JsonPayload[0] = '\0';
                    Inverter.CreateJson(JsonPayload, WiFi.macAddress().c_str());

                    #if MQTT_SUPPORTED == 1
                    if (MqttClient.connected())
                        MqttClient.publish(mqtttopic.c_str(), JsonPayload, true);
                    #endif

                    digitalWrite(LED_RT, 0); // clear red led if everything is ok
                    // leave while-loop
                    readoutSucceeded = 1;
                }
                else
                {
                    WEB_DEBUG_PRINT("ReadData() NOT successful")
                    if (u8RetryCounter)
                    {
                        u8RetryCounter--;
                    }
                    else
                    {
                        WEB_DEBUG_PRINT("Retry counter\n")
                        sprintf(JsonPayload, "{\"InverterStatus\": -1 }");
                        #if MQTT_SUPPORTED == 1
                        if (MqttClient.connected())
                            MqttClient.publish(mqtttopic.c_str(), JsonPayload, true);
                        #endif
                        digitalWrite(LED_RT, 1); // set red led in case of error
                    }
                }
            }
            u8RetryCounter = NUM_OF_RETRIES;
        }

        #if MQTT_SUPPORTED == 1
            if (!MqttClient.connected())
                digitalWrite(LED_RT, 1);
            else
                digitalWrite(LED_RT, 0);
        #endif

        #if PINGER_SUPPORTED == 1
            //frequently check if gateway is reachable
            if (pinger.Ping(GATEWAY_IP) == false)
                WiFi.disconnect();
        #endif

        RefreshTimer = now;
    }
}