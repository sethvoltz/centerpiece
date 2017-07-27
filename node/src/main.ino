// =--------------------------------------------------------------------------------= Libraries =--=

#include <FS.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <math.h>


// =----------------------------------------------------------------------------= Configuration =--=

// Programs
#define PROGRAM_COUNT                 6
#define FRAME_DELAY_MS                33  // 30 fps
#define LED_INTENSITY                 0.5 // 50% power

// Wifi
#define SETUP_AP_NAME                 "Setup Centerpiece"
#define SETUP_AP_PASSWORD             "setupcenterpiece"

// MQTT
#define MQTT_ROOT                     "centerpiece"
#define DEFAULT_MQTT_SERVER           ""
#define MQTT_SERVER_LENGTH            40
#define DEFAULT_MQTT_PORT             "1883"
#define MQTT_PORT_LENGTH              6

// Neopixel
#define NEOPIXEL_PIN                  14
#define NEOPIXEL_COUNT                5

// Buttons
#define BUTTON_PIN                    12
#define DEBOUNCE_MS                   30
#define HOLD_TIME_MS                  3000


// =-------------------------------------------------------------------------------= Prototypes =--=

void updateDisplay(bool first);
void setupWifi(bool reset);


// =----------------------------------------------------------------------------------= Globals =--=

// Programs
int currentProgram = 0;

// WiFi Client
bool wifiFeaturesEnabled = false;
WiFiClient wifiClient;

// MQTT
char mqtt_server[MQTT_SERVER_LENGTH] = DEFAULT_MQTT_SERVER;
char mqtt_port[MQTT_PORT_LENGTH] = DEFAULT_MQTT_PORT;

PubSubClient mqttClient(wifiClient);
String clientId(String(ESP.getChipId(), HEX));

// Neopixel
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRBW + NEO_KHZ800);

// Buttons
int buttonValue = 0; // value read from button
int buttonLastValue = 0; // buffered value of the button's previous state
long buttonDownTime; // time the button was pressed down
long buttonUpTime; // time the button was released
bool ignoreUp = false; // whether to ignore the button release because the click+hold was triggered
bool hasBoot = false; // Handle a bug where a short press is triggered on boot

// Save data flag for setup config
bool shouldSaveConfig = false;


// =--------------------------------------------------------------------------------= Utilities =--=

String makeTopic(String suffix, bool all = false) {
  if (all) {
    return String(String(MQTT_ROOT) + "/all/" + suffix);
  }
  return String(String(MQTT_ROOT) + "/" + clientId + "/" + suffix);
}

bool topicMatch(String topic, String suffix) {
  return topic.equals(makeTopic(suffix)) || topic.equals(makeTopic(suffix, true));
}

// Note: This can be removed and the original `fmod` can be used once the followin issue is closed.
// https://github.com/esp8266/Arduino/issues/612
double floatmod(double a, double b) {
    return (a - b * floor(a / b));
}

uint32_t hsi2rgbw(float H, float S, float I) {
  int r, g, b, w;
  float cos_h, cos_1047_h;

  H = floatmod(H, 360); // cycle H around to 0-360 degrees
  H = 3.14159 * H / (float)180; // Convert to radians.
  S = S > 0 ? (S < 1 ? S : 1) : 0; // clamp S and I to interval [0,1]
  I = I > 0 ? (I < 1 ? I : 1) : 0;

  if(H < 2.09439) {
    cos_h = cos(H);
    cos_1047_h = cos(1.047196667 - H);
    r = S * 255 * I / 3 * (1 + cos_h / cos_1047_h);
    g = S * 255 * I / 3 * (1 + (1 - cos_h / cos_1047_h));
    b = 0;
    w = 255 * (1 - S) * I;
  } else if(H < 4.188787) {
    H = H - 2.09439;
    cos_h = cos(H);
    cos_1047_h = cos(1.047196667 - H);
    g = S * 255 * I / 3 * (1 + cos_h / cos_1047_h);
    b = S * 255 * I / 3 * (1 + (1 - cos_h / cos_1047_h));
    r = 0;
    w = 255 * (1 - S) * I;
  } else {
    H = H - 4.188787;
    cos_h = cos(H);
    cos_1047_h = cos(1.047196667 - H);
    b = S * 255 * I / 3 * (1 + cos_h / cos_1047_h);
    r = S * 255 * I / 3 * (1 + (1 - cos_h / cos_1047_h));
    g = 0;
    w = 255 * (1 - S) * I;
  }

  return strip.Color(r, g, b, w);
}


// =-------------------------------------------------------------------------------------= MQTT =--=

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived ["); Serial.print(topic); Serial.print("] ");
  payload[length] = 0;
  String message((char *)payload);
  Serial.println(message);

  if (topicMatch(topic, "identify")) {
    sendIdentity();
  } else if (topicMatch(topic, "program")) {
    setProgram(message);
  }
}

void sendIdentity() {
  mqttClient.publish(makeTopic("identity").c_str(), "online");
}

void setProgram(String programName) {
  int program = -1;

  if (programName.equals("white")) { program = 0; }
  else if (programName.equals("candle")) { program = 1; }
  else if (programName.equals("rainbow")) { program = 2; }
  else if (programName.equals("twinkle")) { program = 3; }
  else if (programName.equals("night")) { program = 4; }
  else if (programName.equals("dance")) { program = 5; }

  if (program >= 0 && program != currentProgram) {
    currentProgram = program;
    Serial.print("Setting program to "); Serial.println(programName);
    updateDisplay(true);
  }
}

void nextProgram() {
  String program;
  switch((currentProgram + 1) % PROGRAM_COUNT) {
    case 0: program = "white"; break;
    case 1: program = "candle"; break;
    case 2: program = "rainbow"; break;
    case 3: program = "twinkle"; break;
    case 4: program = "night"; break;
    case 5: program = "dance"; break;
  }
  setProgram(program);
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void mqttConnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    strip.setPixelColor(1, hsi2rgbw(60, 1, LED_INTENSITY));
    strip.show();

    Serial.print("Attempting MQTT connection... ");

    // Attempt to connect
    if (mqttClient.connect(clientId.c_str())) {
      strip.setPixelColor(1, hsi2rgbw(120, 1, LED_INTENSITY));
      strip.show();

      Serial.println("connected");
      sendIdentity();

      // Subscribe to topics
      mqttClient.subscribe(makeTopic("identify", true).c_str());
      mqttClient.subscribe(makeTopic("program").c_str());
      mqttClient.subscribe(makeTopic("program", true).c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");

      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


// =---------------------------------------------------------------------------------= Programs =--=

void runProgramWhite(bool first) {
  static unsigned long updateTimer = millis();

  unsigned long updateTimeDiff = millis() - updateTimer;
  if (first || updateTimeDiff > FRAME_DELAY_MS) {
    for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, hsi2rgbw(0, 0, LED_INTENSITY)); }
    strip.show();
    updateTimer = millis();
  }
}

void runProgramCandle(bool first) {
  for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, hsi2rgbw(60, 0.5, LED_INTENSITY)); }
  strip.show();
}

void runProgramRainbow(bool first) {
  static unsigned long updateTimer = millis();
  static float hueOffset = 0.0;

  unsigned long updateTimeDiff = millis() - updateTimer;
  if (first || updateTimeDiff > FRAME_DELAY_MS) {
    for (int i = 0; i < NEOPIXEL_COUNT; ++i) {
      strip.setPixelColor(i, hsi2rgbw(hueOffset, 1, LED_INTENSITY));
    }
    strip.show();
    hueOffset = floatmod(360 + hueOffset - 0.25, 360);
    updateTimer = millis();
  }
}

void runProgramTwinkle(bool first) {
  for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, hsi2rgbw(200, 1, LED_INTENSITY)); }
  strip.show();
}

void runProgramNight(bool first) {
  for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, hsi2rgbw(250, 1, LED_INTENSITY)); }
  strip.show();
}

void runProgramDance(bool first) {
  static unsigned long updateTimer = millis();
  static float danceOffset = 0.0;
  static float danceCurrent;
  static float danceTarget;

  if (first) {
    danceCurrent = random(360);
    danceTarget = random(360);
    danceOffset = 0;
  }

  unsigned long updateTimeDiff = millis() - updateTimer;
  if (first || updateTimeDiff > FRAME_DELAY_MS) {
    float offset = (danceTarget - danceCurrent) / 50;
    float hue = danceCurrent + (offset * danceOffset);

    for (int i = 0; i < NEOPIXEL_COUNT; ++i) {
      strip.setPixelColor(i, hsi2rgbw(hue, 1, LED_INTENSITY));
    }
    strip.show();

    danceOffset++;
    if (danceOffset > 50) {
      danceCurrent = danceTarget; // copy
      danceTarget = random(360); // init
      danceOffset = 0;
    }

    updateTimer = millis();
  }
}


// =---------------------------------------------------------------------------= Setup and Loop =--=

void updateDisplay(bool first = false) {
  switch(currentProgram) {
    case 0: runProgramWhite(first); break;
    case 1: runProgramCandle(first); break;
    case 2: runProgramRainbow(first); break;
    case 3: runProgramTwinkle(first); break;
    case 4: runProgramNight(first); break;
    case 5: runProgramDance(first); break;
  }
}

void buttonLoop() {
  // Read the state of the button
  buttonValue = digitalRead(BUTTON_PIN);

  // Test for button pressed and store the down time
  if (buttonValue == LOW && buttonLastValue == HIGH && (millis() - buttonUpTime) > long(DEBOUNCE_MS)) {
    buttonDownTime = millis();
  }

  // Test for button release and store the up time
  if (buttonValue == HIGH && buttonLastValue == LOW && (millis() - buttonDownTime) > long(DEBOUNCE_MS)) {
    if (ignoreUp == false) {
      if (hasBoot) {
        nextProgram();
      } else {
        hasBoot = true;
      }
    } else {
      ignoreUp = false;
    }
    buttonUpTime = millis();
  }

  // Test for button held down for longer than the hold time
  if (buttonValue == LOW && (millis() - buttonDownTime) > long(HOLD_TIME_MS)) {
    ignoreUp = true;
    buttonDownTime = millis();
    wifiCaptivePortal();
  }

  buttonLastValue = buttonValue;
}

// gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  for (int i = 0; i < NEOPIXEL_COUNT; ++i) {
    strip.setPixelColor(i, hsi2rgbw(240, 1, LED_INTENSITY));
  }
  strip.show();

  Serial.println("Entered config mode...");
  Serial.println(WiFi.softAPIP());

  // if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void setupFileSystem() {
  // Clean FS, for testing
  // SPIFFS.format();

  // Read configuration from FS json
  Serial.println("Mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("Mounted file system");
    if (SPIFFS.exists("/config.json")) {
      // file exists, reading and loading
      Serial.println("Reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("Opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nParsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
        } else {
          Serial.println("Failed to load json config");
        }
      }
    }
  } else {
    Serial.println("Failed to mount FS");
  }
  // end read
}

// Callback notifying us of the need to save config
void saveConfigCallback() {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void finalizeWifi() {
  if (WiFi.status() != WL_CONNECTED){
    wifiFeaturesEnabled = false;
    Serial.print("Failed to connect to wifi. Playing failure animation then proceeding.");

    for (size_t j = 0; j < 6; j++) {
      uint32_t color = hsi2rgbw(j % 2 ? 0 : 240, 1, LED_INTENSITY);
      for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, color); }
      strip.show();
      delay(250);
    }
  } else{
    wifiFeaturesEnabled = true;
    Serial.print("Connected to WiFi. Local IP: ");
    Serial.println(WiFi.localIP());
  }
}

void wifiCaptivePortal() {
  // The extra parameters to be configured (can be either global or just in the setup). After
  // connecting, parameter.getValue() will get you the configured value:
  // id/name, placeholder/prompt, default, length
  WiFiManagerParameter config_mqtt_server("server", "MQTT Server", mqtt_server, MQTT_SERVER_LENGTH);
  WiFiManagerParameter config_mqtt_port("port", "MQTT Port", mqtt_port, MQTT_PORT_LENGTH);
  WiFiManager wifiManager;

  // Set callback for when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);

  // Set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  // Add all your parameters here
  wifiManager.addParameter(&config_mqtt_server);
  wifiManager.addParameter(&config_mqtt_port);

  // Fire up the captive portal
  String setupAPName(String(SETUP_AP_NAME) + " " + clientId);
  if (!wifiManager.startConfigPortal(setupAPName.c_str(), SETUP_AP_PASSWORD)) {
    Serial.println("Failed to connect and hit timeout. Resetting...");

    // Reset and reboot. User can try again by holding button
    delay(3000);
    ESP.reset();
    delay(5000);
  }

  // Maybe save the custom parameters to FS
  if (shouldSaveConfig) {
    // Read updated parameters
    strcpy(mqtt_server, config_mqtt_server.getValue());
    strcpy(mqtt_port, config_mqtt_port.getValue());

    Serial.println("Saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("Failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
  }

  finalizeWifi();
}

void setupWifi() {
  if (WiFi.SSID() == "") {
    Serial.println("We haven't got any access point credentials, so get them now");
    wifiCaptivePortal();
  } else {
    // Force to station mode because if device was switched off while in access point mode it will
    // start up next time in access point mode.
    WiFi.mode(WIFI_STA);

    unsigned long startedAt = millis();
    Serial.print("After waiting ");
    int connRes = WiFi.waitForConnectResult();
    float waited = (millis() - startedAt);
    Serial.print(waited / 1000);
    Serial.print(" secs in setup() connection result is ");
    Serial.println(connRes);
  }

  finalizeWifi();
}

void setupMQTT() {
  int port = atoi(mqtt_port);
  mqttClient.setServer(mqtt_server, port);
  mqttClient.setCallback(callback);
}

void setupNeopixels() {
  strip.begin();
  strip.show();
}

void setupButton() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(BUTTON_PIN, HIGH);
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // Change Watchdog Timer to longer wait
  ESP.wdtDisable();
  ESP.wdtEnable(WDTO_8S);

  // Setup :allthethings:
  setupFileSystem();
  setupButton();
  setupNeopixels();
  setupWifi();

  if (wifiFeaturesEnabled) {
    setupMQTT();
  }
}

void loop() {
  if (shouldSaveConfig) {
    // Config was rewritten, be sure to rerun setup that uses it
    setupMQTT();
    shouldSaveConfig = false;
  }
  if (wifiFeaturesEnabled && !mqttClient.connected()) { mqttConnect(); }

  mqttClient.loop();
  buttonLoop();
  updateDisplay();
}
