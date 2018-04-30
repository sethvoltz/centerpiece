// =--------------------------------------------------------------------------------= Libraries =--=

#include <FS.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <Encoder.h>
#include <Button.h>
#include <math.h>


// =----------------------------------------------------------------------------= Configuration =--=

// Programs
#define DISPLAY_FPS                   30
#define FRAME_DELAY_MS                1000 / DISPLAY_FPS
#define BLINK_DELAY_MS                333 // 3 fps
#define DEFAULT_INTENSITY             0.1 // 0-1.0 - 10% power
#define BEAT_MULTIPLIER               2 // Number of beats per cycle or sequence
#define SPARKLE_FRAMES                7

// Wifi
#define SETUP_AP_NAME                 "Setup Centerpiece"
#define SETUP_AP_PASSWORD             "setupcenterpiece"
#define WIFI_RECONNECT_TIMER          60000 // Delay for rechecking wifi if disconnected

// MQTT
#define MAX_CONNECTION_ATTEMPTS       3 // Number of attempts before enabling display
#define SHORT_CONNECTION_DELAY        3000 // Delay between initial connection attempts
#define LONG_CONNECTION_DELAY         120000 // Delay between attempts after max attempts
#define CONNECTING_BLINK_DELAY        500
#define MQTT_ROOT                     "centerpiece"
#define DEFAULT_MQTT_SERVER           ""
#define MQTT_SERVER_LENGTH            40
#define DEFAULT_MQTT_PORT             "1883"
#define MQTT_PORT_LENGTH              6

// Neopixel
#define NEOPIXEL_PIN                  15
#define NEOPIXEL_COUNT                24

// Encoder
#define ENCODER_PINA                  12
#define ENCODER_PINB                  14
#define ENCODER_TICKS                 4

// Buttons
#define NEOPIXEL_BUTTON_PIN           13
#define NEOPIXEL_BUTTON_COUNT         24
#define CANCEL_BUTTON_PIN             4
#define ACCEPT_BUTTON_PIN             5
#define PULLUP                        true // Whether to use the internal pullup resistor.
#define INVERT                        true // Whether to invert the HIGH-LOW logic from pullup
#define DEBOUNCE_MS                   50
#define HOLD_TIME_MS                  3000


// =-------------------------------------------------------------------------------= Prototypes =--=

void displayLoop(bool);
void runProgramWhite(bool);
void runProgramCandle(bool);
void runProgramRainbow(bool);
void runProgramTwinkle(bool);
void runProgramNight(bool);
void runProgramDance(bool);


// =----------------------------------------------------------------------------------= Globals =--=

// Programs
// Define all the programs in the correct order here.
// The first program will be the default.
int currentProgram = 0;
int displayProgram = 0;
void (*renderFunc[])(bool) {
  runProgramWhite,
  runProgramCandle,
  runProgramRainbow,
  runProgramTwinkle,
  runProgramNight,
  runProgramDance
};
#define PROGRAM_COUNT (int)(sizeof(renderFunc) / sizeof(renderFunc[0]))

// Define the string name mappings for each program here for MQTT translation.
const char *programNames[] = {
  "white",
  "candle",
  "rainbow",
  "twinkle",
  "night",
  "dance"
};

// WiFi Client
WiFiClient wifiClient;
bool wifiFeaturesEnabled = false;

// MQTT
char mqtt_server[MQTT_SERVER_LENGTH] = DEFAULT_MQTT_SERVER;
char mqtt_port[MQTT_PORT_LENGTH] = DEFAULT_MQTT_PORT;
int connectionAttempts = 0;

PubSubClient mqttClient(wifiClient);
String clientId(String(ESP.getChipId(), HEX));

// Neopixel
bool shouldRunDisplay = false;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel buttonStrip = Adafruit_NeoPixel(
  NEOPIXEL_BUTTON_COUNT,
  NEOPIXEL_BUTTON_PIN,
  NEO_GRB + NEO_KHZ800
);

// Encoder
Encoder encoder(ENCODER_PINA, ENCODER_PINB);
long encoderPosition = -999;
int dialPosition = 0;
int encoderSize = PROGRAM_COUNT * ENCODER_TICKS;

// Buttons
Button cancelButton(CANCEL_BUTTON_PIN, PULLUP, INVERT, DEBOUNCE_MS);
Button acceptButton(ACCEPT_BUTTON_PIN, PULLUP, INVERT, DEBOUNCE_MS);
bool cancelHasLongPress = false;

// Colors are ordered G-R-B
uint32_t cancelColor = strip.Color(0, 192, 196);
uint32_t acceptColor = strip.Color(255, 0, 128);
uint32_t blackColor = strip.Color(0, 0, 0);
uint32_t whiteColor = strip.Color(225, 120, 255);

// Save data flag for setup config
bool shouldSaveConfig = false;

// Display
float currentBPS = 1; // Calculate BPM / 60 on save, instead of in each display loop
float globalIntensity = DEFAULT_INTENSITY;


// =--------------------------------------------------------------------------------= Utilities =--=

// Turn a topic suffix into a full MQTT topic for the centerpiece namespace
String makeTopic(String suffix, bool all = false) {
  if (all) {
    return String(String(MQTT_ROOT) + "/all/" + suffix);
  }
  return String(String(MQTT_ROOT) + "/" + clientId + "/" + suffix);
}

String makeTopic(String suffix, String newClientId) {
  return String(String(MQTT_ROOT) + "/" + newClientId + "/" + suffix);
}

// Check if a given topic containes the suffix and is for this or all nodes
bool topicMatch(String topic, String suffix) {
  return topic.equals(makeTopic(suffix)) || topic.equals(makeTopic(suffix, true));
}

// Same as the Arduino API map() function, except for floats
float floatmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint32_t hsi2rgbw(float H, float S, float I) {
  int r, g, b, w;
  float cos_h, cos_1047_h;

  H = fmod(H, 360); // cycle H around to 0-360 degrees
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

void setupMQTT() {
  int port = atoi(mqtt_port);
  mqttClient.setServer(mqtt_server, port);
  mqttClient.setCallback(callback);
  connectionAttempts = 0;
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived ["); Serial.print(topic); Serial.print("] ");
  payload[length] = 0;
  String message((char *)payload);
  Serial.println(message);

  if (topicMatch(topic, "identify")) {
    sendIdentity();
  } else if (topicMatch(topic, "program")) {
    setProgram(message);
  } else if (String(topic).startsWith(MQTT_ROOT) && String(topic).endsWith("identity")) {
    // New device online, send it the current program
    String newTopic = String(topic);
    int start = newTopic.indexOf("/", 0);
    int end = newTopic.indexOf("/", start + 1);
    String newClientId = newTopic.substring(start + 1, end);
    Serial.printf("New client online: %s, sending current program\n", newClientId.c_str());
    sendProgram(newClientId);
  }
}

void sendIdentity() {
  mqttClient.publish(makeTopic("identity").c_str(), "online");
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void mqttConnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    strip.setPixelColor(NEOPIXEL_COUNT - 1, hsi2rgbw(60, 1, globalIntensity));
    strip.setPixelColor(0, hsi2rgbw(60, 1, globalIntensity));
    strip.setPixelColor(1, hsi2rgbw(60, 1, globalIntensity));
    strip.show();

    cancelLED(false);
    acceptLED(false);
    Serial.print("Attempting MQTT connection...");

    // Attempt to connect
    if (mqttClient.connect(clientId.c_str())) {
      strip.setPixelColor(0, hsi2rgbw(120, 1, globalIntensity));
      strip.show();
      cancelLED(true);
      acceptLED(true);

      Serial.println("connected");
      sendIdentity();

      // Subscribe to topics
      mqttClient.subscribe(String(String(MQTT_ROOT) + "/+/identity").c_str());
      mqttClient.subscribe(makeTopic("identify", true).c_str());
      mqttClient.subscribe(makeTopic("program").c_str());
      mqttClient.subscribe(makeTopic("program", true).c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 3 seconds");

      // TODO: This should really be non-blocking
      // Wait 5 seconds before retrying
      delay(3000);
    }
  }
}


// =----------------------------------------------------------------------------------= Display =--=

void setupNeopixels() {
  strip.begin();
  strip.show();
}

void enableDisplay() {
  shouldRunDisplay = true;
}

void disableDisplay() {
  shouldRunDisplay = false;
  strip.clear();
}

void setProgram(int program) {
  if (program >= 0 && program < PROGRAM_COUNT && program != currentProgram) {
    currentProgram = program;
    displayProgram = program;
    Serial.printf("Setting program to %s\n", programNames[program]);
    displayLoop(true);
  }
}

void setProgram(String programName) {
  for (size_t program = 0; program < PROGRAM_COUNT; program++) {
    if (programName.equals(programNames[program])) {
      setProgram(program);
      break;
    }
  }
}

void displayLoop(bool first = false) {
  (*renderFunc[displayProgram])(first);
}

void sendProgram() {
  mqttClient.publish(makeTopic("program", true).c_str(), programNames[displayProgram]);
}

void sendProgram(String newClientId) {
  mqttClient.publish(makeTopic("program", newClientId).c_str(), programNames[displayProgram]);
}

void cancelLED(bool state) {
  buttonStrip.setPixelColor(0, state ? (currentProgram != displayProgram) ? cancelColor : whiteColor : blackColor);
  buttonStrip.show();
}

void acceptLED(bool state) {
  buttonStrip.setPixelColor(1, state ? (currentProgram != displayProgram) ? acceptColor : whiteColor : blackColor);
  buttonStrip.show();
}


// =---------------------------------------------------------------------------------= Programs =--=

// Program: White
// Steady pure white light for ambient lighting
void runProgramWhite(bool first) {
  static unsigned long updateTimer = millis();

  unsigned long updateTimeDiff = millis() - updateTimer;
  if (first || updateTimeDiff > FRAME_DELAY_MS) {
    uint32_t color = hsi2rgbw(0, 0, globalIntensity);
    for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, color); }
    strip.show();
    updateTimer = millis();
  }
}

void runProgramCandle(bool first) {
  uint32_t color = hsi2rgbw(60, 0.5, globalIntensity);
  for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, color); }
  strip.show();
}

void runProgramRainbow(bool first) {
  static unsigned long updateTimer = millis();
  static float hueOffset = 0.0;

  unsigned long updateTimeDiff = millis() - updateTimer;
  if (first || updateTimeDiff > FRAME_DELAY_MS) {
    for (int i = 0; i < NEOPIXEL_COUNT; ++i) {
      float hue = fmod((i + hueOffset) * (360 / NEOPIXEL_COUNT), 360);
      strip.setPixelColor(i, hsi2rgbw(hue, 1, globalIntensity));
    }
    strip.show();
    hueOffset = fmod(360 + hueOffset - 0.25, 360);
    updateTimer = millis();
  }
}

void runProgramTwinkle(bool first) {
  uint32_t color = hsi2rgbw(200, 1, globalIntensity);
  for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, color); }
  strip.show();
}

void runProgramNight(bool first) {
  uint32_t color = hsi2rgbw(250, 1, globalIntensity);
  for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, color); }
  strip.show();
}

void runProgramDance(bool first) {
  static unsigned long updateTimer = millis();
  static float danceOffset = 0.0;
  static float danceCurrent[NEOPIXEL_COUNT];
  static float danceTarget[NEOPIXEL_COUNT];

  if (first) {
    for (int i = 0; i < NEOPIXEL_COUNT; ++i) { danceCurrent[i] = random(360); }
    for (int i = 0; i < NEOPIXEL_COUNT; ++i) { danceTarget[i] = random(360); }
    danceOffset = 0;
  }

  unsigned long updateTimeDiff = millis() - updateTimer;
  if (first || updateTimeDiff > FRAME_DELAY_MS) {
    for (int i = 0; i < NEOPIXEL_COUNT; ++i) {
      float offset = (danceTarget[i] - danceCurrent[i]) / 50;
      float hue = danceCurrent[i] + (offset * danceOffset);
      strip.setPixelColor(i, hsi2rgbw(hue, 1, globalIntensity));
    }
    strip.show();

    danceOffset++;
    if (danceOffset > 50) {
      for (int i = 0; i < NEOPIXEL_COUNT; ++i) { danceCurrent[i] = danceTarget[i]; } // copy
      for (int i = 0; i < NEOPIXEL_COUNT; ++i) { danceTarget[i] = random(360); } // init
      danceOffset = 0;
    }

    updateTimer = millis();
  }
}


// =----------------------------------------------------------------------------------= Encoder =--=

void encoderLoop() {
  // Reset position: encoder.write(0);
  long newPosition;
  long newDial;
  newPosition = encoder.read();

  if (newPosition != encoderPosition) {
    if (newPosition >= encoderSize) {
      newPosition = newPosition % encoderSize;
      encoder.write(newPosition);
    } else if (newPosition < 0) {
      while (newPosition < 0) {
        newPosition += encoderSize;
      }
      encoder.write(newPosition);
    }

    encoderPosition = newPosition;
    newDial = encoderPosition / ENCODER_TICKS;

    if (newDial != dialPosition) {
      dialPosition = newDial;
      displayProgram = dialPosition;
      displayLoop(true);

      Serial.printf(
        "currentProgram: %s, displayProgram: %s\n",
        programNames[currentProgram],
        programNames[displayProgram]
      );
    }
  }
}

void updateEncoder(int newPosition) {
  encoder.write(newPosition * ENCODER_TICKS);
}


// =----------------------------------------------------------------------------------= Buttons =--=

void setupButtons() {
  buttonStrip.begin();
  buttonStrip.show();
}

void buttonLoop() {
  static unsigned long updateTimer = millis();
  static bool lightState = true;

  unsigned long updateTimeDiff = millis() - updateTimer;
  if (updateTimeDiff > BLINK_DELAY_MS) {
    if (displayProgram != currentProgram) {
      cancelLED(lightState);
      acceptLED(lightState);
      lightState = !lightState;
    } else {
      cancelLED(true);
      acceptLED(true);
    }
    updateTimer = millis();
  }

  cancelButton.read();
  acceptButton.read();

  if (cancelButton.pressedFor(HOLD_TIME_MS)) {
    if (!cancelHasLongPress) {
      // First trigger of long press
      cancelHasLongPress = true;
      wifiCaptivePortal();
    }
  } else if (cancelButton.wasReleased()) {
    if (cancelHasLongPress) {
      // Reset long press state
      cancelHasLongPress = false;
    } else {
      // Normal cancel press
      displayProgram = currentProgram;
      updateEncoder(currentProgram);
    }
  }

  if (acceptButton.wasReleased()) {
    currentProgram = displayProgram;
    sendProgram();
  }
}


// =-------------------------------------------------------------------------------------= WIFI =--=

void setupWifi() {
  if (WiFi.SSID() == "") {
    Serial.println("We haven't got any access point credentials, so get them now");
    wifiCaptivePortal();
  } else {
    connectWifi();
  }
}

void connectWifi() {
  // We need to check here again as this can be called multiple places
  if (WiFi.SSID() != "") {
    if (!shouldRunDisplay) {
      // Set status LED for WIFI connection start
      strip.setPixelColor(1, hsi2rgbw(20, 1, globalIntensity / 2));
      strip.show();
    }

    // Force to station mode because if device was switched off while in access point mode it will
    // start up next time in access point mode.
    WiFi.mode(WIFI_STA);
    WiFi.waitForConnectResult();
    finalizeWifi();
  }
}

// Wifi wasn't connected for some reason, let's try again periodically
void maybeConnectWifi() {
  static unsigned long updateTimer = millis();

  unsigned long updateTimeDiff = millis() - updateTimer;
  if (updateTimeDiff > WIFI_RECONNECT_TIMER) {
    Serial.println("Attempting to reconnect to wifi...");
    connectWifi();
    updateTimer = millis();
  }
}

// Finishing steps for after wifi may be complete
void finalizeWifi() {
  if (WiFi.status() != WL_CONNECTED){
    wifiFeaturesEnabled = false;
    Serial.print("Failed to connect to wifi. ");

    if (shouldRunDisplay) {
      // Display is already running
      Serial.printf("Will try again in %d seconds.", (WIFI_RECONNECT_TIMER / 1000));
    } else {
      Serial.println("Playing failure animation then proceeding.");
      // Flash the bad news, then activate the display
      for (size_t j = 0; j < 6; j++) {
        uint32_t color = hsi2rgbw(j % 2 ? 0 : 240, 1, globalIntensity);
        for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, color); }
        strip.show();
        delay(125);
      }
      enableDisplay();
    }
  } else {
    wifiFeaturesEnabled = true;
    Serial.print("Connected to WiFi. Local IP: ");
    Serial.println(WiFi.localIP());
  }
}

// gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  uint32_t color = hsi2rgbw(240, 1, globalIntensity);
  for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, color); }
  strip.show();

  Serial.println("Entered config mode...");
  Serial.println(WiFi.softAPIP());

  // if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

// Callback notifying us of the need to save config
void saveConfigCallback() {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

// Fire up a captive portal to collect network and MQTT info
// This portal is blocking, so the main loop can not run during this
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

    Serial.println("Saving config...");
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
    Serial.println("");
    configFile.close();
  }

  finalizeWifi();
}


// =------------------------------------------------------------------------------= File System =--=

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
}


// =---------------------------------------------------------------------------= Setup and Loop =--=

void setup() {
  Serial.begin(115200);
  delay(100);

  // Change Watchdog Timer to longer wait
  ESP.wdtDisable();
  ESP.wdtEnable(WDTO_8S);

  // Setup :allthethings:
  randomSeed(analogRead(0));
  setupButtons();
  setupNeopixels();
  setupFileSystem();
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

  if (wifiFeaturesEnabled) {
    if (mqttClient.connected()) {
      mqttClient.loop();
    } else {
      mqttConnect();
    }
  } else {
    maybeConnectWifi();
  }

  buttonLoop();
  encoderLoop();
  displayLoop();
}
