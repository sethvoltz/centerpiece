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
#define DISPLAY_FPS                   30
#define FRAME_DELAY_MS                1000 / DISPLAY_FPS
#define LED_INTENSITY                 0.5 // 0-1.0 - 50% power
#define BEAT_MULTIPLIER               2 // Number of beats per cycle or sequence

// Wifi
#define SETUP_AP_NAME                 "Setup Centerpiece"
#define SETUP_AP_PASSWORD             "setupcenterpiece"

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
#define NEOPIXEL_PIN                  14
#define NEOPIXEL_COUNT                5
#define MAX_ATTEMPTS                  4

// Buttons
#define BUTTON_PIN                    12
#define DEBOUNCE_MS                   30
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
void (*renderFunc[])(bool) {
  runProgramWhite,
  runProgramCandle,
  runProgramRainbow,
  runProgramTwinkle,
  runProgramNight,
  runProgramDance
};
#define PROGRAM_COUNT (sizeof(renderFunc) / sizeof(renderFunc[0]))

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

// Buttons
int buttonValue = 0;     // Value read from button
int buttonLastValue = 0; // Buffered value of the button's previous state
long buttonDownTime;     // Time the button was pressed
long buttonUpTime;       // Time the button was released
bool ignoreUp = false;   // Whether to ignore the button release because click+hold was triggered
bool hasBoot = false;    // Handle a bug where a short press is triggered on boot

// Save data flag for setup config
bool shouldSaveConfig = false;

// Current Tempo
float currentBPS = 1; // Calculate BPM / 60 on save, instead of in each display loop


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

// Note: This can be removed and the original `fmod` can be used once the following issue is closed.
// https://github.com/esp8266/Arduino/issues/612
double floatmod(double a, double b) {
  return (a - b * floor(a / b));
}

float floatmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
  } else if (topicMatch(topic, "bpm")) {
    setBPM(message);
  }
}

void sendIdentity() {
  mqttClient.publish(makeTopic("identity").c_str(), "online");
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void mqttConnect() {
  static unsigned long connectTimer = millis();
  static unsigned long updateTimer = millis();
  static bool ledOn = false;

  unsigned long connectTimeDiff = millis() - connectTimer;
  unsigned long updateTimeDiff = millis() - updateTimer;

  if (!mqttClient.connected()) {
    // While not connected, try every few seconds
    bool hasExceededMaxAttempts = connectionAttempts > MAX_CONNECTION_ATTEMPTS;
    int connectionDelay = hasExceededMaxAttempts ? LONG_CONNECTION_DELAY : SHORT_CONNECTION_DELAY;
    if (connectTimeDiff > connectionDelay) {
      Serial.print("Attempting MQTT connection... ");

      // TODO: Make this connect more async -- the 2-3 second delay blocks animation
      if (mqttClient.connect(clientId.c_str())) {
        Serial.println("connected");
        sendIdentity();

        // Subscribe to topics
        mqttClient.subscribe(makeTopic("identify", true).c_str());
        mqttClient.subscribe(makeTopic("program").c_str());
        mqttClient.subscribe(makeTopic("program", true).c_str());
        mqttClient.subscribe(makeTopic("bpm").c_str());
        mqttClient.subscribe(makeTopic("bpm", true).c_str());

        // Reset number of attempts and enable the display
        connectionAttempts = 0;
        enableDisplay();
      } else {
        Serial.printf(
          "failed to connect to MQTT server: rc=%d, trying again in %d seconds\n",
          mqttClient.state(),
          connectionDelay / 1000
        );

        // Enable the display if the connection attempts exceed the max allowed
        if (++connectionAttempts > MAX_CONNECTION_ATTEMPTS) {
          if (!shouldRunDisplay) {
            Serial.printf(
              "Unable to connect to MQTT server in %d attempts. Enabling display and slowing requests.\n",
              MAX_CONNECTION_ATTEMPTS
            );
          }
          enableDisplay();
        } else {
          if (shouldRunDisplay) {
            Serial.println("Initial attempt to connect to MQTT server, disabling display.");
          }
          disableDisplay();
        }
      }
      connectTimer = millis();
    }

    // While connecting, blink the light. Don't blink if the display is active
    if (!shouldRunDisplay && updateTimeDiff > CONNECTING_BLINK_DELAY) {
      ledOn = !ledOn;
      strip.setPixelColor(1, hsi2rgbw(180, 1, ledOn ? LED_INTENSITY : 0));
      strip.show();

      updateTimer = millis();
    }
  }
}


// =----------------------------------------------------------------------------------= Display =--=

void setupNeopixels() {
  strip.begin();
  disableDisplay();
  strip.show();
}

void enableDisplay() {
  shouldRunDisplay = true;
}

void disableDisplay() {
  shouldRunDisplay = false;
  uint32_t color = hsi2rgbw(0, 0, 0);
  for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, color); }
}

void setBPM(String bpm) {
  // Convert Beats per Minute to Beats per Second as a 1-time calculation on save
  currentBPS = atoi(bpm.c_str()) / 60;
  Serial.printf("Setting current BPM to %s\n", bpm.c_str());
}

void setProgram(int program) {
  if (program >= 0 && program < PROGRAM_COUNT && program != currentProgram) {
    currentProgram = program;
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

void nextProgram() {
  setProgram((currentProgram + 1) % PROGRAM_COUNT); // Next with loop around
}

void displayLoop(bool first = false) {
  (*renderFunc[currentProgram])(first);
}


// =---------------------------------------------------------------------------------= Programs =--=

// Program: White
// Steady pure white light for ambient lighting
void runProgramWhite(bool first) {
  static unsigned long updateTimer = millis();

  unsigned long updateTimeDiff = millis() - updateTimer;
  if (first || updateTimeDiff > FRAME_DELAY_MS) {
    uint32_t color = hsi2rgbw(0, 0, LED_INTENSITY);
    for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, color); }
    strip.show();
    updateTimer = millis();
  }
}

// Program: Candle Flicker
// Random flicker of brightness and duration, maybe tint
// Inspiration:
//   - https://github.com/timpear/NeoCandle
//   - https://cpldcpu.com/2013/12/08/hacking-a-candleflicker-led/
// Algorithm:
// Candle LEDs follow a simple 16 level breakdown with 50% at full and equal weighting amongst the
// remaining top 12 levels. The original sampling was around 13-14fps, this will run at 30, so we
// could sample every other frame and interpolate between the two. The second algorith runs much
// faster at up to 100fps but the flicker and burn modes only flip about 6 times a second.
//
// The 8-LED method uses a gradient along the LED strip which is set at pre-determined dips in a
// random-like pattern that loops. The flicker/flutter/burn reduces the amount of green in the
// initial amber-yellow color to increase the percieved red, as if the candle is losing power. With
// HSI color space, this works out to moving towards red and increasing suration. This could be
// simulated by having a max amber and min red, and using linear interpolation along the Hue and
// Saturation values based on the output of the 4-15 random generator above.
void runProgramCandle(bool first) {
  static unsigned long updateTimer = millis();
  static bool getLevel = true;
  static uint8_t level = 15;
  static uint8_t lastLevel = 15;

  unsigned long updateTimeDiff = millis() - updateTimer;
  if (first || updateTimeDiff > FRAME_DELAY_MS) {
    float nowLevel;
    if (getLevel) {
      lastLevel = level;
      level = random(24);
      level = level >= 11 ? 15 : level + 4;
      nowLevel = level;
    } else {
      nowLevel = abs(lastLevel - level) / 2;
    }

    // green-high 21, 0.5; green-mid 19, 0.9; green-low 15, 0.9
    uint8_t hue = map(nowLevel, 0, 15, 15, 21);
    float saturation = floatmap(nowLevel, 0, 15, 0.9, 0.6);
    float intensity = map(nowLevel, 0, 15, LED_INTENSITY * 0.75, LED_INTENSITY);

    // Find the tip color and the edge color
    uint32_t centerColor = hsi2rgbw(hue, saturation, intensity);
    uint32_t edgeColor = hsi2rgbw(hue, saturation, intensity - 0.1);

    // Write them out
    for (int i = 0; i < NEOPIXEL_COUNT; ++i) {
      strip.setPixelColor(i, i == 1 ? centerColor : edgeColor);
    }
    strip.show();

    getLevel = !getLevel;
    updateTimer = millis();
  }
}

// Program: Rainbow Fade
// Cycle through the color wheel at full saturation
// Uses global rate value to control duration
void runProgramRainbow(bool first) {
  static unsigned long updateTimer = millis();
  static float hueOffset = 0.0;

  unsigned long updateTimeDiff = millis() - updateTimer;
  if (first || updateTimeDiff > FRAME_DELAY_MS) {
    uint32_t color = hsi2rgbw(hueOffset, 1, LED_INTENSITY);
    for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, color); }
    strip.show();

    // 100 bpm / 60 sec = 1.6667 bps. 30 fps / 1.667 bps = 18 frames per beat
    // Assume 4 count, so 360 deg / 72 frames = 5 deg per frame
    float degPerFrame = 360 / (BEAT_MULTIPLIER * DISPLAY_FPS / currentBPS);
    hueOffset = floatmod(360 + hueOffset - degPerFrame, 360);
    updateTimer = millis();
  }
}

// Program: Twinkle
// Light blue and white strobes randomly spaced
void runProgramTwinkle(bool first) {
  static unsigned long updateTimer = millis();

  unsigned long updateTimeDiff = millis() - updateTimer;
  if (first || updateTimeDiff > FRAME_DELAY_MS) {
    uint32_t color = hsi2rgbw(200, 1, LED_INTENSITY);
    for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, color); }
    strip.show();
    updateTimer = millis();
  }
}

// Program: Night Sky
// Shades of blue and purple rolling fade, twinkle strobes
void runProgramNight(bool first) {
  static unsigned long updateTimer = millis();

  unsigned long updateTimeDiff = millis() - updateTimer;
  if (first || updateTimeDiff > FRAME_DELAY_MS) {
    uint32_t color = hsi2rgbw(250, 1, LED_INTENSITY);
    for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, color); }
    strip.show();
    updateTimer = millis();
  }
}

// Program: Dance Mode
// Fade between random colors
// Uses global rate value to control duration
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
    // 100 bpm / 60 sec = 1.6667 bps. 30 fps / 1.667 bps = 18 frames per beat
    // Assume a 4 count for most music and fade over 4 beats? At 100 bpm, that's 72 frames
    int framesPerFade = DISPLAY_FPS / currentBPS * BEAT_MULTIPLIER;
    float offset = (danceTarget - danceCurrent) / framesPerFade;
    float hue = danceCurrent + (offset * danceOffset);

    uint32_t color = hsi2rgbw(hue, 1, LED_INTENSITY);
    for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, color); }
    strip.show();

    danceOffset++;
    if (danceOffset > framesPerFade) {
      danceCurrent = danceTarget; // copy
      danceTarget = random(360); // init
      danceOffset = 0;
    }

    updateTimer = millis();
  }
}


// =----------------------------------------------------------------------------------= Buttons =--=

void setupButton() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(BUTTON_PIN, HIGH);
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


// =-------------------------------------------------------------------------------------= WIFI =--=

// gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  uint32_t color = hsi2rgbw(240, 1, LED_INTENSITY);
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

// Finishing steps for after wifi may be complete
void finalizeWifi() {
  if (WiFi.status() != WL_CONNECTED){
    wifiFeaturesEnabled = false;
    Serial.print("Failed to connect to wifi. Playing failure animation then proceeding.");

    // Flash the bad news, then activate the display
    for (size_t j = 0; j < 6; j++) {
      uint32_t color = hsi2rgbw(j % 2 ? 0 : 240, 1, LED_INTENSITY);
      for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, color); }
      strip.show();
      delay(125);
    }
    enableDisplay();
  } else {
    wifiFeaturesEnabled = true;
    Serial.print("Connected to WiFi. Local IP: ");
    Serial.println(WiFi.localIP());
  }
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

void setupWifi() {
  if (WiFi.SSID() == "") {
    Serial.println("We haven't got any access point credentials, so get them now");
    wifiCaptivePortal();
  } else {
    // Force to station mode because if device was switched off while in access point mode it will
    // start up next time in access point mode.
    WiFi.mode(WIFI_STA);
    int connRes = WiFi.waitForConnectResult();
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
  setupNeopixels();
  setupFileSystem();
  setupButton();
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
  }

  buttonLoop();
  if (shouldRunDisplay) {
    displayLoop();
  }
}
