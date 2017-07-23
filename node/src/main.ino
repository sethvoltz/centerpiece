// =--------------------------------------------------------------------------------= Libraries =--=

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>


// =----------------------------------------------------------------------------= Configuration =--=

// Programs
#define PROGRAM_COUNT   6
#define FRAME_DELAY_MS  33  // 30 fps
#define LED_INTENSITY   0.5 // 50% power

// Wifi
#define WLAN_SSID       "get_chippy_with_it"
#define WLAN_PASS       "bananaphone"

// MQTT
#define MQTT_SERVER     "172.20.0.1"
#define MQTT_PORT       1883
#define MQTT_ROOT       "centerpiece"

// Neopixel
#define NEOPIXEL_PIN    14
#define NEOPIXEL_COUNT  5
// #define DEG_TO_RAD(X) (M_PI*(X)/180)

// Buttons
#define BUTTON_PIN      12
#define DEBOUNCE_MS     30
#define HOLD_TIME_MS    3000


// =----------------------------------------------------------------------------------= Globals =--=

// Programs
int currentProgram = 0;

// WiFi Client
WiFiClient wifiClient;

// MQTT
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


// =-------------------------------------------------------------------------------= Prototypes =--=

void updateDisplay(bool first);


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

  // return strip.Color(neopix_gamma[r], neopix_gamma[g], neopix_gamma[b], neopix_gamma[w]);
  return strip.Color(r, g, b, w);
}


// =---------------------------------------------------------------------------= MQTT Callbacks =--=

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
    // setupWifi(true);
    Serial.println("TODO: Setup Wifi");
  }

  buttonLastValue = buttonValue;
}

void setupWifi() {
  int pixel = 0;
  int on = true;

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (pixel < NEOPIXEL_COUNT) {
      strip.setPixelColor(pixel++, hsi2rgbw(0, (on ? 1 : 0), LED_INTENSITY));
      strip.show();
    } else {
      on = !on;
      pixel = 0;
    }
  }
  Serial.println();

  randomSeed(micros());
  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  Serial.print("Unique ID: "); Serial.println(clientId.c_str());

  strip.setPixelColor(0, hsi2rgbw(60, 1, LED_INTENSITY));
  strip.show();
}

void setupMQTT() {
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
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
  setupButton();
  setupNeopixels();
  setupWifi();
  setupMQTT();
}

void loop() {
  if (!mqttClient.connected()) { mqttConnect(); }
  mqttClient.loop();
  buttonLoop();
  updateDisplay();
}
