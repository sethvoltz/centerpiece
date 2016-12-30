// =--------------------------------------------------------------------------------= Libraries =--=

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include <Encoder.h>
#include <math.h>


// =----------------------------------------------------------------------------= Configuration =--=

// Programs
#define PROGRAM_COUNT   6

// Wifi
#define WLAN_SSID       "get_chippy_with_it"
#define WLAN_PASS       "bananaphone"

// MQTT
#define MQTT_SERVER     "172.20.0.1"
#define MQTT_PORT       1883
#define MQTT_ROOT       "centerpiece"

// Neopixel
#define NEOPIXEL_PIN    15
#define NEOPIXEL_COUNT  24
#define DEG_TO_RAD(X) (M_PI*(X)/180)

// Encoder
#define ENCODER_PINA    12
#define ENCODER_PINB    14
#define ENCODER_TICKS   4


// =----------------------------------------------------------------------------------= Globals =--=

// Programs
int currentProgram = 0;
int displayProgram = 0;

// WiFi Client
WiFiClient wifiClient;

// MQTT
PubSubClient mqttClient(wifiClient);
String clientId(String(ESP.getChipId(), HEX));

// Neopixel
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRBW + NEO_KHZ800);

// Encoder
Encoder encoder(ENCODER_PINA, ENCODER_PINB);
long encoderPosition = -999;
int dialPosition = 0;
int encoderSize = PROGRAM_COUNT * ENCODER_TICKS;


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

void setProgram(String program) {
  Serial.print("Setting program to "); Serial.println(program);
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void mqttConnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    strip.setPixelColor(0, hsi2rgbw(0, 1, 0.05));
    strip.show();
    Serial.print("Attempting MQTT connection...");

    // Attempt to connect
    if (mqttClient.connect(clientId.c_str())) {
      strip.setPixelColor(0, hsi2rgbw(120, 1, 0.05));
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
  for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, hsi2rgbw(0, 1, 0.05)); }
  strip.show();
}

void runProgramCandle(bool first) {
  for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, hsi2rgbw(60, 1, 0.05)); }
  strip.show();
}

void runProgramRainbow(bool first) {
  for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, hsi2rgbw(120, 1, 0.05)); }
  strip.show();
}

void runProgramTwinkle(bool first) {
  for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, hsi2rgbw(180, 1, 0.05)); }
  strip.show();
}

void runProgramNight(bool first) {
  for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, hsi2rgbw(240, 1, 0.05)); }
  strip.show();
}

void runProgramDance(bool first) {
  for (int i = 0; i < NEOPIXEL_COUNT; ++i) { strip.setPixelColor(i, hsi2rgbw(300, 1, 0.05)); }
  strip.show();
}


// =---------------------------------------------------------------------------= Setup and Loop =--=

void updateDisplay(bool first = false) {
  switch(displayProgram) {
    case 0: runProgramWhite(first); break;
    case 1: runProgramCandle(first); break;
    case 2: runProgramRainbow(first); break;
    case 3: runProgramTwinkle(first); break;
    case 4: runProgramNight(first); break;
    case 5: runProgramDance(first); break;
  }
}

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
    long newDial = encoderPosition / ENCODER_TICKS;

    if (newDial != dialPosition) {
      dialPosition = newDial;
      Serial.print("Dial = ");
      Serial.println(dialPosition);
      displayProgram = dialPosition;
    }
  }
}

void setupWifi() {
  int pixel = 0;

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (pixel < NEOPIXEL_COUNT) {
      strip.setPixelColor(pixel++, hsi2rgbw(0, 1, 0.05));
      strip.show();
    }
  }
  Serial.println();

  randomSeed(micros());
  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  Serial.print("Unique ID: "); Serial.println(clientId.c_str());

  strip.setPixelColor(0, hsi2rgbw(60, 1, 0.05));
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

void setup() {
  Serial.begin(115200);
  delay(100);

  setupNeopixels();
  setupWifi();
  setupMQTT();
}

void loop() {
  if (!mqttClient.connected()) { mqttConnect(); }
  mqttClient.loop();
  encoderLoop();
  updateDisplay();
}
