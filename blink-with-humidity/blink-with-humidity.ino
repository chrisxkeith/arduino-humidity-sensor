 /*
  Test humidity sensors.
  D2 : https://www.amazon.com/Digital-Temperature-Humidity-measure-Arduino/dp/B00R4KGG6Q
  D7 : https://www.amazon.com/Digital-Temperature-Humidity-measure-Arduino/dp/B018JO5BRK
  https://www.elecrow.com/wiki/index.php?title=Temperature_%26_Humidity_Sensor
  - library (.h/.cpp) in ~/Documents/Arduino/libraries/Humidity_Temperature_Sensor
  - ino in ~/Documents/Github/arduino-humidity-sensor/blink-with-humidity
*/

#ifdef PARTICLE_H
  void log(char* message) {
    Particle.publish("", message, 1, PRIVATE);
  }
#else
  unsigned long minSecToMillis(unsigned long minutes, unsigned long seconds) {
    return (minutes * 60 * 1000) + (seconds * 1000);
  }

  const long MAX_LONG = 2147483647;
  const unsigned long MAX_UNSIGNED_LONG = 4294967295;
  const unsigned int MAX_MESSAGE_SIZE = 96;
  char gMessage[MAX_MESSAGE_SIZE] = "";
  unsigned long previousLogTime = MAX_UNSIGNED_LONG;

  char* toFormattedInterval(unsigned long i) {
    unsigned long hours = i / minSecToMillis(60, 0);
    unsigned long remainder = i % minSecToMillis(60, 0);
    unsigned long minutes = remainder / minSecToMillis(1, 0);
    remainder = remainder % minSecToMillis(1, 0);
    unsigned long seconds = remainder / minSecToMillis(0, 1);
    snprintf(gMessage, MAX_MESSAGE_SIZE, "%02i:%02i:%02i", (int)hours, (int)minutes, (int)seconds);
    return gMessage;
  }

  char logLine[MAX_MESSAGE_SIZE];
  void log(char* message) {
      unsigned long logInterval = 0;
      if (previousLogTime != MAX_UNSIGNED_LONG) {
        logInterval = (millis() - previousLogTime) / 1000;
      }
      previousLogTime = millis();
      snprintf(logLine, MAX_MESSAGE_SIZE, "%s\t%s",
              toFormattedInterval(previousLogTime), message);
      Serial.println(logLine);
  }
#endif

#include "DHT.h"

#define DHTPIN 2     // what pin we're connected to

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11 
//#define DHTTYPE DHT22   // DHT 22  (AM2302)
#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

DHT dht1(2, DHT21);
float dht1_h = -1.0e-100;
float dht1_t = -1.0e-100;

DHT dht2(7, DHT22);
float dht2_h = -1.0e-100;
float dht2_t = -1.0e-100;

void setup() {
  Serial.begin(9600);
  dht1.begin();
  dht2.begin();
}

bool hasDelta(float a, float b, float delta) {
  return (abs(a - b) > delta);
}

void readSensor(float* prev_h, float* prev_t, float h, float t, char* sensor_name) {
  // check if returns are valid, if they are NaN (not a number) then something went wrong!
  String s = sensor_name;
  s.concat("\t");
  if (isnan(t)) {
    s.concat("Failed to read temperature");
    log(s.c_str());
  } else if (isnan(h)) {
    s.concat("Failed to read humidity");
    log(s.c_str());
  } else {
    if (hasDelta(*prev_t, t, 0.5) || hasDelta(*prev_h, h, 0.5)) {
      s.concat("Humidity: ");
      s.concat(h);
      s.concat(" %\t");
      s.concat("Temperature: ");
      s.concat(t);
      s.concat(" *C");
      log(s.c_str());
      *prev_t = t;
      *prev_h = h;
    }
  }
}

void loop() {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  readSensor(&dht1_h, &dht1_t, dht1.readHumidity(), dht1.readTemperature(), "Smakn DHT21 AM2301");
  readSensor(&dht2_h, &dht2_t, dht2.readHumidity(), dht2.readTemperature(), "SMAKN DHT22 AM2302");
}

