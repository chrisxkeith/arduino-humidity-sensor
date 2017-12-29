/*
  Test humidity sensors.
  D2 : https://www.amazon.com/Digital-Temperature-Humidity-measure-Arduino/dp/B00R4KGG6Q
  D7 : https://www.amazon.com/Digital-Temperature-Humidity-measure-Arduino/dp/B018JO5BRK
  https://www.elecrow.com/wiki/index.php?title=Temperature_%26_Humidity_Sensor
  - NOPE, brute force include for now : library (.h/.cpp) in ~/Documents/Arduino/libraries/Humidity_Temperature_Sensor
  - ino in ~/Documents/Github/arduino-humidity-sensor/blink-with-humidity
*/

// #define PARTICLE

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#ifdef PARTICLE
void log(const char* message) {
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
  
  void log(const char* message) {
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

/* DHT library

MIT license
written by Adafruit Industries
*/

// how many timing transitions we need to keep track of. 2 * number bits + extra
#define MAXTIMINGS 85

#define DHT11 11
#define DHT22 22
#define DHT21 21
#define AM2301 21

class DHT {
 private:
  uint8_t data[6];
  uint8_t _pin, _type, _count;
  boolean read(void);
  unsigned long _lastreadtime;
  boolean firstreading;

 public:
  DHT(uint8_t pin, uint8_t type, uint8_t count=6);
  void begin(void);
  float readTemperature(bool S=false);
  float convertCtoF(float);
  float readHumidity(void);

};

DHT::DHT(uint8_t pin, uint8_t type, uint8_t count) {
  _pin = pin;
  _type = type;
  _count = count;
  firstreading = true;
}

void DHT::begin(void) {
  // set up the pins!
  pinMode(_pin, INPUT);
  digitalWrite(_pin, HIGH);
  _lastreadtime = 0;
}

//boolean S == Scale.  True == Farenheit; False == Celcius
float DHT::readTemperature(bool S) {
  float f;

  if (read()) {
    switch (_type) {
    case DHT11:
      f = data[2];
      if(S)
        f = convertCtoF(f);

      return f;
    case DHT22:
    case DHT21:
      f = data[2] & 0x7F;
      f *= 256;
      f += data[3];
      f /= 10;
      if (data[2] & 0x80)
  f *= -1;
      if(S)
  f = convertCtoF(f);

      return f;
    }
  }
  return NAN;
}

float DHT::convertCtoF(float c) {
  return c * 9 / 5 + 32;
}

float DHT::readHumidity(void) {
  float f;
  if (read()) {
    switch (_type) {
    case DHT11:
      f = data[0];
      return f;
    case DHT22:
    case DHT21:
      f = data[0];
      f *= 256;
      f += data[1];
      f /= 10;
      return f;
    }
  }
  return NAN;
}


boolean DHT::read(void) {
  uint8_t laststate = HIGH;
  uint8_t counter = 0;
  uint8_t j = 0, i;
  unsigned long currenttime;

  // pull the pin high and wait 250 milliseconds
  digitalWrite(_pin, HIGH);
  delay(250);

  currenttime = millis();
  if (currenttime < _lastreadtime) {
    // ie there was a rollover
    _lastreadtime = 0;
  }
  if (!firstreading && ((currenttime - _lastreadtime) < 2000)) {
    return true; // return last correct measurement
    //delay(2000 - (currenttime - _lastreadtime));
  }
  firstreading = false;
  /*
    Serial.print("Currtime: "); Serial.print(currenttime);
    Serial.print(" Lasttime: "); Serial.print(_lastreadtime);
  */
  _lastreadtime = millis();

  data[0] = data[1] = data[2] = data[3] = data[4] = 0;

  // now pull it low for ~20 milliseconds
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  delay(20);
  cli();
  digitalWrite(_pin, HIGH);
  delayMicroseconds(40);
  pinMode(_pin, INPUT);

  // read in timings
  for ( i=0; i< MAXTIMINGS; i++) {
    counter = 0;
    while (digitalRead(_pin) == laststate) {
      counter++;
      delayMicroseconds(1);
      if (counter == 255) {
        break;
      }
    }
    laststate = digitalRead(_pin);

    if (counter == 255) break;

    // ignore first 3 transitions
    if ((i >= 4) && (i%2 == 0)) {
      // shove each bit into the storage bytes
      data[j/8] <<= 1;
      if (counter > _count)
        data[j/8] |= 1;
      j++;
    }

  }

  sei();

  /*
  Serial.println(j, DEC);
  Serial.print(data[0], HEX); Serial.print(", ");
  Serial.print(data[1], HEX); Serial.print(", ");
  Serial.print(data[2], HEX); Serial.print(", ");
  Serial.print(data[3], HEX); Serial.print(", ");
  Serial.print(data[4], HEX); Serial.print(" =? ");
  Serial.println(data[0] + data[1] + data[2] + data[3], HEX);
  */

  // check we read 40 bits and that the checksum matches
  if ((j >= 40) &&
      (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) ) {
    return true;
  }


  return false;

}

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
float dht1_h = -2000.0;
float dht1_t = -2000.0;

DHT dht2(7, DHT22);
float dht2_h = -2000.0;
float dht2_t = -2000.0;

void setup() {
  Serial.begin(9600);
  dht1.begin();
  dht2.begin();
}

bool hasDelta(float a, float b, float delta) {
  if (isnan(a) && !isnan(b)) {
    return true;
  }
  if (!isnan(a) && isnan(b)) {
    return true;
  }
  return (abs(a - b) > delta);
}

String buildString(String sensor_name, float h, float t) {
  String s = sensor_name;
  s.concat("\t");
  s.concat(h);
  s.concat(" %\t");
  s.concat(t);
  s.concat(" *C");
  return s;
}

bool checkDiff(float* prev_h, float* prev_t, float h, float t, float d) {
  // check if returns are valid, if they are NaN (not a number) then something went wrong!
  if (hasDelta(*prev_t, t, d) || hasDelta(*prev_h, h, d)) {
    *prev_t = t;
    *prev_h = h;
    return true;
  }
  return false;
}

void loop() {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht1.readHumidity();
  if (isnan(h)) {
    h = 0.0;
  }
  float t = dht1.readTemperature();
  if (isnan(t)) {
    t = 0.0;
  }
  String s1 = "";
  bool b1 = checkDiff(&dht1_h, &dht1_t, h, t, 1.0);
  s1.concat(buildString("Smakn DHT21 AM2301", h, t));
  h = dht2.readHumidity();
  if (isnan(h)) {
    h = 0.0;
  }
  t = dht2.readTemperature();
  if (isnan(t)) {
    t = 0.0;
  }
  bool b2 = checkDiff(&dht2_h, &dht2_t, h, t, 1.0);
  s1.concat("\t");
  s1.concat(buildString("SMAKN DHT22 AM2302", h, t));
  if (b1 || b2) {
    log(s1.c_str());
  }
}

