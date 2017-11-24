 /*
  Test humidity sensors.
  A1 : https://www.amazon.com/Digital-Temperature-Humidity-measure-Arduino/dp/B00R4KGG6Q
  A2 : https://www.amazon.com/Digital-Temperature-Humidity-measure-Arduino/dp/B018JO5BRK
  https://www.elecrow.com/wiki/index.php?title=Temperature_%26_Humidity_Sensor
  - library (.h/.cpp) in ~/Documents/Arduino/libraries/Humidity_Temperature_Sensor
  - ino in ~/Documents/Github/arduino-humidity-sensor/blink-with-humidity
*/

/*
long masterCount = 0;
long HT21Count = 0;
long HT21Average = 0;
long DHT22Count = 0;
long DHT22Average = 0;

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

// digital pin 2 has a pushbutton attached to it. Give it a name:
int pushButton = 2;

void setupHumiditySensors() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  log("masterCount\tHT21 avg\tDHT22 avg");
}

void setup() {
  Serial.begin(9600);

  // baseline validation that code has loaded and is running.
  pinMode(LED_BUILTIN, OUTPUT);

  // validation that digital input is working. Hold down pushbutton to get digital '1' input.
  pinMode(pushButton, INPUT);
  setupHumiditySensors();
}

void handleButton() {
  if (masterCount % 1000 == 0) {
    int buttonState = digitalRead(pushButton);
    // print out the state of the button:
    String s = "Button\t";
    s.concat(buttonState);
    log(s.c_str());
  }
}

void handlePot() {
  if (masterCount % 1000 == 0) {
    String s = "Pot\t";
    s.concat(analogRead(A0));
    log(s.c_str());
  }
}

void readHumiditySensor(String name, int pin, long* count, long* average) {
  // Wraps around to negative at 32K
  long val = analogRead(pin);
  if ((*average) * (*count) + val < 0) {
    (*count) = 0;
    (*average) = 0;
  }
  (*average) = (((*average) * (*count)) + val) / (*count + 1);
  (*count)++;
}

void handleHumiditySensors() {
  readHumiditySensor("HT21 AM2301", A1, &HT21Count, &HT21Average);
  readHumiditySensor("DHT22 AM2302", A2, &DHT22Count, &DHT22Average);
  if (masterCount % 1000 == 0) {
    String msg = "";
    msg.concat(masterCount);
    msg.concat("\t\t");
    msg.concat(HT21Average);
    msg.concat("\t\t");
    msg.concat(DHT22Average);
    log(msg.c_str());  
  }
}

void loop() {
  if (masterCount % 1000 == 0) {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  delay(1);

//  handleButton();
//  handlePot();
  handleHumiditySensors();

  if (masterCount % 2000 == 0) {
    digitalWrite(LED_BUILTIN, LOW);
  }
  delay(1);
  masterCount++;
}
*/
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

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600); 
  Serial.println("DHTxx test!");
 
  dht.begin();
}

void loop() {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // check if returns are valid, if they are NaN (not a number) then something went wrong!
  if (isnan(t) || isnan(h)) {
    Serial.println("Failed to read from DHT");
  } else {
    Serial.print("Humidity: "); 
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("Temperature: "); 
    Serial.print(t);
    Serial.println(" *C");
  }
}
