/*
  Test humidity sensors.
  A0 : nothing (baseline)
  A1 : https://www.amazon.com/Digital-Temperature-Humidity-measure-Arduino/dp/B00R4KGG6Q
  A2 : https://www.amazon.com/Digital-Temperature-Humidity-measure-Arduino/dp/B018JO5BRK
*/
long masterCount = 0;
long baselineCount = 0;
long baselineAverage = 0;
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

void setup() {
  Serial.begin(9600);
  log("masterCount\tbase avg\tHT21 avg\tDHT22 avg");

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  
  pinMode(0, INPUT);
  pinMode(2, INPUT);
  pinMode(A2, INPUT);
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

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);
  
  readHumiditySensor("baseline", A0, &baselineCount, &baselineAverage);
  readHumiditySensor("HT21 AM2301", A1, &HT21Count, &HT21Average);
  readHumiditySensor("DHT22 AM2302", A2, &DHT22Count, &DHT22Average);
  if (masterCount % 100 == 0) {
    String msg = "";
    msg.concat(masterCount);
    msg.concat("\t\t");
    msg.concat(baselineAverage);
    msg.concat("\t\t");
    msg.concat(HT21Average);
    msg.concat("\t\t");
    msg.concat(DHT22Average);
    log(msg.c_str());  
  }
  masterCount++;
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(100);
}
