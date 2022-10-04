// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12   // sonar sensor TRIGGER
#define PIN_ECHO 13   // sonar sensor ECHO

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300.0   // maximum distance to be measured (unit: mm)
#define _BRIGHT_MIN 255

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficent to convert duration to distance

unsigned long lastSamplingTime;   // unit: msec
unsigned int lastBrightness = 255;

// 밝기는 거리 100 미만과 300 초과에서 최소, 200에서 최대값을 가지는 피라미드형 그래프임.
// 거리값을 [0, 255] 범위의 밝기값으로 변환하여 반환함.
unsigned int calculateBrightness(double distance) {
    if (distance < _DIST_MIN || distance > _DIST_MAX)
        return _BRIGHT_MIN;

    double duty = abs(distance - 200.0) / 100.0;

    return (unsigned int)(duty * 255.0);
}

void setup() {
    // initialize GPIO pins
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_TRIG, OUTPUT);    // sonar TRIGGER
    pinMode(PIN_ECHO, INPUT);     // sonar ECHO
    digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 

    // initialize serial port
    Serial.begin(57600);
}

void loop() {
    // wait until next sampling time. 
    // millis() returns the number of milliseconds since the program started.
    // Will overflow after 50 days.
    if (millis() < (lastSamplingTime + INTERVAL))
        return;

    // update last sampling time
    lastSamplingTime += INTERVAL;

    double distance = USSMeasure(PIN_TRIG, PIN_ECHO); // read distance
    unsigned int brightness = calculateBrightness(distance);

    if (distance < _DIST_MIN) {
        distance = _DIST_MIN - 10;
        analogWrite(PIN_LED, lastBrightness); // 이전 밝기값 사용
    } else if (distance > _DIST_MAX) {
        distance = _DIST_MAX + 10;
        analogWrite(PIN_LED, lastBrightness); // 이전 밝기값 사용
    } else
        analogWrite(PIN_LED, brightness);

    lastBrightness = brightness;

    // output the distance to the serial port
    Serial.print("Min:");       Serial.print(_DIST_MIN);
    Serial.print(",distance:"); Serial.print(distance);
    Serial.print(",Max:");      Serial.print(_DIST_MAX);
    Serial.println();
}

// get a distance reading from USS. return value is in millimeter.
double USSMeasure(unsigned int TRIG, unsigned int ECHO)
{
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(PULSE_DURATION);
    digitalWrite(TRIG, LOW);
    
    return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm

    // Pulse duration to distance conversion example (target distance = 17.3m)
    // - round trip distance: 34.6m
    // - expected pulse duration: 0.1 sec, or 100,000us
    // - pulseIn(ECHO, HIGH, timeout) * 0.001 * 0.5 * SND_VEL
    //        = 100,000 micro*sec * 0.001 milli/micro * 0.5 * 346 meter/sec
    //        = 100,000 * 0.001 * 0.5 * 346 * micro * sec * milli * meter
    //                                        ----------------------------
    //                                         micro * sec
    //        = 100 * 173 milli*meter = 17,300 mm = 17.3m
    // pulseIn() returns microseconds.
}
