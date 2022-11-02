#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10
#define PIN_LED 9
#define PIN_IR 0

#define _DIST_MIN 100 // Sensor dist min (unit: mm)
#define _DIST_MAX 250 // Sensor dist max (unit: mm)

#define _DUTY_MIN 550  // servo full clock-wise position (0 degree)
#define _DUTY_NEU 1475 // servo neutral position (90 degree)
#define _DUTY_MAX 2400 // servo full counter-clockwise position (180 degree)

#define LOOP_INTERVAL 25   // Loop Interval (unit: msec)

#define _EMA_ALPHA 0.325

Servo myservo;
unsigned long last_loop_time;   // unit: msec

float a_value;

float dist_prev = _DIST_MIN;
float dist, dist_ema;

float getRawDist() {
    a_value = analogRead(PIN_IR);
    return (6762.0 / (a_value - 9.0) - 4.0) * 10.0 - 60.0;
}

float getEMAFilteredDist(float dist) {
    return dist * _EMA_ALPHA + (1.0 - _EMA_ALPHA) * dist_ema;
}

void setup()
{
    myservo.attach(PIN_SERVO); 
    myservo.writeMicroseconds(_DUTY_NEU);
    
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    Serial.begin(57600);
}

void loop()
{
    unsigned long time_curr = millis();

    // wait until next event time
    if (time_curr < (last_loop_time + LOOP_INTERVAL))
        return;

    last_loop_time += LOOP_INTERVAL;

    dist = getRawDist();

    // Apply range filter
    if (dist < _DIST_MIN || dist > _DIST_MAX) {
        dist = dist_prev;
        digitalWrite(PIN_LED, HIGH);
    } else {
        dist_prev = dist;
        digitalWrite(PIN_LED, LOW);
    }

    // Apply EMA filter
    dist_ema = getEMAFilteredDist(dist);

    // map distance into duty
    int duty = (dist_ema - (float)_DIST_MIN) * (float)(_DUTY_MAX - _DUTY_MIN) /
               (float)(_DIST_MAX - _DIST_MIN) + (float)_DUTY_MIN;
    myservo.writeMicroseconds(duty);

    // print IR sensor value, distnace, duty
    Serial.print("MIN:");    Serial.print(_DIST_MIN);
    Serial.print(",IR:");    Serial.print(a_value);
    Serial.print(",dist:");  Serial.print(dist);
    Serial.print(",ema:");   Serial.print(dist_ema);
    Serial.print(",servo:"); Serial.print(duty);
    Serial.print(",MAX:");   Serial.print(_DIST_MAX);
    Serial.println();
}
