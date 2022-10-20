// use assert function
#define __ASSERT_USE_STDERR
#include <assert.h>

#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10
#define PIN_LED 9

// configurable parameters
#define _DUTY_MIN 550  // servo full clock-wise position (0 degree)
#define _DUTY_MAX 2400 // servo full counter-clockwise position (180 degree)

#define INTERVAL 20    // servo update interval (unit: msec)

Servo myservo;

// handle diagnostic informations given by assertion and abort program execution
// source: https://gist.github.com/jlesech/3089916
void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp) {
    // transmit diagnostic informations through serial link. 
    Serial.println(__func);
    Serial.println(__file);
    Serial.println(__lineno, DEC);
    Serial.println(__sexp);
    Serial.flush();    
    // abort program execution.
    abort();
}

void setup() {
    // initialize GPIO pins
    myservo.attach(PIN_SERVO); 

    // initialize serial port
    Serial.begin(57600);
}

// move servo from start  to end  at speed
// start must be less than or equal to end
// units: start(degree), end(degree), speed(degree/second)
void moveServo(int start, int end, double speed) {
    assert(start <= end);

    myservo.write(start);

    // set target duty time as end degree
    const double duty_target = (double)(_DUTY_MAX - _DUTY_MIN) * (end / 180.0) + _DUTY_MIN;

    // convert angular speed into duty change per interval.
    const double duty_change_per_interval = (double)(_DUTY_MAX - _DUTY_MIN) * (speed / 180.0) * (INTERVAL / 1000.0);

    // set current duty time as start degree
    double duty_curr = (double)(_DUTY_MAX - _DUTY_MIN) * (start / 180.0) + _DUTY_MIN;

    // if servo reached end, this value will be set to true
    bool servo_reached_end = false;

    // initialize next sampling time (unit: ms)
    unsigned long next_sampling_time = millis();

    while (!servo_reached_end) {
        // wait until next sampling time. 
        if (millis() < next_sampling_time)
            continue;

        // update next sampling time
        next_sampling_time += INTERVAL;

        // adjust duty_curr toward duty_target by duty_change_per_interval
        if (duty_target > duty_curr) {
            duty_curr += duty_change_per_interval;

            // servo reached end
            if (duty_curr > duty_target) {
                duty_curr = duty_target;
                servo_reached_end = true;
            }
        }

        // update servo position
        myservo.writeMicroseconds((int)duty_curr);

        // output the read value to the serial port
        // Serial.print("Min:1000");
        // Serial.print(",duty_target:"); Serial.print(duty_target);
        // Serial.print(",duty_curr:");   Serial.print(duty_curr);
        // Serial.println(",Max:2000");
    }
}

void loop() {
    // 0 to 180 with speed 3
    myservo.write(0);
    delay(1000);

    analogWrite(PIN_LED, 0);
    moveServo(0, 180, 3);
    analogWrite(PIN_LED, 255);
    delay(1000);

    // 0 to 90 with speed 0.3
    myservo.write(0);
    delay(1000);

    analogWrite(PIN_LED, 0);
    moveServo(0, 90, 0.3);
    analogWrite(PIN_LED, 255);
    delay(1000);

    while (true);
}
