#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10
#define PIN_VAR A3

// configurable parameters
#define _INTERVAL_SERVO   20 // servo interval (unit: ms)
#define _INTERVAL_SERIAL  500 // serial interval (unit: ms)
#define _DUTY_MIN 500
#define _DUTY_NEU 1500
#define _DUTY_MAX 2500
#define _ADJ_RANGE 1000

// global variables
Servo myservo;

int var_initial;  // Inital resistance value of the potentiometer
int duty_adj;
unsigned long last_sampling_time_servo;  // unit: msec
unsigned long last_sampling_time_serial; // unit: msec

bool event_servo, event_serial; // event triggered?


void setup() {
    myservo.attach(PIN_SERVO);  // Initialize servo
    Serial.begin(1000000); // Initialize serial controller 
    var_initial = analogRead(PIN_VAR); // Read the initial knob position of the potentiometer
}

void loop() {
    unsigned long time_curr = millis();

    // wait until next event time
    if (time_curr >= (last_sampling_time_servo + _INTERVAL_SERVO)) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
    }
    if (time_curr >= (last_sampling_time_serial + _INTERVAL_SERIAL)) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
    }

    if (event_servo) {
        event_servo = false;

        int var_diff = analogRead(PIN_VAR) - var_initial; // Measure the knob displacement of the potentiometer
        duty_adj = _DUTY_NEU + var_diff / 512.0 * _ADJ_RANGE;

        if (duty_adj > _DUTY_MAX)
            duty_adj = _DUTY_MAX; // for servo arm protection
        if (duty_adj < _DUTY_MIN)
            duty_adj = _DUTY_MIN;

        myservo.writeMicroseconds(duty_adj);
    }

    if (event_serial) {
        event_serial = false;
        // output the read value to the serial port
        Serial.print("Min:500,DUTY:"); Serial.print(duty_adj);  
        Serial.println(",Max:2500");
    }
}
