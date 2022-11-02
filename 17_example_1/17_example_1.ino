#include <Servo.h>

// Arduino pin assignment

#define PIN_POTENTIOMETER 3 // Potentiometer at Pin A3
// Add IR Sensor Definition Here !!!
#define PIN_SERVO 10

#define _DUTY_MIN 553  // servo full clock-wise position (0 degree)
#define _DUTY_NEU 1476 // servo neutral position (90 degree)
#define _DUTY_MAX 2399 // servo full counter-clockwise position (180 degree)

#define LOOP_INTERVAL 50   // Loop Interval (unit: msec)

Servo myservo;
unsigned long last_loop_time;   // unit: msec

void setup()
{
    myservo.attach(PIN_SERVO); 
    myservo.writeMicroseconds(_DUTY_NEU);
    
    Serial.begin(57600);
}

void loop()
{
    unsigned long time_curr = millis();
    int a_value, duty;

    // wait until next event time
    if (time_curr < (last_loop_time + LOOP_INTERVAL))
        return;
    last_loop_time += LOOP_INTERVAL;

    // Remove Next line !!!
    a_value = analogRead(PIN_POTENTIOMETER);
    // Read IR Sensor value !!!
    // Convert IR sensor value into distance !!!
    // we need distance range filter here !!!
    // we need EMA filter here !!!

    // map distance into duty
    duty = map(a_value, 0, 1023, _DUTY_MIN, _DUTY_MAX);
    myservo.writeMicroseconds(duty);

    // print IR sensor value, distnace, duty !!!
    Serial.print("ADC Read: "); Serial.print(a_value);
    Serial.print(" = ");
    Serial.print((a_value / 1024.0) * 5.0);
    Serial.print(" Volt => Duty : ");
    Serial.print(duty);
    Serial.println("usec");
}
