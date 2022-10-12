#include <Servo.h>

#define PIN_SERVO 10

Servo myservo;

unsigned int degree;
int sign = 1;

void setup() {
    myservo.attach(PIN_SERVO); 
    myservo.write(0);
}

void loop() {
    delay(1000);
    myservo.write(180);
    delay(255);
    myservo.write(0);
}
