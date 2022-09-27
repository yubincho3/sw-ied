#define ONE_SEC_AS_US 1000000
#define PIN_LED 7

long period;
double duty;
double fade;

void set_period(long t_period) {
    period = t_period;
    fade = 100.0 / (double)((ONE_SEC_AS_US / period) / 2);
}

void set_duty(double t_duty) {
    duty = t_duty;
}

void setup() {
    pinMode(PIN_LED, OUTPUT);
    set_period(100);
}

void loop() {
    long delay_us = period * duty / 100;
    
    digitalWrite(PIN_LED, LOW);
    delayMicroseconds(delay_us);
    digitalWrite(PIN_LED, HIGH);
    delayMicroseconds(period - delay_us);

    if (duty + fade < 0 || duty + fade > 100)
        fade = -fade;
    
    set_duty(duty + fade);
}
