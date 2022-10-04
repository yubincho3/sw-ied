#define PIN_LED 7

#define ONE_SEC_AS_US 1000000.0

unsigned int period;
double duty;
double fade;

void set_period(unsigned int t_period) {
    period = t_period;
    fade = 100.0 / ((ONE_SEC_AS_US / (double)period) / 2.0);
}

void set_duty(double t_duty) {
    if (t_duty < 0 || t_duty > 100)
        fade = -fade;

    duty += fade;
}

void setup() {
    pinMode(PIN_LED, OUTPUT);
    set_period(100);
}

void loop() {
    unsigned int delay_us = period * duty / 100;
    
    digitalWrite(PIN_LED, LOW);
    delayMicroseconds(delay_us);
    digitalWrite(PIN_LED, HIGH);
    delayMicroseconds(period - delay_us);
    
    set_duty(duty + fade);
}
