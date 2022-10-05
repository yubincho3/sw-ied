#define PIN_LED 7

#define ONE_SEC_AS_US 1000000.0

unsigned int period;
unsigned int sign = 1;
double duty;
double fade;

void set_period(unsigned int t_period) {
    period = t_period;
    fade = 100.0 / ((ONE_SEC_AS_US / (double)period) / 2.0);
}

void set_duty(double t_duty) {
    duty = t_duty;
}

unsigned int getNextDuty(unsigned int duty, double fade, unsigned int sign) {
    unsigned int ret = duty + fade * sign;

    if (ret < 0 || ret > 100) {
        sign = -sign;
        ret = duty + fade * sign;
    }

    return ret;
}

void pwmWrite(uint8_t pin) {
    unsigned int delay_us = period * duty / 100;
    
    digitalWrite(pin, LOW);
    delayMicroseconds(delay_us);
    digitalWrite(pin, HIGH);
    delayMicroseconds(period - delay_us);
}

void setup() {
    pinMode(PIN_LED, OUTPUT);
    set_period(1000);
}

void loop() {
    pwmWrite(PIN_LED);

    unsigned int newDuty = getNextDuty(duty, fade, sign);
    set_duty(newDuty);
}
