#define PIN_LED 9

int brightness = 0;
int fade = 5;

void setup() {
    pinMode(PIN_LED, OUTPUT);
}

void loop() {
    analogWrite(PIN_LED, brightness);

    brightness += fade;

    if (brightness <= 0 || brightness >= 255)
        fade = -fade;
    
    delay(30);
}
