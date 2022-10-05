// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 10      // minimum distance to be measured (unit: mm)
#define _DIST_MAX 350     // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficent to convert duration to distance

#define _EMA_ALPHA 0.325  // EMA weight of new sample (range: 0 to 1)
                          // Setting EMA to 1 effectively disables EMA filter.

// global variables
unsigned long last_sampling_time;   // unit: msec
double dist_prev = _DIST_MAX;        // Distance last-measured
double dist_ema;                     // EMA distance

double get_ema_filtered_dist(double dist) {
    double ret = dist * _EMA_ALPHA + (1.0 - _EMA_ALPHA) * dist_ema;
    return ret;
}

void setup() {
    // initialize GPIO pins
    pinMode(PIN_LED,OUTPUT);
    pinMode(PIN_TRIG,OUTPUT);
    pinMode(PIN_ECHO,INPUT);
    digitalWrite(PIN_TRIG, LOW);

    // initialize serial port
    Serial.begin(57600);

    // initialize last sampling time
    last_sampling_time = 0;
}

void loop() {
    // wait until next sampling time. 
    // millis() returns the number of milliseconds since the program started. 
    // Will overflow after 50 days.
    if (millis() < last_sampling_time + INTERVAL)
        return;

    // update last sampling time
    last_sampling_time += INTERVAL;

    // get a distance reading from the USS
    double dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);
    
    if (dist_raw < _DIST_MIN) {
        dist_raw = dist_prev;           // Set Prev Value
        digitalWrite(PIN_LED, 1);       // LED OFF
    } else if (dist_raw > _DIST_MAX) {
        dist_raw = dist_prev;           // Set Prev Value
        digitalWrite(PIN_LED, 1);       // LED OFF
    } else {    // In desired Range
        digitalWrite(PIN_LED, 0);       // LED ON
        dist_prev = dist_raw;
    }

    // Modify the below line to implement the EMA equation
    dist_ema = get_ema_filtered_dist(dist_raw);

    // output the distance to the serial port
    Serial.print("Min:");   Serial.print(_DIST_MIN);
    Serial.print(",raw:");  Serial.print(dist_raw);
    Serial.print(",ema:");  Serial.print(dist_ema);
    Serial.print(",Max:");  Serial.print(_DIST_MAX);
    Serial.println();
}

// get a distance reading from USS. return value is in millimeter.
double USS_measure(int TRIG, int ECHO)
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
