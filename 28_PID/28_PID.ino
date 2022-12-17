#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

// Event interval parameters
//////////////// DO NOT modify below section!! /////////////////////////        
#define _INTERVAL_DIST    20 // distance sensor interval (unit: ms)
#define _INTERVAL_SERVO   20 // servo interval (unit: ms)
#define _INTERVAL_SERIAL  80 // serial interval (unit: ms)
#define _INTERVAL_MOVE    10000 // mission interval (unit: ms)
////////////////////////////////////////////////////////////////////////

// EMA filter configuration for the IR distance sensor
#define _EMA_ALPHA 0.35  // EMA weight of new sample (range: 0 to 1)

// Servo adjustment
#define _DUTY_NEU 1535
#define _DUTY_MIN 1031
#define _DUTY_MAX 2150
#define _SERVO_ANGLE_DIFF 85 // servo angle difference between _DUTY_MIN and _DUTY_MAX (unit: degree)
#define _SERVO_SPEED 330  // servo speed limit (unit: degree/second)

// PID parameters
#define _DIST_TARGET 155 // center of the rail (unit: mm)
#define _KP 4.95    // proportional gain
#define _KD 417     // derivative gain
#define _KI 0.17    // integral gain 0.22 0.28 0.17
#define ITERM_TRIGGER_DIST 25 // trigger distance of integral control(unit: mm)

// When stable_count is bigger than this values, pterm or iterm changes.
#define MAX_RELAX_ITERM_COUNT 10
#define MAX_HALF_ITERM_COUNT  15
#define MAX_ZERO_ITERM_COUNT  40
#define MAX_RELAX_PTERM_COUNT 8
#define MAX_HALF_PTERM_COUNT  22
#define MAX_ZERO_PTERM_COUNT  45
// #define _ITERM_MAX 250 // uncomment if necessary

//////////////// DO NOT modify below section!! /////////////////////////        
unsigned long error_sum, error_cnt, toggle_cnt;
////////////////////////////////////////////////////////////////////////

// global variables
float dist_filtered, dist_ema; // unit: mm
Servo myservo;

int duty_change_per_interval; // maximum duty difference per interval
int duty_target;    // Target duty
int duty_curr;      // Current duty
int dist_target;
unsigned long last_sampling_time_dist;   // unit: msec
unsigned long last_sampling_time_servo;  // unit: msec
unsigned long last_sampling_time_serial; // unit: msec
unsigned long last_sampling_time_move;   // unit: msec
bool event_dist, event_servo, event_serial; // event triggered?
float error_curr, error_prev, control, pterm, dterm, iterm;
int stable_cnt; // the count of stable error(less than 3mm)

void setup()
{
    // initialize GPIO pins
    pinMode(PIN_LED, OUTPUT);
    myservo.attach(PIN_SERVO);

    // set default value of gpio
    myservo.writeMicroseconds(_DUTY_MIN);
    digitalWrite(PIN_LED, HIGH);
    delay(2500);

    duty_target = _DUTY_NEU;
    duty_curr = _DUTY_MIN;

    // convert angular speed into duty change per interval.
    duty_change_per_interval =
        (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / (float) _SERVO_ANGLE_DIFF) * (_INTERVAL_SERVO / 1000.0);

    // initialize serial port
    Serial.begin(1000000);  
    //////////////// DO NOT modify below section!! /////////////////////////        
    dist_target = 55;
    ////////////////////////////////////////////////////////////////////////
}

void loop()
{
    unsigned long time_curr = millis();

    // wait until next event time
    if (time_curr >= (last_sampling_time_dist + _INTERVAL_DIST)) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
    }
    if (time_curr >= (last_sampling_time_servo + _INTERVAL_SERVO)) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
    }
    if (time_curr >= (last_sampling_time_serial + _INTERVAL_SERIAL)) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
    }

    // If the target is changed, the iterm is initialized to 0.
    if (time_curr >= (last_sampling_time_move + _INTERVAL_MOVE))
        iterm = 0;

//////////////// DO NOT modify below section!! /////////////////////////        
    if (time_curr >= (last_sampling_time_move + _INTERVAL_MOVE)) {
        last_sampling_time_move += _INTERVAL_MOVE;
        dist_target = 310 - dist_target;

        if (++toggle_cnt == 5) {
            Serial.println("----------------------------------------------------"),
            Serial.print("ERR_cnt:"),
            Serial.print(error_cnt);
            Serial.print(",ERR_sum:"),
            Serial.print(error_sum);
            Serial.print(",ERR_ave:"),
            Serial.println(error_sum / (float) error_cnt);
            exit(0);
        }
    }
////////////////////////////////////////////////////////////////////////

    if (event_dist) {
        event_dist = false;

        // Get a distance reading from the distance sensor
        dist_filtered = volt_to_distance(ir_sensor_filtered(110, 0.5));
        dist_ema = _EMA_ALPHA * dist_ema + (1.0 - _EMA_ALPHA) * dist_filtered;

        // Update PID variables
        error_curr = dist_target - dist_ema;

//////////////// DO NOT modify below section!! /////////////////////////
        if (time_curr >= last_sampling_time_move + 3000) {
            error_cnt++;
            if (abs(error_curr) > 3)
                error_sum += abs(error_curr);
        }

        digitalWrite(PIN_LED, abs(error_curr) > 3);
////////////////////////////////////////////////////////////////////////

        // ┌────────────────────────────────────────────────┐
        // │  The PID control section.                                                      │
        // │  Use PD control to approach the target close.                                  │
        // │  The I control is used for fine control.                                       │
        // │  If the error is less than ITERM_TRIGGER_DIST, Integral control is also used.  │
        // └────────────────────────────────────────────────┘

        pterm = _KP * error_curr;
        dterm = _KD * (error_curr - error_prev);

        bool flag = stable_cnt < MAX_ZERO_ITERM_COUNT;

        // Under 3mm is stable.
        if (abs(error_curr) <= 3) {
            if (++stable_cnt >= MAX_ZERO_ITERM_COUNT) {
                iterm = 0;
                flag = false;
            } else if (++stable_cnt >= MAX_HALF_ITERM_COUNT) {
                iterm /= 2.5;
                flag = false;
            } else if (stable_cnt >= MAX_RELAX_ITERM_COUNT)
                iterm *= 0.9;

            if (stable_cnt >= MAX_ZERO_PTERM_COUNT)
                pterm = 0;
            else if (stable_cnt >= MAX_HALF_PTERM_COUNT)
                pterm /= 1.5;
            else if (stable_cnt >= MAX_RELAX_PTERM_COUNT)
                pterm *= 0.9;
        } else
            stable_cnt = 0;

        // Change the value of iterm only if there is an error of less than or equal to ITERM_TRIGGER_DIST.
        if (abs(error_curr) <= ITERM_TRIGGER_DIST && flag)
            iterm += _KI * error_curr;

        // Write solution code of iterm windup if you need.
        // ...

        control = pterm + iterm + dterm;
        duty_target = _DUTY_NEU + control;
        error_prev = error_curr;
    }

    if (event_servo) {
        event_servo = false;

        // adjust duty_curr toward duty_target by duty_change_per_interval
        if (duty_target > duty_curr) {
            duty_curr += duty_change_per_interval;
            if (duty_curr > duty_target)
                duty_curr = duty_target;
        } else {
            duty_curr -= duty_change_per_interval;
            if (duty_curr < duty_target)
                duty_curr = duty_target;
        }
        // update servo position
        if (duty_curr > _DUTY_MAX)
            duty_curr = _DUTY_MAX; // for servo arm protection
        if (duty_curr < _DUTY_MIN)
            duty_curr = _DUTY_MIN;
        myservo.writeMicroseconds(duty_curr);
    }

    if (event_serial) {
        event_serial = false;

        if (0) { // use for debugging with serial plotter
            Serial.print("MIN:0,MAX:310");
            Serial.print(",TARGET:");       Serial.print(dist_target);
            Serial.print(",DIST:");         Serial.print(dist_ema);
            Serial.print(",ERR_curr:");     Serial.print(error_curr);
            Serial.print(",ERR_ave(10X):"); Serial.print(error_sum / (float) error_cnt * 10.0);
            Serial.println();
        }

        if (0) { // use for debugging with serial monitor
            Serial.print("ERR_curr:");    Serial.print(error_curr);
            Serial.print(",PTERM:");      Serial.print(pterm);
            Serial.print(",ITERM:");      Serial.print(iterm);
            Serial.print(",stable_cnt:"); Serial.print(stable_cnt);
            Serial.println();
        }

        if (0) { // use for debugging with serial monitor
            Serial.print("MIN:0,MAX:310");
            Serial.print(",TARGET:");   Serial.print(dist_target);
            Serial.print(",DIST:");     Serial.print(dist_ema);
            Serial.print(",pterm:");    Serial.print(pterm);
            Serial.print(",dterm:");    Serial.print(dterm);
            Serial.print(",iterm:");    Serial.print(iterm);
            Serial.print(",ERR_cnt:");  Serial.print(error_cnt);
            Serial.print(",ERR_sum:");  Serial.print(error_sum);
            Serial.print(",ERR_curr:"); Serial.print(abs(error_curr));
            Serial.print(",ERR_ave:");  Serial.print(error_sum / (float) error_cnt);
            Serial.println();
        }

        if (1) { // use for evaluation
            Serial.print("ERR_cnt:");   Serial.print(error_cnt);
            Serial.print(",ERR_curr:"); Serial.print(abs(error_curr));
            Serial.print(",ERR_sum:");  Serial.print(error_sum);
            Serial.print(",ERR_ave:");  Serial.print(error_sum / (float) error_cnt);
            Serial.println();
        }
    }
}

float volt_to_distance(int a_value)
{
    const float x = (float)a_value;
    //1381 + -8.91x + 0.0223x^2 + -2.44E-05x^3 + 9.42E-09x^4
    return (1381.0 - 8.91 * x + 0.0223 * pow(x, 2) - 2.44E-05 * pow(x, 3) + 9.42E-09 * pow(x, 4));
}

unsigned int ir_sensor_filtered(unsigned int n, float position)
{
    // Eliminate spiky noise of an IR distance sensor by repeating measurement and taking a middle value
    // n: number of measurement repetition
    // position: the percentile of the sample to be taken (0.0 <= position <= 1.0)

    // The output of Sharp infrared sensor includes lots of spiky noise.
    // To eliminate such a spike, ir_sensor_filtered() performs the following two steps:
    // Step 1. Repeat measurement n times and collect n * position smallest samples, where 0 <= postion <= 1.
    // Step 2. Return the largest sample among the collected ones.

    unsigned int *ir_val, tmp, ret_idx, ret_val;
    unsigned int start_time;

    ret_idx = (unsigned int)ceil(n * position);

    // Step 1. Repeat measurement n times and collect n * position smallest samples.
    // Note: simple implementation requires an array of n elements to store n samples.
    // Instead, we can save memory by allocating an array of (n * position + 1) elements.

    ir_val = (unsigned int *)malloc(sizeof(unsigned int) * (ret_idx + 2));
    ir_val[0] = analogRead(PIN_IR);

    for (int i = 1; i < n; i++) {
        int j;
        if (i < ret_idx + 1) {
            ir_val[i] = analogRead(PIN_IR);
            j = i - 1;
        } else {
            ir_val[ret_idx + 1] = analogRead(PIN_IR);
            j = ret_idx;
        }

        for (; j >= 0; j--) {
            if (ir_val[j] > ir_val[j + 1]) {
                tmp = ir_val[j];
                ir_val[j] = ir_val[j + 1];
                ir_val[j + 1] = tmp;
            }
        }
    }

    // Step 2. Return the largest sample among the collected ones.
    if (position > 0.0)
        ret_val = ir_val[ret_idx];
    else
        ret_val = ir_val[0];

    free(ir_val);

    return ret_val;
}
