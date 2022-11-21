// Arduino pin assignment
#define PIN_IR A0

// configurable parameters
#define LOOP_INTERVAL 20   // Loop Interval (unit: msec)
#define _EMA_ALPHA 0.7    // EMA weight of new sample (range: 0 to 1)
                          // Setting EMA to 1 effectively disables EMA filter.

// global variables
unsigned long last_sampling_time; // unit: msec
float dist_raw, dist_filtered, dist_ema; // Distance values (unit: mm)

void setup() {
    // initialize serial port
    Serial.begin(1000000);
    Serial.setTimeout(1);

    // initialize last sampling time
    last_sampling_time = 0;
}

void loop() {
    // wait until next sampling time.
    if (millis() < (last_sampling_time + LOOP_INTERVAL))
        return;

    last_sampling_time += LOOP_INTERVAL;

    // Distance measurement
    dist_raw = volt_to_distance(analogRead(PIN_IR));
    dist_filtered = volt_to_distance(ir_sensor_filtered(110, 0.5)); // Replace n with your desired value
    dist_ema = _EMA_ALPHA * dist_ema + (1.0 - _EMA_ALPHA) * dist_filtered;

    // Oupput the raw, filtered, and EMA values for comparison purpose
    Serial.print("MIN:"); Serial.print(0); Serial.print(",");
    //  Serial.print("RAW:"); Serial.print(dist_raw); Serial.print(",");
    //  Serial.print("FLT:"); Serial.print(dist_filtered); Serial.print(",");
    Serial.print("EMA:"); Serial.print(dist_ema); Serial.print(",");
    Serial.print("MAX:"); Serial.println(320);
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

    ret_idx = (unsigned int) ceil(n * position);

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
