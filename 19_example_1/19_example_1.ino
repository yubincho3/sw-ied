// Arduino pin assignment
#define PIN_IR A0

// configurable parameters
#define LOOP_INTERVAL 20   // Loop Interval (unit: msec)
#define _EMA_ALPHA 0.7     // EMA weight of new sample (range: 0 to 1)
                           // Setting EMA to 1 effectively disables EMA filter.

// global variables
unsigned long last_sampling_time; // unit: msec
int raw, filtered, ema; // Voltage values from the IR sensor [0, 1023]

void setup()
{
    // initialize serial port
    Serial.begin(1000000);
    Serial.setTimeout(1);

    // initialize last sampling time
    last_sampling_time = 0;

    // while(1) {
    //     int incomingByte;
    //     while (Serial.available() == 0);
    //     incomingByte = Serial.read();
    //     filtered = ir_sensor_filtered(10000, 0.5, 2);
    // }
}

void loop()
{
    // wait until next sampling time.
    if (millis() < (last_sampling_time + LOOP_INTERVAL))
        return;
    last_sampling_time += LOOP_INTERVAL;

    // Take a single measurement
    raw = analogRead(PIN_IR);

    // Take a median value from multiple measurements
    filtered = ir_sensor_filtered(110, 0.5, 0);

    // Calculate EMA
    ema = (1.0 - _EMA_ALPHA) * filtered + _EMA_ALPHA * ema;

    // Oupput the raw, filtered, and EMA values for comparison purpose
    Serial.print("MIN:"); Serial.print(0);              Serial.print(",");
    Serial.print("RAW:"); Serial.print(raw);            Serial.print(",");
    Serial.print("FLT:"); Serial.print(filtered + 100); Serial.print(",");
    Serial.print("EMA:"); Serial.print(ema + 200);      Serial.print(",");
    Serial.print("MAX:"); Serial.println(600);
}

unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose)
{
    // Eliminate spiky noise of an IR distance sensor by repeating measurement and taking a middle value
    // n: number of measurement repetition
    // position: the percentile of the sample to be taken (0.0 <= position <= 1.0)
    // verbose: 0 for normal operation, 1 for observing the internal procedures, and 2 for calculating elapsed time.
    // Example 1: ir_sensor_filtered(n, 0.5, 0) => return the median value among the n measured samples.
    // Example 2: ir_sensor_filtered(n, 0.0, 0) => return the smallest value among the n measured samples.
    // Example 3: ir_sensor_filtered(n, 1.0, 0) => return the largest value among the n measured samples.

    // The output of Sharp infrared sensor includes lots of spiky noise.
    // To eliminate such a spike, ir_sensor_filtered() performs the following two steps:
    // Step 1. Repeat measurement n times and collect n * position smallest samples, where 0 <= postion <= 1.
    // Step 2. Return the largest sample among the collected ones.

    unsigned int *ir_val, tmp, ret_idx, ret_val;
    unsigned int start_time;

    ret_idx = (unsigned int)ceil(n * position);

    if (ret_idx > 0)
        ret_idx--;

    // Step 1. Repeat measurement n times and collect n * position smallest samples.
    // Note: simple implementation requires an array of n elements to store n samples.
    // Instead, we can save memory by allocating an array of (n * position + 1) elements.
    if (verbose == 2)
        start_time = millis(); 

    ir_val = (unsigned int*) malloc(sizeof(unsigned int) * (ret_idx + 2));
    ir_val[0] = analogRead(PIN_IR);
    if (verbose == 1) {
        Serial.print("ir_val[0]: "); Serial.print(ir_val[0]);
    }

    for (int i = 1; i < n; i++) {
        int j;
        if (i < ret_idx + 1) {
            ir_val[i] = analogRead(PIN_IR);
            if(verbose == 1) {
                Serial.print("\nir_val["); Serial.print(i); Serial.print("]: "); 
                Serial.println(ir_val[i]);
            }
            j = i - 1;
        }
        else {
            ir_val[ret_idx + 1] = analogRead(PIN_IR);
            if (verbose == 1) {
                Serial.print("\nir_val["); Serial.print(ret_idx + 1); Serial.print("]: ");
                Serial.println(ir_val[ret_idx + 1]);
            }
            j = ret_idx;
        }

        if (verbose == 1) {
            Serial.print("Before insertion: ");
            for (int k = 0; k < ret_idx + 2; k++) {
                Serial.print(ir_val[k]); Serial.print(" ");
                if (k >= i)
                    break;
            }
        }

        for (; j >= 0; j--) {
            if (ir_val[j] > ir_val[j+1]) {
                tmp = ir_val[j];
                ir_val[j] = ir_val[j + 1];
                ir_val[j + 1] = tmp;
            }
        }

        if (verbose == 1) {
            Serial.print("\nAfter insertion:  ");
            for (int k = 0; k < ret_idx + 2; k++) {
                Serial.print(ir_val[k]); Serial.print(" ");
                if (k >= i)
                    break;
            }
        }
    }

    // Step 2. Return the largest sample among the collected ones.
    if (position > 0.0) {
        ret_val = ir_val[ret_idx];
        if (verbose == 1) {
            Serial.print("\nReturn ir_val["); Serial.print(ret_idx); Serial.print("]: ");
            Serial.println(ret_val);
        }
    }
    else {
        ret_val = ir_val[0];
        if (verbose == 1) {
            Serial.print("\nReturn ir_val[0]: "); Serial.println(ret_val);
        }
    }

    if (verbose == 2) {
        Serial.print("Elapsed time:"); Serial.print(millis() - start_time); Serial.println("ms");
    }

    free(ir_val);

    return ret_val;
}
