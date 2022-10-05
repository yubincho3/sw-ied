/*
영문은 원본 파일의 기존 주석,
한글은 직접 추가한 주석
*/

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
                          // 0.325가 적절하다 생각하여 설정
#define _MEDIAN_FILTER_MAX_LEN 30  // 중앙값 필터 배열의 최대 길이

// global variables
unsigned long last_sampling_time;             // unit: msec

// EMA 필터링된 거리값이 담기는 변수
double dist_ema;

// 정렬된 배열을 합침
// Merge sort
void merge(double arr[], unsigned int start, unsigned int end) {
    double *temp = (double *)malloc(sizeof(double) * (end + 1));

    unsigned int mid = (start + end) / 2;
    unsigned int left = start;
    unsigned int right = mid + 1;
    unsigned int idx = start;

    while ((left <= mid) && (right <= end)) {
        if (arr[left] < arr[right])
            temp[idx++] = arr[left++];
        else
            temp[idx++] = arr[right++];
    }

    while (left <= mid)
        temp[idx++] = arr[left++];
    
    while (right <= end)
        temp[idx++] = arr[right++];
    
    for (unsigned int i = start; i <= end; i++)
        arr[i] = temp[i];
    
    free(temp);
}

// double 배열을 오름차순으로 정렬함.
// Merge sort
void sort_arr(double arr[], unsigned int start, unsigned int end) {
    if (start < end) {
        unsigned int mid = (start + end) / 2;

        sort_arr(arr, start, mid);
        sort_arr(arr, mid + 1, end);

        merge(arr, start, end);
    }
}

// 측정된 거리값을 EMA 필터링하여 반환한다.
double get_ema_filtered_dist(double dist, double prev_ema) {
    double ret = dist * _EMA_ALPHA + (1.0 - _EMA_ALPHA) * prev_ema;
    return ret;
}

// 배열을 정렬하는 방식의 중앙값 필터 클래스
// sorted는 정렬에 사용할 임시 배열
// arr은 값이 저장되는 배열
// idx는 다음번에 값이 저장될 배열의 인덱스
// now_len는 현재 배열의 길이
// full은 배열이 다 찼는지 표시하는 변수
class MedianFilter_Array {
    double sorted[_MEDIAN_FILTER_MAX_LEN];
    double arr[_MEDIAN_FILTER_MAX_LEN];
    unsigned int now_len;
    unsigned int idx;
    bool full;

public:
    MedianFilter_Array() : sorted{}, arr{}, idx(0), now_len(0), full(false) {}

    // 배열에 새 값을 추가함
    // 배열의 길이가 최대 길이에 도달한 경우에는 가장 오래전의 값부터 덮어씌움
    void add_new_value(double new_value) {
        if (now_len == _MEDIAN_FILTER_MAX_LEN)
            full = true;
        
        if (idx == _MEDIAN_FILTER_MAX_LEN)
            idx = 0;
        
        arr[idx++] = new_value;

        if (full == false)
            now_len++;
    }

    // 원본 배열을 복사하여 정렬한 뒤 중앙값을 구하여 반환함
    // 중앙값을 구하기 위해 배열을 정렬하지만 sorted 배열에 복사를 한 후에
    // sorted 배열을 정렬하는 것이라 원본인 arr 배열은 변하지 않음
    double get_median_value() {
        for (unsigned int i = 0; i < now_len; i++)
            sorted[i] = arr[i];

        sort_arr(sorted, 0, now_len - 1);

        double ret = sorted[now_len / 2];

        // 배열의 길이가 짝수라면 중앙의 두 값의 평균이 중앙값
        if (now_len % 2 == 0) {
            ret += sorted[now_len / 2 - 1];
            ret /= 2;
        }

        return ret;
    }
};

// 중앙값 필터 객체
MedianFilter_Array filter;

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
        digitalWrite(PIN_LED, HIGH);       // LED OFF
    } else if (dist_raw > _DIST_MAX) {
        digitalWrite(PIN_LED, HIGH);       // LED OFF
    } else {    // In desired Range
        digitalWrite(PIN_LED, LOW);        // LED ON
    }

    // ema 필터링
    dist_ema = get_ema_filtered_dist(dist_raw, dist_ema);

    // median 필터링
    filter.add_new_value(dist_raw);                 // 중앙값 필터에 현재 거리를 추가하고,
    double dist_median = filter.get_median_value(); // 중앙값 필터에서 중앙값을 가져온다.

    // output the distance to the serial port
    Serial.print("Min:");      Serial.print(_DIST_MIN);
    Serial.print(",raw:");     Serial.print(dist_raw);
    Serial.print(",ema:");     Serial.print(dist_ema);
    Serial.print(",median:");  Serial.print(dist_median);
    Serial.print(",Max:");     Serial.print(_DIST_MAX);
    Serial.println();
}

// get a distance reading from USS. return value is in millimeter.
double USS_measure(unsigned int TRIG, unsigned int ECHO)
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
