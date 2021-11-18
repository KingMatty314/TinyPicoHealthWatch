#include <steps_counter.h>
#include <math.h>

Pedometer::Pedometer(MPU6050 mpu){
    accelgyro = mpu;
}


int Pedometer::count_steps(){
    float ax, ay, az;
    double mag[PEDOMETER_SAMPLES] = {};

    // these need to determined using test data
    int deriv_coeffs[5] = {-6, 31, 0, -31, 6};
    int lpf_coeffs[9] = {-5, 6, 34, 68, 84, 68, 34, 5, -5};

    // 1) Collect samples
    // the theroetical max sampling time of the MPU is 1kHz
    for (int i = 0; i<PEDOMETER_SAMPLES; i++){
        accelgyro.getFloatAcceleration(&ax, &ay, &az);
        mag[i] = sqrt(pow(ax, 2.0)+pow(ay,2.0)+ pow(az, 2.0));
        delay(50);
    }

    // 2) Apply FIR low pass filter
    int lpf[PEDOMETER_SAMPLES] = {};
    int temp_lpf = 0;
    for (int n = 0; n < PEDOMETER_SAMPLES; n++){
        temp_lpf = 0;
        for (int i = 0; i < 9; i++){
            if (n-i >= 0){
                temp_lpf += (int)lpf_coeffs[i]*mag[n-i];
            }
        }
        lpf[n]= temp_lpf;
    }

    // 3) Remove Mean
    int mean = 0;
    for (int i = 0; i < PEDOMETER_SAMPLES; i++){
        mean += lpf[i];
    }
    mean = mean/PEDOMETER_SAMPLES;
    for (int i = 0; i < PEDOMETER_SAMPLES; i++){
        lpf[i] = lpf[i] -  mean;
    }

    // 4) Autocorrection
    int autocorr_buff[NUM_AUTOCORR_LAGS] = {};
    for (int lag = 0; lag < NUM_AUTOCORR_LAGS; lag++){
        int temp_autocorr = 0;
        for (int i = 0; i < NUM_AUTOCORR_LAGS - lag; i++){
            temp_autocorr += lpf[i] * lpf[i+lag];
        }
        autocorr_buff[lag] = temp_autocorr;
    }

    // 5) Calculate FIR Deriative
    int deriv[NUM_AUTOCORR_LAGS] = {};
    for (int lag = 0; lag < NUM_AUTOCORR_LAGS; lag++){
        int temp_deriv = 0;
        for (int i = 0; i < PEDOMETER_SAMPLES; i++){
            if (lag-i >= 0){
                temp_deriv += deriv_coeffs[i] * autocorr_buff[lag-i];
            }
        }
        deriv[lag] = temp_deriv;
    }

    // 6) Determine autocorrelation peak index
    int peak_idx = 0;
    for (int lag = 0; lag < NUM_AUTOCORR_LAGS; lag++){
        if ((deriv[lag] > 0) && (deriv[lag-1] >0) && (deriv[lag-2] < 0) && (deriv[lag-3])){
            peak_idx = lag - 1;
            break;
        }
    }
    
    int loop_limit = 0;
    if ((autocorr_buff[peak_idx] > autocorr_buff[peak_idx-1]) && (autocorr_buff[peak_idx] > autocorr_buff[peak_idx+1])) {
        //peak_ind is perfectly set at the peak. nothing to do
    }
    else if ((autocorr_buff[peak_idx] > autocorr_buff[peak_idx+1]) && (autocorr_buff[peak_idx] < autocorr_buff[peak_idx-1])) {
        //peak is to the left. keep moving in that direction
        loop_limit = 0;
        while ((autocorr_buff[peak_idx] > autocorr_buff[peak_idx+1]) && (autocorr_buff[peak_idx] < autocorr_buff[peak_idx-1]) && (loop_limit < 10)) {
            peak_idx = peak_idx - 1;
            loop_limit++;
        }
    }
    else {
        //peak is to the right. keep moving in that direction
        loop_limit = 0;
        while ((autocorr_buff[peak_idx] > autocorr_buff[peak_idx-1]) && (autocorr_buff[peak_idx] < autocorr_buff[peak_idx+1]) && (loop_limit < 10)) {
            peak_idx = peak_idx + 1;
            loop_limit++;
        }
    }

    // 7) Calculate statistics on autocorrection peak to determine walking
    int neg_slope_count = 0;
    int delta_amplitude_right = 0;
    int pos_slope_count = 0;
    int delta_amplitude_left = 0;
    //first look to the right of the peak. walk forward until the slope begins decreasing
    int neg_slope_ind = peak_idx;
    int loop_limit_stats = NUM_AUTOCORR_LAGS-1;

    while ((autocorr_buff[neg_slope_ind+1] - autocorr_buff[neg_slope_ind] < 0) && (neg_slope_ind < loop_limit_stats)) {
        neg_slope_count = neg_slope_count + 1;
        neg_slope_ind = neg_slope_ind + 1;
    }
    
    //get the delta amplitude between peak and right trough
    delta_amplitude_right = autocorr_buff[peak_idx] - autocorr_buff[neg_slope_ind];
    
    //next look to the left of the peak. walk backward until the slope begins increasing
    int pos_slope_ind = peak_idx;
    loop_limit_stats = 0;
    while ((autocorr_buff[pos_slope_ind] - autocorr_buff[pos_slope_ind-1] > 0) && (pos_slope_ind > loop_limit_stats)) {
        pos_slope_count = pos_slope_count + 1;
        pos_slope_ind = pos_slope_ind - 1;
    }
    
    //get the delta amplitude between the peak and the left trough
    delta_amplitude_left = autocorr_buff[peak_idx] - autocorr_buff[pos_slope_ind];
    

    // 8) If  the autocorrelation peak is valid, calculate number of steps using the correlation frequency
    int num_steps = 0;
    if ((pos_slope_count > AUTOCORR_MIN_HALF_LEN) && (neg_slope_count > AUTOCORR_MIN_HALF_LEN) && (delta_amplitude_right > AUTOCORR_DELTA_AMPLITUDE_THRESH) && (delta_amplitude_left > AUTOCORR_DELTA_AMPLITUDE_THRESH)) {
        //the period is peak_ind/sampling_rate seconds. that corresponds to a frequency of 1/period
        //with the frequency known, and the number of seconds is 4 seconds, you can then find out the number of steps
        num_steps = (SAMPLING_RATE*WINDOW_LENGTH)/peak_idx;
    } else {
        //not a valid autocorrelation peak
        num_steps = 0;
    }

    return num_steps;
}