#ifndef _steps_counter_H_
#define _steps_counter_H_

#include <stdint.h>
#include <stdio.h>
#include <math.h>

// These values need to be tunned
#define PI 3.141592653589793238
#define SAMPLING_FREQ 20
#define SAMPLING_PER 0.05
#define SAMPLES 600
#define NUM_AUTOCORR_LAGS 340
#define PEAK_QUALITY 0.5
#define LOW_PASS_CUTTOFF 2

class Pedometer{ 
    private:
        void low_pass_filter(float* accel_mag, float* accel_lpf);
        void remove_mean_filter(float* accel_lpf, float* accel_mean);
        void autocorrelation(float* accel_mean, float* accel_corr);
        void derivative(float* accel_corr, float* accel_der);
        int find_peaks(float* accel_der);

        // Arrays
        int16_t index = 0;
        float accel_buffer[SAMPLES] = {};
        float accel_lpf[SAMPLES] = {};
        float accel_mean[SAMPLES] = {};
        float accel_corr[NUM_AUTOCORR_LAGS] = {};
        float accel_der[NUM_AUTOCORR_LAGS] = {};

    public:
        Pedometer();
        void add_data(float ax, float ay, float az);
        bool is_buffer_full();
        int get_count_steps();
};

#endif