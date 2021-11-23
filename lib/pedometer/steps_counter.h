#ifndef _steps_counter_H_
#define _steps_counter_H_

#include <stdint.h>
#include <stdio.h>
#include <I2Cdev.h>
#include <MPU6050.h>

// These values need to be tunned
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
        void deriative(float* accel_corr, float* accel_der);
        int find_peaks(float* accel_der);
        MPU6050 accelgyro;
    public:
        Pedometer(MPU6050 accelgyro);
        int count_steps();
};

#endif