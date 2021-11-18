#ifndef _steps_counter_H_
#define _steps_counter_H_

#include <stdint.h>
#include <stdio.h>
#include <I2Cdev.h>
#include <MPU6050.h>

// These values need to be tunned
#define SAMPLING_RATE 20
#define PEDOMETER_SAMPLES 80
#define NUM_AUTOCORR_LAGS 50
#define AUTOCORR_MIN_HALF_LEN 3
#define AUTOCORR_DELTA_AMPLITUDE_THRESH 5e8
#define WINDOW_LENGTH PEDOMETER_SAMPLES/SAMPLING_RATE

class Pedometer{
    private:
        float lowpassfilter(float* data);

        MPU6050 accelgyro;
    public:
        Pedometer(MPU6050 accelgyro);
        int count_steps();
};

#endif