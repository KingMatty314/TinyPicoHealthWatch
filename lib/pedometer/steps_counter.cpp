#include <steps_counter.h>
#include <math.h>

Pedometer::Pedometer(){
}

void Pedometer::low_pass_filter(float* accel_mag, float* accel_lpf){
    float omega = 2.0 * PI * 2;
    float Ts = SAMPLING_PER;
    float alpha = (2 - Ts*omega)/(2 + Ts*omega);
    float beta = Ts*omega/(2 + Ts*omega);
    for (int i = 0; i < SAMPLES; i++)
    {
        accel_lpf[i] = alpha*accel_mag[i-1] + beta*(accel_mag[i] - accel_mag[i-1]);
    }
}

void Pedometer::remove_mean_filter(float* accel_lpf, float* accel_mean){
    float mean = 0.0;
    for (int i = 0; i < SAMPLES; i++)
    {
        mean = mean + accel_lpf[i];
    }
    mean = mean / SAMPLES;
    for (int i = 0; i < SAMPLES; i++)
    {
        accel_mean[i] = accel_lpf[i] - mean;
    }
}

void Pedometer::autocorrelation(float* accel_mean, float* accel_corr){
    float temp_autocorr = 0.0;
    for (int lag = 0; lag < NUM_AUTOCORR_LAGS; lag++)
    {
        temp_autocorr = 0.0;
        for (int j = 0; j < NUM_AUTOCORR_LAGS - lag; j++)
        {
            temp_autocorr =  temp_autocorr + accel_mean[j] * accel_mean[j + lag];
        }
        accel_corr[lag] = temp_autocorr;
    }
}

void Pedometer::derivative(float* accel_corr, float* accel_der){
    for (int i = 0; i < NUM_AUTOCORR_LAGS - 1; i++)
    {
        accel_der[i] = (accel_corr[i+1] - accel_corr[i]) / SAMPLING_PER;
    }  
}

int Pedometer::find_peaks(float* accel_der){
    int peaks = 0;
    bool sign_positive = false;
    bool sign_negative = false;
    for (int i = 0; i < NUM_AUTOCORR_LAGS - 1; i++)
    {
        if (accel_der[i]>0 && sign_positive == false && sign_negative == true)
        {
            if (accel_der[i+1]> PEAK_QUALITY)
            {
                peaks = peaks + 1;
            }

            if (accel_der[i] < 0)
            {
                sign_negative = true;
                sign_positive = false;   
            }
            
            if (accel_der[i] > 0)
            {
                sign_positive = true;
                sign_negative = false;
            }
        }   
    }
    return peaks;
}

void Pedometer::add_data(float ax, float ay, float az){
    if (!is_buffer_full){
        accel_buffer[index] = sqrt(pow(ax, 2) + pow(ay, 2) + pow(az, 2));
        index += 1;
    }
}

bool Pedometer::is_buffer_full(){
    if (index >= SAMPLES)
        return true;
    else
        return false;
}


int Pedometer::get_count_steps(){
    // Check buffer size
    if(!is_buffer_full()){
        return 0;
    }

    // Apply low pass filter
    low_pass_filter(accel_buffer, accel_lpf);

    // Apply mean filter
    remove_mean_filter(accel_lpf, accel_mean);

    // Apply correlation alogrithm
    autocorrelation(accel_mean, accel_corr);
    
    // Apply derivative
    derivative(accel_corr, accel_der);

    // Reset buffer index
    index = 0;

    // Find peaks
    return find_peaks(accel_der);
}