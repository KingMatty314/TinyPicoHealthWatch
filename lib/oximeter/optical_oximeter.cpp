#include <optical_oximeter.h>
#include <ff.h>

Oximeter::Oximeter(){

}


void Oximeter::getResults(){
    // 1) Set sample data N, with Sample Frequency fs

    // 2) Remove noise with low-pass filter

    // 3) Gaussian Filter to determine DC Signal

    // 4) Autocorrelation find peaks

    // 5) Calculate RMS values using peak
    // u_rms = (1/t * integate(u^2))^0.5

    // 6) Find Ratio of Illumination Values

    // 7) Look up table determine Sp02 Values

    // 8) Apply fast fourier transform

    // 9) Determine the frequency of AC signal

    // 10) Calculate heart rate
}