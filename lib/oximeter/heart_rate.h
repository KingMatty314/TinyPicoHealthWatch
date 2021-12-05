#ifndef _heart_rate_H_
#define _heart_rate_H_

#define PI 3.141592653589793238
#define SAMPLING_FREQ 50
#define SAMPLING_PER 0.02
#define SAMPLES 1500

class HeartRate {
    private:
        void calculate_heart_rate();
        void low_pass_filter();
        void derivative();
        void find_peaks();

    public:
        HeartRate();
        void add_data(long ir);
        bool is_buffer_full();
        int get_heart_rate();
};

#endif