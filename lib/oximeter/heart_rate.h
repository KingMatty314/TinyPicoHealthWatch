#ifndef _heart_rate_H_
#define _heart_rate_H_

#define PI 3.141592653589793238
#define SAMPLING_FREQ_HEART 50
#define SAMPLING_PER_HEART 0.02
#define SAMPLES_HEART 1500

class HeartRate {
    private:
        void calculate_heart_rate();
        void low_pass_filter();
        void derivative();
        void find_peaks();

        // Buffer Index
        int index = 0;

        // Array
        float ir_buffer[SAMPLES_HEART] = {};

    public:
        HeartRate();
        void add_data(long ir);
        void clear_data();
        bool is_buffer_full();
        int get_heart_rate();
        int get_buffer_index();
};

#endif