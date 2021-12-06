#ifndef _spO2_H_
#define _spO2_H_

#define PI 3.141592653589793238
#define SAMPLING_FREQ_OXYG 50
#define SAMPLING_PER_OXYG 0.02
#define SAMPLES_OXYG 1500

class Oxygenation {
    private:
        void calculate_oxygenation_rate();
        void low_pass_filter();

        // Buffer Index
        int index = 0;

        // Array
        float ir_buffer[SAMPLES_OXYG] = {};
        float red_buffer[SAMPLES_OXYG] = {};

    public:
        Oxygenation();
        void add_data(long ir, long red);
        void clear_data();
        bool is_buffer_full();
        int get_oxygenation();
};

#endif