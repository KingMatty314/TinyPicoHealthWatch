#ifndef _spO2_H_
#define _spO2_H_

#define PI 3.141592653589793238
#define SAMPLING_FREQ 50
#define SAMPLING_PER 0.02
#define SAMPLES 1500

class Oxygenation {
    private:
        void calculate_oxygenation_rate();
        void low_pass_filter();

    public:
        Oxygenation();
        void add_data(long ir, long red);
        bool is_buffer_full();
        int get_oxygenation();
};

#endif