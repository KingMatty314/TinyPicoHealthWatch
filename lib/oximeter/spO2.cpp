#include <spO2.h>
#include <math.h>

Oxygenation::Oxygenation(){
}

void Oxygenation::add_data(long ir, long red){
    if (!is_buffer_full()){
        ir_buffer[index] = ir;
        red_buffer[index] = red;
        index += 1;
    }
}

void Oxygenation::clear_data(){
    index = 0;
}

bool Oxygenation::is_buffer_full(){
    if (index >= SAMPLES_OXYG)
        return true;
    else
        return false;
}

int Oxygenation::get_oxygenation(){
    // Check buffer size

    // Do oygenation calculation stuff here!

    // Reset buffer index
    index = 0;

    // return oxygenation
    return 98;
}