#include <heart_rate.h>
#include <math.h>

HeartRate::HeartRate(){
}

void HeartRate::calculate_heart_rate(){

}

void HeartRate::low_pass_filter(){

}

void HeartRate::derivative(){

}

void HeartRate::find_peaks(){

}

void HeartRate::add_data(long irValue){
    if (!is_buffer_full()){
        ir_buffer[index] = irValue;
        index += 1;
    }
}

void HeartRate::clear_data(){
    index = 0;
}

bool HeartRate::is_buffer_full(){
    if (index >= SAMPLES_HEART)
        return true;
    else
        return false;
}

int HeartRate::get_heart_rate(){
    // Check buffer size
    
    // Do heartbeat stuff here!

    // Reset buffer index
    index = 0;

    // return heart beat
    return 82;
}

int HeartRate::get_buffer_index(){
    return index;
}
