#include <a9_timer.h>
#include <stdint.h>


#define JP1_BASE    0xFF200060
#define ULTRASONIC (*((volatile uint32_t*)(JP1_BASE + 0x00)))
#define ULTRASONIC_CONTROL (*((volatile uint32_t*)(JP1_BASE + 0x04)))
//ultrasonic sensor operation
    //set trig to output
    //set echo to input

    //for 10 seconds, send high signal on trigPin
    
    //read how long until echoPin receives signal

    //calculate distance

void ultrasonic_init(){
    ULTRASONIC_CONTROL |= (1<<0);
    ULTRASONIC_CONTROL |= (0<<1);
    
}


uint32_t calculate_distance(uint32_t time_microseconds){
    return time_microseconds * 0.034 /2;
}


void ultrasonic_set_high(){
    ULTRASONIC |=1;
}

void ultrasonic_set_low(){
    ULTRASONIC |= 0;
}

uint8_t ultrasonic_read(){
    return (ULTRASONIC & 0b10);
}

int main()
{
    //ultrasonic has two states
        //trig which sends a signal for 10us
        //echo which waits to receive signal

    uint8_t state = 0; //stores state of "state-machine"
    uint8_t timed_out = 1; //stores if timer has timed out
    uint32_t max_duration = 0xFFFFFFFF; // stores max timer duration
    uint32_t time_of_flight = 0; // stores time of flight for detected distance
    uint32_t distance = 0; //stores distance calculation


    if(state ==0){
    // sends trigger signal of ultrasonic
        if(timer_reset_timeout==1){
        //waits for timer to finish 10us count
            timer_reset_timeout();
            ultrasonic_set_low();
            state =1;
        }else{
        //sets counter to 10us count
            ultrasonic_set_high();
            timer_init(10);
        }
        
    }else if(state ==1){
    // waits and listens for echo
        if(ultrasonic_read == 1){
            time_of_flight = timer_time_passed(max_duration);
            distance = calculate_distance(time_of_flight);
        }

        if(timer_is_timed_out==1){
            //no ultrasonic reading, reset
            timed_out=1;
        }

        if(timed_out==1){
            //start timer count
            timer_init(max_duration);
            timed_out =0;
        }
        
    }

    return 0;
}
