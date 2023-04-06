#define TIMER_BASE_ADDR   0xFFFEC600
#define TIMER_LOAD        (*((volatile int*)(TIMER_BASE_ADDR + 0x00)))
#define TIMER_COUNTER     (*((volatile int*)(TIMER_BASE_ADDR + 0x04)))
#define TIMER_CONTROL     (*((volatile int*)(TIMER_BASE_ADDR + 0x08)))
#define TIMER_INTERRUPT   (*((volatile int*)(TIMER_BASE_ADDR + 0x0C)))

#define JP1_BASE    0xFF200060
#define ULTRASONIC (*((volatile int*)(JP1_BASE + 0x00)))
#define ULTRASONIC_CONTROL (*((volatile int*)(JP1_BASE + 0x04)))

#define SW_BASE 0xFF200040
#define BUTTON_BASE 0xFF200050
#define HEX0_HEX3_BASE 0xFF200020
#define HEX4_HEX5_BASE 0xFF200030

volatile unsigned int* switchPointer = (unsigned int *)SW_BASE;
volatile unsigned int* buttonPointer = (unsigned int *)BUTTON_BASE;

volatile unsigned int* hexPointer[2] = {HEX0_HEX3_BASE, HEX4_HEX5_BASE};

unsigned char hexCode[16] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7C, 0x7, 0x7F, 0x67, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71};

// setDistance, currentDistance, vehicleSpeed
unsigned char displayValues[3] = {0, 0, 0};



/**
 * @brief starts the timer and counts down for a duration of time
 * 
 * @param duration_microseconds // duration of time timer counts down
 */
void timer_init(int duration_microseconds){
    int time_load_value = (duration_microseconds * 200) -1; 
    // max time is 21.47 s
    TIMER_LOAD = time_load_value;
    TIMER_CONTROL |= (1<<0); //turn on timer
    TIMER_CONTROL |= (1<<2); //enable interrupts

} 

/**
 * @brief checks if timer has timed out
 * 
 * @return true 
 * @return false 
 */
int timer_is_timed_out(){
    int interrupt_status = TIMER_INTERRUPT;
    if (interrupt_status == 1){
        return 1;
    }else{
        return 0;
    }
}

/**
 * @brief stops timer
 * 
 */
void timer_stop(){
    TIMER_CONTROL &= ~(1 << 0); //sets first bit to zero, masks by 0x1110
}

/**
 * @brief resets interrupt and stops timer
 * 
 */
void timer_reset_timeout(){
    TIMER_INTERRUPT = 1; //resets interrupt bit (kinda confusing, but you set to 1 not zero to reset)
    timer_stop();
}

/**
 * @brief checks the time passed
 * 
 * @param timeout_period 
 * @return uint32_t 
 */
int timer_time_passed(int timeout_period){
    int current_tick_count = TIMER_COUNTER; //gets tick value in timer
    int elapsed_ticks = (timeout_period/200) - current_tick_count; //converts time in microseconds to ticks, calculates tick delta
    int elapsed_time_us = elapsed_ticks*200;
    return elapsed_time_us;
}

void ultrasonic_init(){
    ULTRASONIC_CONTROL |= (1<<0);
    ULTRASONIC_CONTROL |= (0<<1);
    
}


int calculate_distance(int time_microseconds){
    return time_microseconds * 0.034 /2;
}


void ultrasonic_set_high(){
    ULTRASONIC |=1;
}

void ultrasonic_set_low(){
    ULTRASONIC |= 0;
}

int ultrasonic_read(){
    return (ULTRASONIC & 0b10);
}

// Function to display needed information on the dash (7-seg displays)
void DisplayHex(){
	
	unsigned char digitDisplay[6] = {0, 0, 0, 0, 0, 0};
	
    unsigned char i, j;
	
	for (i = 0; i < sizeof(digitDisplay) / sizeof(digitDisplay[0]); i++){
		j = i / 2;
		digitDisplay[i] = (displayValues[j] % 10);
		displayValues[j] = displayValues[j] / 10;
	}

    *hexPointer[0] = (((hexCode[digitDisplay[3]] << 8 | hexCode[digitDisplay[2]]) << 8 | hexCode[digitDisplay[1]]) << 8 | hexCode[digitDisplay[0]]);
    *hexPointer[1] = (hexCode[digitDisplay[5]] << 8 | hexCode[digitDisplay[4]]);
}



int main()
{
    //ultrasonic has two states
        //trig which sends a signal for 10us
        //echo which waits to receive signal

    int state = 0; //stores state of "state-machine"
    int timed_out = 1; //stores if timer has timed out
    int max_duration = 0xFFFFFFFF; // stores max timer duration
    int time_of_flight = 0; // stores time of flight for detected distance
    int distance = 0; //stores distance calculation

    while (1){
        DisplayHex();
        if(state ==0){
    // sends trigger signal of ultrasonic
        if(timer_is_timed_out()==1){
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
        if(ultrasonic_read() == 1){
            time_of_flight = timer_time_passed(max_duration);
            distance = calculate_distance(time_of_flight);
            displayValues[0] = distance;
            
        }

        if(timer_is_timed_out()==1){
            //no ultrasonic reading, reset
            timed_out=1;
        }

        if(timed_out==1){
            //start timer count
            timer_init(max_duration);
            timed_out =0;
        }
        
    }

    }
    
    return 0;
}
