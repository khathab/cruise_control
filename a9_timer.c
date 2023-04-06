#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
//code to control A9 timer

//info on A9 timer
    //counts down
    //
#define TIMER_BASE_ADDR   0xFFFEC600
#define TIMER_LOAD        (*((volatile uint32_t*)(TIMER_BASE_ADDR + 0x00)))
#define TIMER_COUNTER     (*((volatile uint32_t*)(TIMER_BASE_ADDR + 0x04)))
#define TIMER_CONTROL     (*((volatile uint32_t*)(TIMER_BASE_ADDR + 0x08)))
#define TIMER_INTERRUPT   (*((volatile uint32_t*)(TIMER_BASE_ADDR + 0x0C)))


/**
 * @brief starts the timer and counts down for a duration of time
 * 
 * @param duration_microseconds // duration of time timer counts down
 */
void timer_init(uint32_t duration_microseconds){
    uint32_t time_load_value = (duration_microseconds * 200) -1; 
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
    uint32_t interrupt_status = TIMER_INTERRUPT;
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
uint32_t timer_time_passed(uint32_t timeout_period){
    uint32_t current_tick_count = TIMER_COUNTER; //gets tick value in timer
    uint32_t elapsed_ticks = (timeout_period/200) - current_tick_count; //converts time in microseconds to ticks, calculates tick delta
    uint32_t elapsed_time_us = elapsed_ticks*200;
    return elapsed_time_us;
}