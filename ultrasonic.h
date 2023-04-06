#pragma once
#include <stdint.h>
void ultrasonic_init();

uint32_t calculate_distance(uint32_t time_microseconds);
void ultrasonic_set_high();
void ultrasonic_set_low();
uint8_t ultrasonic_read();