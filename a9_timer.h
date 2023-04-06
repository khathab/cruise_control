#pragma once
#include <stdint.h>
#

void timer_init(uint32_t duration_microseconds);
int timer_is_timed_out();
void timer_stop();
void timer_reset_timeout();
uint32_t timer_time_passed(uint32_t timeout_period);