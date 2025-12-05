#pragma once

#include "RTDebugOutput.h"
#include "freertos/timers.h"

static const int MAX_CYCLES = 1000;

class CycleTimer {
    private:
        RTDebugOutput<2> _rt_output;
        int64_t _time_first;
        int64_t _delta_acc = 0;
        int64_t _time_start;
        unsigned int _cycle_count;

    public:
        CycleTimer(String timer_name) : _rt_output({".rt", ".per"}, timer_name) {
            reset_timer();
        }

        void reset_timer() {
            _time_first = esp_timer_get_time();
            _cycle_count = 0;
            _delta_acc = 0;
        }

        void bump_start() {
            _cycle_count++;
            _time_start = esp_timer_get_time();
        }

        void bump_end() {
            int64_t time_end = esp_timer_get_time();
            _delta_acc += time_end - _time_start;
            if (_cycle_count > MAX_CYCLES) {
                int64_t time_elapsed = time_end - _time_first;

                float average_runtime = float(_delta_acc) / MAX_CYCLES;
                float average_cycle_time = float(time_elapsed) / MAX_CYCLES;
                _rt_output.offer_data({average_runtime, average_cycle_time});

                reset_timer();
            }
        }
};
