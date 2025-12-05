#pragma once

#include "ConfigManager.h"
#include "Physics.h"

static const long ABS_ACTIVE_TIME_PER_TRIGGER_MILLIS = 100;
static const long RPM_ACTIVE_TIME_PER_TRIGGER_MILLIS = 100;
static const long BP_ACTIVE_TIME_PER_TRIGGER_MILLIS = 100;
static const long WS_ACTIVE_TIME_PER_TRIGGER_MILLIS = 100;
static const long CV_ACTIVE_TIME_PER_TRIGGER_MILLIS = 100;
static int RPM_VALUE_LAST = 0;

class ABSOscillation : public SimElement {
    private:
        long _time_last_trigger_millis;
        long _abs_time_millis;
        long _last_call_time_millis = 0;
        const ABSEffectConfig* _config = nullptr;

    public:
        ABSOscillation() : _time_last_trigger_millis(0) {
        }

    public:
        void set_config(const ABSEffectConfig& config) {
            _config = &config;
        }
        void trigger() {
            _time_last_trigger_millis = millis();
        }

        void update(Sim* sim, float& f_sum) {
            if (!_enabled) return;
            if (!_config) return;
            long time_now_millis = millis();
            float time_since_trigger = (time_now_millis - _time_last_trigger_millis);
            float abs_force_offset_local = 0;

            if (time_since_trigger > ABS_ACTIVE_TIME_PER_TRIGGER_MILLIS) {
                _abs_time_millis = 0;
            } else {
                _abs_time_millis += time_now_millis - _last_call_time_millis;
                float abs_time_seconds = _abs_time_millis / 1000.0f;

                // abs amplitude
                float abs_amp = 0;
                switch (_config->mode) {
                    case ABSMode_ABS_MODE_FORCE:
                        abs_amp = _config->ampl;
                        break;
                    default:
                        abs_amp = 0.0f;
                        break;
                }

                switch (_config->pattern) {
                    case ABSPattern_ABS_PATTERN_SINE:
                        // sine wave pattern
                        abs_force_offset_local = abs_amp * sin(2 * PI * _config->freq * abs_time_seconds);
                        break;
                    case ABSPattern_ABS_PATTERN_SAWTOOTH:
                        // sawtooth pattern
                        if (_config->freq > 0) {
                            abs_force_offset_local = abs_amp * fmod(abs_time_seconds, 1.0 / (float)_config->freq) * (float)_config->freq;
                            abs_force_offset_local -= abs_amp * 0.5f;  // make it symmetrical around 0
                        }
                        break;
                    default:
                        break;
                }

                f_sum += abs_force_offset_local;
            }

            _last_call_time_millis = time_now_millis;
        }
};

class RPMOscillation : public SimElement {
    private:
        long _time_last_trigger_millis;
        long _rpm_time_millis;
        long _last_call_time_millis = 0;
        float f_curr = 0.0f;

    public:
        RPMOscillation() : _time_last_trigger_millis(0) {
        }
        float rpm_value = 0;
        int32_t rpm_position_offset = 0;

    public:
        void trigger() {
            _time_last_trigger_millis = millis();
        }

        void update(Sim* sim, float& f_sum) {
            if (!_enabled) return;
            f_sum += f_curr;
        }
        void force_offset(RPMEffectConfig& config) {
            long time_now_millis = millis();
            float time_since_trigger = (time_now_millis - _time_last_trigger_millis);
            float rpm_force_offset = 0;
            float rpm_max_freq = config.max_freq;
            float rpm_min_freq = config.min_freq;
            // float RPM_max =10;
            float rpm_amp_base = config.amp;
            float rpm_amp = 0;
            if (rpm_value == 0) {
                rpm_min_freq = 0;
            }
            rpm_amp = rpm_amp_base * (1 + 0.3 * rpm_value / 100);

            float rpm_freq = constrain(rpm_value * (rpm_max_freq - rpm_min_freq) / 100, rpm_min_freq, rpm_max_freq);

            if (time_since_trigger > RPM_ACTIVE_TIME_PER_TRIGGER_MILLIS) {
                _rpm_time_millis = 0;
                f_curr = RPM_VALUE_LAST;
            } else {
                _rpm_time_millis += time_now_millis - _last_call_time_millis;
                float rpm_time_seconds = _rpm_time_millis / 1000.0f;

                // rpm_force_offset = calcVars_st->absAmplitude * sin(calcVars_st->absFrequency * rpm_time_seconds);
                f_curr = rpm_amp * sin(2 * PI * rpm_freq * rpm_time_seconds);
            }

            _last_call_time_millis = time_now_millis;
            // return rpm_force_offset;
        }
};

class BitePointOscillation : public SimElement {
    private:
        long _time_last_trigger_millis;
        long _bite_time_millis;
        long _last_call_time_millis = 0;
        float f_curr = 0.0f;

    public:
        BitePointOscillation() : _time_last_trigger_millis(0) {
        }
        // float rpm_value =0;
        float bite_point_force_offset = 0;

    public:
        void trigger() {
            _time_last_trigger_millis = millis();
        }

        void update(Sim* sim, float& f_sum) {
            if (!_enabled) return;
            f_sum += f_curr;
        }
        void force_offset(BitePointEffectConfig& config) {
            long time_now_millis = millis();
            float time_since_trigger = (time_now_millis - _time_last_trigger_millis);
            float bp_freq = config.freq;
            // float bp_freq = 15;
            float bp_amp = config.amp;
            // float bp_amp = 2;

            if (time_since_trigger > BP_ACTIVE_TIME_PER_TRIGGER_MILLIS) {
                _bite_time_millis = 0;
                f_curr = 0;
            } else {
                _bite_time_millis += time_now_millis - _last_call_time_millis;
                float bp_time_seconds = _bite_time_millis / 1000.0f;

                // rpm_force_offset = calcVars_st->absAmplitude * sin(calcVars_st->absFrequency * rpm_time_seconds);
                f_curr = bp_amp * sin(2 * PI * bp_freq * bp_time_seconds);
            }
            _last_call_time_millis = time_now_millis;
            // RPM_VALUE_LAST=rpm_force_offset;

            // return rpm_force_offset;
        }
};

// moving average filter:https://github.com/sebnil/Moving-Avarage-Filter--Arduino-Library-/tree/master

#define MAX_DATA_POINTS 100
class MovingAverageFilter {
    public:
        // construct without coefs
        MovingAverageFilter(unsigned int new_data_points_count) {
            k = 0;  // initialize so that we start to write at index 0
            if (new_data_points_count < MAX_DATA_POINTS)
                data_points_count = new_data_points_count;
            else
                data_points_count = MAX_DATA_POINTS;

            for (i = 0; i < data_points_count; i++) {
                values[i] = 0;  // fill the array with 0's
            }
        }
        int data_points_count;
        float process(float in) {
            out = 0;

            values[k] = in;
            k = (k + 1) % data_points_count;

            for (i = 0; i < data_points_count; i++) {
                out += values[i];
            }

            float ret_value = 0;
            if (data_points_count > 0) {
                ret_value = out / data_points_count;
            }

            return ret_value;
        }

    private:
        float values[MAX_DATA_POINTS];
        int k;  // k stores the index of the current array read to create a circular memory through the array

        float out;
        int i;  // just a loop counter
};

// G force effect
class GForceEffect : public SimElement {
    public:
        float g_value = 0;
        float g_force_raw = 0;
        float g_force = 0;
        float f_curr = 0.0f;
        MovingAverageFilter moving_average_filter = MovingAverageFilter(100);

        void update(Sim* sim, float& f_sum) {
            if (!_enabled) return;
            f_sum += f_curr;
        }
        void force_offset(GForceEffectConfig& config) {
            uint32_t force_range;
            float g_multiplier = ((float)config.multi) / 100;
            if (g_value == -128) {
                g_force_raw = 0;

            } else {
                g_force_raw = 10 * (g_value)*g_multiplier / 9.8;
                // g_force_raw=constrain(g_force_raw,-1*force_range*0.25,force_range*0.25);
            }

            // apply filter
            g_force = moving_average_filter.process(g_force_raw);
            f_curr = g_force;
            // g_force=g_force_raw;
        }
};
// Wheel slip
class WheelSlipOscillation : public SimElement {
    private:
        long _time_last_trigger_millis;
        long _ws_time_millis;
        long _last_call_time_millis = 0;
        float f_curr = 0.0f;
        float _ws_force_offset = 0.0f;

    public:
        WheelSlipOscillation() : _time_last_trigger_millis(0) {
        }
        // float rpm_value =0;

    public:
        void trigger() {
            _time_last_trigger_millis = millis();
        }

        void update(Sim* sim, float& f_sum) {
            if (!_enabled) return;
            f_sum += f_curr;
        }
        void force_offset(WheelSlipEffectConfig& config) {
            long time_now_millis = millis();
            float time_since_trigger = (time_now_millis - _time_last_trigger_millis);
            float ws_force_offset_local = 0;
            float ws_freq = config.freq;
            // float bp_freq = 15;
            float ws_amp = config.amp;
            // float bp_amp = 2;

            if (time_since_trigger > WS_ACTIVE_TIME_PER_TRIGGER_MILLIS) {
                _ws_time_millis = 0;
                ws_force_offset_local = 0;
            } else {
                _ws_time_millis += time_now_millis - _last_call_time_millis;
                float ws_time_seconds = _ws_time_millis / 1000.0f;

                // rpm_force_offset = calcVars_st->absAmplitude * sin(calcVars_st->absFrequency * rpm_time_seconds);
                ws_force_offset_local = ws_amp * sin(2 * PI * ws_freq * ws_time_seconds);
                /*if (ws_freq > 0)
                {
                  //ws_force_offset_local = ws_amp * fmod(ws_time_seconds, 1.0 / (float)ws_freq) * ws_freq;
                  //ws_force_offset_local = ws_amp * (2*fmod(ws_time_seconds, 1.0 / (float)ws_freq) * ws_freq-1);
                }
                */
            }
            _ws_force_offset = ws_force_offset_local;
            f_curr = _ws_force_offset;
            _last_call_time_millis = time_now_millis;
            // RPM_VALUE_LAST=rpm_force_offset;

            // return rpm_force_offset;
        }
};
// Road impact
class RoadImpactEffect {
    public:
        float road_impact_force = 0;
        float road_impact_force_raw = 0;
        uint8_t road_impact_value = 0;
        float f_curr = 0.0f;
        MovingAverageFilter moving_average_filter = MovingAverageFilter(100);

        void force_offset(RoadImpactEffectConfig& config) {
            uint32_t force_range;
            float road_multiplier = ((float)config.multi) / 100;
            force_range = 10;  // TODO ????
            // road_multiplier=0.1;
            road_impact_force_raw = 0.3 * road_multiplier * ((float)force_range) * ((float)road_impact_value) / 100;

            // apply filter
            road_impact_force = moving_average_filter.process(road_impact_force_raw);
            f_curr = road_impact_force;
        }
};
// Wheel slip
class CustomVibration : public SimElement {
    private:
        long _time_last_trigger_millis;
        long _cv_time_millis;
        long _last_call_time_millis = 0;
        float f_curr = 0.0f;
        float _cv_force_offset = 0.0f;

    public:
        CustomVibration() : _time_last_trigger_millis(0) {
        }
        // float rpm_value =0;

    public:
        void trigger() {
            _time_last_trigger_millis = millis();
        }

        void update(Sim* sim, float& f_sum) {
            if (!_enabled) return;
            f_sum += f_curr;
        }
        void force_offset(CustomVibrationEffectConfig& config) {
            long time_now_millis = millis();
            float time_since_trigger = (time_now_millis - _time_last_trigger_millis);
            float cv_force_offset_local = 0;

            if (time_since_trigger > CV_ACTIVE_TIME_PER_TRIGGER_MILLIS) {
                _cv_time_millis = 0;
                cv_force_offset_local = 0;
            } else {
                _cv_time_millis += time_now_millis - _last_call_time_millis;
                float cv_time_seconds = _cv_time_millis / 1000.0f;

                cv_force_offset_local = config.amp / 20.0f * sin(2 * PI * config.freq * cv_time_seconds);
            }
            _cv_force_offset = cv_force_offset_local;
            f_curr = _cv_force_offset;
            _last_call_time_millis = time_now_millis;
        }
};
// MovingAverageFilter averagefilter_rudder(50);
// MovingAverageFilter averagefilter_rudder_force(50);
// class Rudder {
//     public:
//         int32_t Center_offset;
//         int32_t offset_raw;
//         int32_t offset_filter;
//         int32_t stepper_range;
//         int32_t dead_zone_upper;
//         int32_t dead_zone_lower;
//         int32_t dead_zone;
//         int16_t sync_pedal_position;
//         int16_t current_pedal_position;
//         float endpos_travel;
//         float force_range;
//         float force_offset_raw;
//         float force_offset_filter;
//         float force_center_offset;
//         float position_ratio_sync;
//         float position_ratio_current;
//         int debug_count = 0;

//         void offset_calculate(DAP_calculationVariables_st* calcVars_st) {
//             //    current_pedal_position=calcVars_st->current_pedal_position;
//             //    position_ratio_sync=calcVars_st->Sync_pedal_position_ratio;
//             endpos_travel = (float)calcVars_st->stepperPosRange;
//             position_ratio_current = ((float)(current_pedal_position - calcVars_st->stepperPosMin)) / endpos_travel;
//             dead_zone = 20;
//             Center_offset = calcVars_st->stepperPosMin + calcVars_st->stepperPosRange / 2;
//             float center_deadzone = 0.51;
//             if (calcVars_st->Rudder_status) {
//                 if (position_ratio_sync > center_deadzone) {
//                     offset_raw = (int32_t)(-1 * (position_ratio_sync - 0.50) * endpos_travel);

//                 } else {
//                     offset_raw = 0;
//                 }
//                 if (calcVars_st->rudder_brake_status) {
//                     offset_raw = 0;
//                 }
//                 offset_filter = averagefilter_rudder.process(offset_raw + Center_offset);
//             } else {
//                 offset_filter = calcVars_st->stepperPosMin;
//             }
//         }
//         void force_offset_calculate(DAP_calculationVariables_st* calcVars_st) {
//             dead_zone = 20;
//             Center_offset = calcVars_st->stepperPosRange / 2;
//             dead_zone_upper = Center_offset + dead_zone / 2;
//             dead_zone_lower = Center_offset - dead_zone / 2;
//             //    sync_pedal_position=calcVars_st->sync_pedal_position;
//             //    current_pedal_position=calcVars_st->current_pedal_position;
//             stepper_range = calcVars_st->stepperPosRange;
//             force_range = calcVars_st->force_range;
//             force_center_offset = force_range / 2 + calcVars_st->Force_Min;
//             endpos_travel = (float)calcVars_st->stepperPosRange;
//             // endpos_travel=((float)(calcVars_st->current_pedal_position-calcVars_st->stepperPosMin))/((float)calcVars_st->stepperPosRange);
//             //    position_ratio_sync=calcVars_st->Sync_pedal_position_ratio;
//             position_ratio_current = ((float)(current_pedal_position - calcVars_st->stepperPosMin)) / endpos_travel;

//             float center_deadzone = 0.51;
//             if (calcVars_st->Rudder_status) {
//                 if (position_ratio_sync > center_deadzone) {
//                     force_offset_raw = (float)(-1 * (position_ratio_sync - 0.50) * force_range);

//                 } else {
//                     force_offset_raw = 0;
//                 }
//                 if (calcVars_st->rudder_brake_status) {
//                     force_offset_raw = 0;
//                 }

//                 force_offset_filter = averagefilter_rudder_force.process(force_offset_raw + force_center_offset);
//             } else {
//                 force_offset_filter = 0;
//             }
//         }
// };
// // Rudder impact
// MovingAverageFilter Averagefilter_Rudder_G_Offset(50);
// class Rudder_G_Force {
//     public:
//         int32_t offset_raw;
//         int32_t offset_filter;
//         int32_t stepper_range;
//         uint8_t g_value;
//         long stepperPosMax;
//         void offset_calculate(DAP_calculationVariables_st* calcVars_st) {
//             stepperPosMax = (float)calcVars_st->stepperPosMax;
//             stepper_range = (float)calcVars_st->stepperPosRange;
//             float Amp_max = 0.3 * stepper_range;
//             if (calcVars_st->Rudder_status) {
//                 float offset = Amp_max * ((float)g_value) / 100.0f;
//                 // offset=constrain(offset,0,Amp_max);
//                 offset_filter = Averagefilter_Rudder_G_Offset.process((stepperPosMax - offset));
//             } else {
//                 offset_filter = calcVars_st->stepperPosMax;
//             }
//         }
// };
