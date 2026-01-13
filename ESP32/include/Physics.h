#pragma once

#include <list>
#include <vector>

#include "Arduino.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

namespace fastmath {
constexpr float PIO2 = 1.57079632679489661923f;      // pi/2
constexpr float INV_PIO2 = 0.63661977236758134308f;  // 2/pi

static inline int32_t fast_round_to_int(float x) {
    return (int32_t)(x + (x >= 0.0f ? 0.5f : -0.5f));
}

// Accurate when r ~ [-pi/4, pi/4]
static inline float sin_poly(float r) {
    // sin(r) ≈ r - r^3/6 + r^5/120 - r^7/5040
    float r2 = r * r;
    return r * (1.0f + r2 * (-0.1666666716f + r2 * (0.0083333477f + r2 * (-0.0001984090f))));
}

static inline float cos_poly(float r) {
    // cos(r) ≈ 1 - r^2/2 + r^4/24 - r^6/720
    float r2 = r * r;
    return 1.0f + r2 * (-0.5f + r2 * (0.0416666418f + r2 * (-0.0013888378f)));
}

static inline float fast_sinf(float x) {
    // Reduce x to r around nearest k*(pi/2)
    int32_t k = fast_round_to_int(x * INV_PIO2);
    float r = x - (float)k * PIO2;
    int32_t q = k & 3;

    float s = sin_poly(r);
    float c = cos_poly(r);

    // sin(x) by quadrant:
    // q=0: sin =  s
    // q=1: sin =  c
    // q=2: sin = -s
    // q=3: sin = -c
    if (q == 0) return s;
    if (q == 1) return c;
    if (q == 2) return -s;
    return -c;
}

}  // namespace fastmath

class Sim;

inline float normalize_value(float value, float min_val, float max_val) {
    float val_range = (max_val - min_val);
    if (abs(val_range) < 0.01) {
        return 0.0;  // avoid div-by-zero
    }
    if (value <= min_val) {
        return 0.0;
    }
    if (value >= max_val) {
        return 1.0;
    }

    return (value - min_val) / val_range;
}

class SimElement {
    public:
        virtual void update(Sim *sim, float &f_sum);
        void enable(void) {
            _enabled = true;
        }
        void disable(void) {
            _enabled = false;
        }

    protected:
        bool _enabled = true;
};

class Sim {
    public:
        Sim(float m, float x_min, float x_max)
            : _m(m), _x_min_tgt(x_min), _x_max_tgt(x_max) {
        }
        float get_x(void) {
            return _x;
        }
        float get_v(void) {
            return _v;
        }
        float get_a(void) {
            return _a;
        }
        float get_f_sum(void) {
            return _f_sum;
        }
        float get_m(void) const {
            return _m;
        }
        float get_dt_ms(void) const {
            return _dt_ms;
        }
#ifdef UNIT_TEST
        void set_state(float x, float x_prev) {
            _x = x;
            _x_prev = x_prev;
        }
        float get_x_prev(void) const {
            return _x_prev;
        }
        void set_dt_ms(float dt_ms) {
            _dt_ms = dt_ms;
        }
        float compute_force_sum(float f_in);
#endif
        void set_m(float val) {
            _m = val;
        }
        void set_x_min(float val, bool immediate = false) {
            _x_min_tgt = val;
            if (immediate) {
                _x_min = val;
            }
        }
        void set_x_max(float val, bool immediate = false) {
            _x_max_tgt = val;
            if (immediate) {
                _x_max = val;
            }
        }
        float get_x_min(void) {
            return _x_min_tgt;
        }
        float get_x_max(void) {
            return _x_max_tgt;
        }
        void add_element(SimElement *element) {
            _elements.emplace_back(element);
        }
        void remove_element(SimElement *element) {
            _elements.remove(element);
        }
        void update(float &dt, float &f_in, bool final_f = false);

    private:
        std::list<SimElement *> _elements = {};
        float _m;
        float _x_min = 0.0;
        float _x_max = 0.0;
        float _x_min_tgt;
        float _x_max_tgt;
        float _x = 0.0;
        float _x_prev = 0.0;
        float _v = 0.0;
        float _a = 0.0;
        float _f_sum;
        float _dt_ms = 0.0f;
};

class CompoundElement : public SimElement {
    public:
        void update(Sim *sim, float &f_sum);
        void add_element(SimElement *element) {
            _elements.emplace_back(element);
        }
        void remove_element(SimElement *element) {
            _elements.remove(element);
        }

    private:
        std::list<SimElement *> _elements = {};
};

class Spring : public SimElement {
    public:
        Spring(float offset, float k) : _k(k), _offset(offset) {};
        void update(Sim *sim, float &f_sum);
        void set_k(float val) {
            _k = val;
        }
        void set_offset(float val) {
            _offset = val;
        }

    private:
        float _k;
        float _offset;
};

class Damper : public SimElement {
    public:
        Damper(float k) : _k_neg(k), _k_pos(k) {};
        Damper(float k_neg, float k_pos) : _k_neg(k_neg), _k_pos(k_pos) {};
        void update(Sim *sim, float &f_sum);
        void set_k(float val) {
            _k_neg = val;
            _k_pos = val;
        }
        void set_k_neg(float val) {
            _k_neg = val;
        }
        void set_k_pos(float val) {
            _k_pos = val;
        }

    private:
        float _k_neg;
        float _k_pos;
};

class OscillationGuard : public SimElement {
    public:
        OscillationGuard(float k_max = 0.5f,
                         float min_amplitude = 0.2f,
                         float min_velocity = 0.5f,
                         uint32_t min_half_period_us = 5000,
                         uint32_t max_half_period_us = 250000,
                         uint32_t hold_time_us = 150000,
                         uint32_t ramp_time_us = 80000,
                         uint8_t required_hits = 2)
            : _k_max(k_max),
              _min_amplitude(min_amplitude),
              _min_velocity(min_velocity),
              _min_half_period_us(min_half_period_us),
              _max_half_period_us(max_half_period_us),
              _hold_time_us(hold_time_us),
              _ramp_time_us(ramp_time_us),
              _required_hits(required_hits) {}
        void update(Sim *sim, float &f_sum);
        void set_damping_gain(float k_max) {
            _k_max = max(k_max, 0.0f);
        }
        void set_detection(float min_amplitude, float min_velocity, uint32_t min_half_period_us, uint32_t max_half_period_us) {
            _min_amplitude = min_amplitude;
            _min_velocity = min_velocity;
            _min_half_period_us = min_half_period_us;
            _max_half_period_us = max_half_period_us;
        }
        void set_timing(uint32_t hold_time_us, uint32_t ramp_time_us) {
            _hold_time_us = hold_time_us;
            _ramp_time_us = ramp_time_us;
        }
        void set_required_hits(uint8_t required_hits) {
            _required_hits = required_hits < 1 ? 1 : required_hits;
        }

    private:
        float _k_max;
        float _min_amplitude;
        float _min_velocity;
        uint32_t _min_half_period_us;
        uint32_t _max_half_period_us;
        uint32_t _hold_time_us;
        uint32_t _ramp_time_us;
        uint8_t _required_hits;
        uint8_t _oscillation_hits = 0;
        int8_t _last_v_sign = 0;
        float _last_peak_x = 0.0f;
        float _damping_gain = 0.0f;
        uint32_t _last_flip_us = 0;
        uint32_t _last_oscillation_us = 0;
        uint32_t _last_update_us = 0;
};

class Buffet : public SimElement {
    public:
        Buffet(float amplitude = 0.0f) : _amplitude(max(amplitude, 0.0f)) {}
        void update(Sim *sim, float &f_sum);
        void set_amplitude(float amplitude) {
            _amplitude = max(amplitude, 0.0f);
        }

    private:
        float _amplitude = 0.0f;
        float _fast_state = 0.0f;
        float _slow_state = 0.0f;
        uint32_t _last_update_us = 0;
        uint32_t _rng_state = 0x6d2b79f5;
};

class Friction : public SimElement {
    public:
        Friction(float f) : _f(f) {};
        void update(Sim *sim, float &f_sum);
        void set_f(float val) {
            _f = val;
        }

    private:
        float _f;
};

class ConstForce : public SimElement {
    public:
        ConstForce(float f) : _f(f) {};
        void update(Sim *sim, float &f_sum);
        void set_f(float val) {
            _f = val;
        }

    private:
        float _f;
};

class ForceMap : public SimElement {
    public:
        ForceMap(std::vector<float> x_vect, std::vector<float> f_vect) : _x_vect(x_vect), _f_vect(f_vect) {};
        void update(Sim *sim, float &f_sum);
        void set_map(std::vector<float> x_vect, std::vector<float> f_vect) {
            _x_vect = x_vect;
            _f_vect = f_vect;
        }

    private:
        std::vector<float> _x_vect;
        std::vector<float> _f_vect;
        int _last_idx = 0;
};

class DampingMap : public SimElement {
    public:
        DampingMap(std::vector<float> x_vect, std::vector<float> k_vect) : _x_vect(x_vect), _k_vect_pos(k_vect), _k_vect_neg(k_vect) {};
        DampingMap(std::vector<float> x_vect, std::vector<float> k_vect_neg, std::vector<float> k_vect_pos)
            : _x_vect(x_vect), _k_vect_neg(k_vect_neg), _k_vect_pos(k_vect_pos) {};
        void update(Sim *sim, float &f_sum);
        void set_map(std::vector<float> x_vect, std::vector<float> k_vect) {
            _x_vect = x_vect;
            _k_vect_neg = k_vect;
            _k_vect_pos = k_vect;
        }
        void set_map(std::vector<float> x_vect, std::vector<float> k_vect_neg, std::vector<float> k_vect_pos) {
            _x_vect = x_vect;
            _k_vect_neg = k_vect_neg;
            _k_vect_pos = k_vect_pos;
        }

    private:
        std::vector<float> _x_vect;
        std::vector<float> _k_vect_neg;
        std::vector<float> _k_vect_pos;
        int _last_idx = 0;
};

class Cam : public SimElement {
    public:
        Cam(float f_max, float center, float half_width) : _f_max(f_max), _center(center), _half_width(half_width) {};
        void update(Sim *sim, float &f_sum);
        void set_f_max(float val) {
            _f_max = val;
        }
        void set_center(float val) {
            _center = val;
        }
        void set_half_width(float val) {
            _half_width = val;
        }

    private:
        float _f_max;
        float _center;
        float _half_width;
};
