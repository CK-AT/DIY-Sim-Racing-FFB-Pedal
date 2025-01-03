#pragma once

#include "Arduino.h"
#include <list>
#include <vector>

class Sim;

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
        Sim(float m, float x_min, float x_max, float v_min, float v_max, float a_min, float a_max) :
            _m(m), _x_min_tgt(x_min), _x_max_tgt(x_max), _v_min(v_min), _v_max(v_max), _a_min(a_min), _a_max(a_max) {}
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
        void set_m(float val) {
            _m = val;
        }
        void set_x_min(float val, bool immediate=false) {
            _x_min_tgt = val;
            if (immediate) {
                _x_min = val;
            }
        }
        void set_x_max(float val, bool immediate=false) {
            _x_max_tgt = val;
            if (immediate) {
                _x_max = val;
            }
        }
        void set_v_min(float val) {
            _v_min = val;
            _v = constrain(_v, _v_min, _v_max);
        }
        void set_v_max(float val) {
            _v_max = val;
            _v = constrain(_v, _v_min, _v_max);
        }
        void set_a_min(float val) {
            _a_min = val;
            _a = constrain(_a, _a_min, _a_max);
        }
        void set_a_max(float val) {
            _a_max = val;
            _a = constrain(_a, _a_min, _a_max);
        }
        void add_element(SimElement *element) {
            _elements.emplace_back(element);
        }
        void remove_element(SimElement *element) {
            _elements.remove(element);
        }
        void update(float &dt, float &f_in, bool final_f=false);

    private:
        std::list<SimElement*> _elements = {};
        float _m;
        float _x_min = 0.0;
        float _x_max = 0.0;
        float _x_min_tgt;
        float _x_max_tgt;
        float _v_min;
        float _v_max;
        float _a_min;
        float _a_max;
        float _x = 0.0;
        float _v = 0.0;
        float _a = 0.0;
        float _f_sum;
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
        std::list<SimElement*> _elements = {};
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
        int last_idx = 0;
};

class DampingMap : public SimElement {
    public:
        DampingMap(std::vector<float> x_vect, std::vector<float> k_vect) : _x_vect(x_vect), _k_vect_pos(k_vect), _k_vect_neg(k_vect) {};
        DampingMap(std::vector<float> x_vect, std::vector<float> k_vect_neg, std::vector<float> k_vect_pos) : _x_vect(x_vect), _k_vect_neg(k_vect_neg), _k_vect_pos(k_vect_pos) {};
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
        int last_idx = 0;
};
