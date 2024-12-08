#pragma once

#include "Arduino.h"
#include <list>
#include <vector>

class Sim;

class SimElement {
    public:
        virtual void update(Sim *sim, double &f_sum);
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
        Sim(double m, double x_min, double x_max, double v_min, double v_max, double a_min, double a_max) :
            _m(m), _x_min_tgt(x_min), _x_max_tgt(x_max), _v_min(v_min), _v_max(v_max), _a_min(a_min), _a_max(a_max) {}
        double get_x(void) {
            return _x;
        }
        double get_v(void) {
            return _v;
        }
        double get_a(void) {
            return _a;
        }
        double get_f_sum(void) {
            return _f_sum;
        }
        void set_m(double val) {
            _m = val;
        }
        void set_x_min(double val, bool immediate=false) {
            _x_min_tgt = val;
            if (immediate) {
                _x_min = val;
            }
        }
        void set_x_max(double val, bool immediate=false) {
            _x_max_tgt = val;
            if (immediate) {
                _x_max = val;
            }
        }
        void set_v_min(double val) {
            _v_min = val;
            _v = constrain(_v, _v_min, _v_max);
        }
        void set_v_max(double val) {
            _v_max = val;
            _v = constrain(_v, _v_min, _v_max);
        }
        void set_a_min(double val) {
            _a_min = val;
            _a = constrain(_a, _a_min, _a_max);
        }
        void set_a_max(double val) {
            _a_max = val;
            _a = constrain(_a, _a_min, _a_max);
        }
        void add_element(SimElement *element) {
            _elements.emplace_back(element);
        }
        void remove_element(SimElement *element) {
            _elements.remove(element);
        }
        void update(double &dt, double &f_in, bool final_f=false);

    private:
        std::list<SimElement*> _elements = {};
        double _m;
        double _x_min = 0.0;
        double _x_max = 0.0;
        double _x_min_tgt;
        double _x_max_tgt;
        double _v_min;
        double _v_max;
        double _a_min;
        double _a_max;
        double _x = 0.0;
        double _v = 0.0;
        double _a = 0.0;
        double _f_sum;
};

class CompoundElement : public SimElement {
    public:
        void update(Sim *sim, double &f_sum);
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
        Spring(double offset, double k) : _k(k), _offset(offset) {};
        void update(Sim *sim, double &f_sum);
        void set_k(double val) {
            _k = val;
        }
        void set_offset(double val) {
            _offset = val;
        }

    private:
        double _k;
        double _offset;
};

class Damper : public SimElement {
    public:
        Damper(double k) : _k_neg(k), _k_pos(k) {};
        Damper(double k_neg, double k_pos) : _k_neg(k_neg), _k_pos(k_pos) {};
        void update(Sim *sim, double &f_sum);
        void set_k_neg(double val) {
            _k_neg = val;
        }
        void set_k_pos(double val) {
            _k_pos = val;
        }
 
    private:
        double _k_neg;
        double _k_pos;
};

class Friction : public SimElement {
    public:
        Friction(double f) : _f(f) {};
        void update(Sim *sim, double &f_sum);
        void set_f(double val) {
            _f = val;
        }

    private:
        double _f;
};

class ConstForce : public SimElement {
    public:
        ConstForce(double f) : _f(f) {};
        void update(Sim *sim, double &f_sum);
        void set_f(double val) {
            _f = val;
        }

    private:
        double _f;
};

class ForceMap : public SimElement {
    public:
        ForceMap(std::vector<double> x_vect, std::vector<double> f_vect) : _x_vect(x_vect), _f_vect(f_vect) {};
        void update(Sim *sim, double &f_sum);
        void set_map(std::vector<double> x_vect, std::vector<double> f_vect) {
            _x_vect = x_vect;
            _f_vect = f_vect;
        }

    private:
        std::vector<double> _x_vect;
        std::vector<double> _f_vect;
        int last_idx = 0;
};

class DampingMap : public SimElement {
    public:
        DampingMap(std::vector<double> x_vect, std::vector<double> k_vect) : _x_vect(x_vect), _k_vect_pos(k_vect), _k_vect_neg(k_vect) {};
        DampingMap(std::vector<double> x_vect, std::vector<double> k_vect_neg, std::vector<double> k_vect_pos) : _x_vect(x_vect), _k_vect_neg(k_vect_neg), _k_vect_pos(k_vect_pos) {};
        void update(Sim *sim, double &f_sum);
        void set_map(std::vector<double> x_vect, std::vector<double> k_vect) {
            _x_vect = x_vect;
            _k_vect_neg = k_vect;
            _k_vect_pos = k_vect;
        }
        void set_map(std::vector<double> x_vect, std::vector<double> k_vect_neg, std::vector<double> k_vect_pos) {
            _x_vect = x_vect;
            _k_vect_neg = k_vect_neg;
            _k_vect_pos = k_vect_pos;
        }

    private:
        std::vector<double> _x_vect;
        std::vector<double> _k_vect_neg;
        std::vector<double> _k_vect_pos;
        int last_idx = 0;
};
