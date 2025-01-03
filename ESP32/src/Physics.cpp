#include <Physics.h>

void CompoundElement::update(Sim *sim, float &f_sum) {
    if (!_enabled) return;
    for (auto element: _elements) {
        element->update(sim, f_sum);
    }
}

void Spring::update(Sim *sim, float &f_sum) {
    if (!_enabled) return;
    f_sum = f_sum - ((sim->get_x() - _offset) * _k);
}

void Damper::update(Sim *sim, float &f_sum) {
    if (!_enabled) return;
    float v = sim->get_v();
    if (v < 0.0) {
        f_sum = f_sum - (v * _k_neg);
    } else {
        f_sum = f_sum - (v * _k_pos);
    }
}

void Friction::update(Sim *sim, float &f_sum) {
    if (!_enabled) return;
    if (sim->get_v() > 0.0) {
        f_sum = f_sum - min(_f, abs(f_sum));
    } else {
        f_sum = f_sum + min(_f, abs(f_sum));
    }
}

void ConstForce::update(Sim *sim, float &f_sum) {
    if (!_enabled) return;
    f_sum += _f;
}

void ForceMap::update(Sim *sim, float &f_sum) {
    if (!_enabled) return;
    float x = sim->get_x();
    if (x <= _x_vect[0]) {
        f_sum -= _f_vect[0];
        last_idx = 0;
    } else if (x >= _x_vect.back()) {
        f_sum -= _f_vect.back();
        last_idx = _x_vect.size() - 1;
    } else {
        while ((last_idx >= 0) && (last_idx <= (_x_vect.size() - 1))) {
            if (x < _x_vect[last_idx]) {
                last_idx--;
            } else if (x > _x_vect[last_idx + 1]) {
                last_idx++;
            } else {
                float k = (_f_vect[last_idx+1]-_f_vect[last_idx]) / (_x_vect[last_idx+1]-_x_vect[last_idx]);
                float d = _f_vect[last_idx] - (k * _x_vect[last_idx]);
                f_sum -= ((k * x) + d);
                return;
            }
        }
    }
}

void DampingMap::update(Sim *sim, float &f_sum) {
    if (!_enabled) return;
    float x = sim->get_x();
    float v = sim->get_v();
    std::vector<float> *k_vect;
    if (v < 0.0) {
        k_vect = &_k_vect_neg;
    } else {
        k_vect = &_k_vect_pos;
    }
    if (x <= _x_vect[0]) {
        f_sum -= (sim->get_v() * k_vect->front());
        last_idx = 0;
    } else if (x >= _x_vect.back()) {
        f_sum -= (sim->get_v() * k_vect->back());
        last_idx = _x_vect.size() - 1;
    } else {
        while ((last_idx >= 0) && (last_idx <= (_x_vect.size() - 1))) {
            if (x < _x_vect[last_idx]) {
                last_idx--;
            } else if (x > _x_vect[last_idx + 1]) {
                last_idx++;
            } else {
                float k = (k_vect->at(last_idx+1)-k_vect->at(last_idx)) / (_x_vect[last_idx+1]-_x_vect[last_idx]);
                float d = k_vect->at(last_idx) - (k * _x_vect[last_idx]);
                f_sum -= (sim->get_v() * ((k * x) + d));
                return;
            }
        }
    }
}

void Sim::update(float &dt, float &f_in, bool final_f) {
    float f_sum = f_in;
    
    if (!final_f) {
        for (auto element: _elements) {
            element->update(this, f_sum);
        }
    }

    _x_min += constrain(_x_min_tgt - _x_min, -10.0 * dt / 1000.0, 10.0 * dt / 1000.0);
    _x_max += constrain(_x_max_tgt - _x_max, -10.0 * dt / 1000.0, 10.0 * dt / 1000.0);
    
    float a_raw = f_sum / _m * 1000.0;
    float a_lim = constrain(a_raw, _a_min, _a_max);
    if (abs(a_lim) < 0.001) {
        a_lim = 0.0;
    }
    float v_raw = _v + (((a_lim + _a) * dt) / 2000.0);
    float v_lim = constrain(v_raw, _v_min, _v_max);
    if (abs(v_lim) < 0.001) {
        v_lim = 0.0;
    }
    float x_raw = _x + (((v_lim + _v) * dt) / 2000.0);
    float x_lim = constrain(x_raw, _x_min, _x_max);
    if (abs(x_raw - x_lim) > 0.01) {
        v_lim = 0.0;
    }
    if (abs(v_raw - v_lim) > 0.01) {
        a_lim = 0.0;
    }
    _a = a_lim;
    _v = v_lim;
    _x = x_lim;
    _f_sum = f_sum;
}