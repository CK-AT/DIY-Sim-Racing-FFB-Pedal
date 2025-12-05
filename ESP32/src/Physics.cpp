#include <Physics.h>

void CompoundElement::update(Sim *sim, float &f_sum) {
    if (!_enabled) return;
    for (auto element : _elements) {
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
        f_sum = f_sum - _f;
    } else if (sim->get_v() < 0.0) {
        f_sum = f_sum + _f;
    } else {
        if (f_sum > _f) {
            f_sum = f_sum - _f;
        } else if (f_sum < -_f) {
            f_sum = f_sum + _f;
        } else {
            f_sum = 0.0;
        }
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
        _last_idx = 0;
    } else if (x >= _x_vect.back()) {
        f_sum -= _f_vect.back();
        _last_idx = _x_vect.size() - 1;
    } else {
        while ((_last_idx >= 0) && (_last_idx <= (_x_vect.size() - 1))) {
            if (x < _x_vect[_last_idx]) {
                _last_idx--;
            } else if (x > _x_vect[_last_idx + 1]) {
                _last_idx++;
            } else {
                float k = (_f_vect[_last_idx + 1] - _f_vect[_last_idx]) / (_x_vect[_last_idx + 1] - _x_vect[_last_idx]);
                float d = _f_vect[_last_idx] - (k * _x_vect[_last_idx]);
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
        _last_idx = 0;
    } else if (x >= _x_vect.back()) {
        f_sum -= (sim->get_v() * k_vect->back());
        _last_idx = _x_vect.size() - 1;
    } else {
        while ((_last_idx >= 0) && (_last_idx <= (_x_vect.size() - 1))) {
            if (x < _x_vect[_last_idx]) {
                _last_idx--;
            } else if (x > _x_vect[_last_idx + 1]) {
                _last_idx++;
            } else {
                float k = (k_vect->at(_last_idx + 1) - k_vect->at(_last_idx)) / (_x_vect[_last_idx + 1] - _x_vect[_last_idx]);
                float d = k_vect->at(_last_idx) - (k * _x_vect[_last_idx]);
                f_sum -= (sim->get_v() * ((k * x) + d));
                return;
            }
        }
    }
}

void Sim::update(float &dt, float &f_in, bool final_f) {
    float f_sum = f_in;
    _v = (_x - _x_prev) * 1000.0 / dt;

    if (!final_f) {
        for (auto element : _elements) {
            element->update(this, f_sum);
        }
    }

    _x_min += constrain(_x_min_tgt - _x_min, -20.0 * dt / 1000.0, 20.0 * dt / 1000.0);
    _x_max += constrain(_x_max_tgt - _x_max, -20.0 * dt / 1000.0, 20.0 * dt / 1000.0);

    _a = f_sum / _m * 1000.0;
    float x_raw = (2.0 * _x) - _x_prev + (((_a * dt * dt) / 1000.0) / 1000.0);
    _x_prev = _x;
    _x = constrain(x_raw, _x_min, _x_max);
    _f_sum = f_sum;
}
