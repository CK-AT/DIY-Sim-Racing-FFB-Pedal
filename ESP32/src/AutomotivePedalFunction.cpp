#include "AutomotivePedalFunction.h"

AutomotivePedalFunction::AutomotivePedalFunction(void) {
    disable();
    add_element(&force_curve);
    add_element(&abs_effect);
    add_element(&damper);
}

void AutomotivePedalFunction::update_config(const AutomotivePedalConfig &config) {
    _config = config;
    if (_config.has_force_curve_config) {
        force_curve.set_config(_config.force_curve_config);
        force_curve.enable();
    } else {
        force_curve.disable();
    }
    if (_config.has_abs_effect_config) {
        abs_effect.set_config(_config.abs_effect_config);
        abs_effect.enable();
    } else {
        abs_effect.disable();
    }
    if (_config.has_damper_config) {
        damper.set_k_pos(_config.damper_config.positive_factor);
        damper.set_k_neg(_config.damper_config.negative_factor);
    } else {
        /* fall back to a reasonable damper setting */
        damper.set_k(0.1f);
    }
}

void AutomotivePedalFunction::on_ffb_action(const FFBAction &ffb_action) {
    if (ffb_action.function.automotive_pedal.trigger_abs) trigger_abs();
}

