#pragma once
#include <vector>

#include "Arduino.h"
#include "CommManager.fwd.h"
#include "ConfigManager.h"
#include "IFunction.h"
#include "Physics.h"

class ShifterFunction : public IFunction {
    public:
        ShifterFunction(void);
        void update_config(const ShifterConfig &config, CommManager &comm_manager, const AxisID *linked_axes);
        float get_x_contact_point_min(void) override;
        float get_x_contact_point_max(void) override;
        void on_ffb_action(const FFBAction &ffb_action) override;
        void update(Sim *sim, float &f_sum) override;

    private:
        enum class AxisRole {
            Unknown,
            X,
            Y
        };

        void rebuild_map(void);
        AxisRole resolve_axis_role(const AxisID *linked_axes);

        ShifterConfig _config = ShifterConfig_init_default;
        CommManager *_comm_manager = nullptr;
        AxisRole _axis_role = AxisRole::Unknown;
        AxisID _axis_id_x = AxisID_AXIS_UNDEFINED;
        AxisID _axis_id_y = AxisID_AXIS_UNDEFINED;
        bool _invert_x = false;
        bool _invert_y = false;

        Damper _damper = Damper(1.0f);
        Spring _centering_spring = Spring(0.0f, 5.0f);
        ForceMap _detents = ForceMap({-20.0f, -18.0, -16.0f, 16.0f, 18.0f, 20.0f}, {0.0f, 50.0f, 0.0f, 0.0f, -50.0f, 0.0f});
};
