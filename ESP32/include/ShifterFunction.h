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

    private:
        enum class AxisRole {
            Unknown,
            X,
            Y
        };

        struct ForceMap {
            float x_min = 0.0f;
            float y_min = 0.0f;
            float step = 1.0f;
            float max_force = 0.0f;
            uint16_t x_count = 0;
            uint16_t y_count = 0;
            std::vector<float> fx = {};
            std::vector<float> fy = {};
            bool is_valid(void) const {
                return x_count > 0 && y_count > 0 && fx.size() == fy.size();
            }
            float sample_fx(float x, float y) const;
            float sample_fy(float x, float y) const;
        };

        class ShifterMapForce : public SimElement {
            public:
                void configure(CommManager *comm_manager, const ForceMap *map, AxisRole role, AxisID axis_x, AxisID axis_y, bool invert_x,
                               bool invert_y, bool use_fixed_x, float fixed_x);
                void update(Sim *sim, float &f_sum) override;

            private:
                CommManager *_comm_manager = nullptr;
                const ForceMap *_map = nullptr;
                AxisRole _role = AxisRole::Unknown;
                AxisID _axis_id_x = AxisID_AXIS_UNDEFINED;
                AxisID _axis_id_y = AxisID_AXIS_UNDEFINED;
                bool _invert_x = false;
                bool _invert_y = false;
                bool _use_fixed_x = false;
                float _fixed_x = 0.0f;
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

        ForceMap _force_map = {};
        ShifterMapForce _map_force = {};
        Damper _damper = Damper(1.0f);
};
