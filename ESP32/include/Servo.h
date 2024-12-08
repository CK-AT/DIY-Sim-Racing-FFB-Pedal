#include <Arduino.h>

class Servo {
    public:
        enum class State {
            Disabled, Enabled, Homing
        };
        enum class HomingState {
            HomeUnknown, Homed
        };
        virtual void setup(uint32_t steps_per_mm, uint32_t mm_per_rev);
        virtual uint8_t home(void);
        virtual void enable(void);
        virtual void disable(void);
        virtual int8_t move_to(double position, bool blocking = false);
        virtual void move_to_slow(double position);
        virtual double get_min_pos(void);
        virtual double get_max_pos(void);
        State get_state(void) {
            return _state;
        }
        HomingState get_homing_state(void) {
            return _homing_state;
        }
    protected:
        double _curr_pos = 0.0;
        State _state = State::Disabled;
        HomingState _homing_state = HomingState::HomeUnknown;
};