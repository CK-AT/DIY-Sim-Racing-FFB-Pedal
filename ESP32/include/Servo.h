#include <Arduino.h>

class Servo {
    public:
        enum class State {
            Disabled, Enabled, Homing
        };
        enum class HomingState {
            HomeUnknown, Pending, Homed, LockedIn, LockingError
        };
        virtual bool setup(uint32_t steps_per_mm, uint32_t mm_per_rev);
        virtual bool home(void);
        virtual bool enable(void);
        virtual bool disable(void);
        virtual bool move_to(double position, bool blocking = false);
        virtual void move_to_slow(double position);
        virtual double get_min_pos(void);
        virtual double get_max_pos(void);
        virtual void periodic_task_func(void);
        virtual void lock_onto_curr_pos(void);
        State get_state(void) {
            return _state;
        }
        HomingState get_homing_state(void) {
            return _homing_state;
        }
    protected:
        double _curr_pos = 0.0;
        bool _curr_pos_valid = false;
        State _state = State::Disabled;
        HomingState _homing_state = HomingState::HomeUnknown;
};