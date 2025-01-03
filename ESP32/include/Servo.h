#include <Arduino.h>

class Servo {
    public:
        enum class State {
            Disabled, Enabled, Homing
        };
        enum class HomingState {
            HomeUnknown, Pending, Homed, LockedIn, LockingError, LockingBlocked
        };
        virtual bool setup(uint32_t steps_per_mm, uint32_t mm_per_rev, bool autohome=true);
        virtual bool home(void);
        virtual bool enable(void);
        virtual bool disable(void);
        virtual bool move_to(float position, bool blocking = false);
        virtual void move_to_slow(float position);
        virtual float get_min_pos(void);
        virtual float get_max_pos(void);
        virtual void periodic_task_func(void);
        State get_state(void) {
            return _state;
        }
        HomingState get_homing_state(void) {
            return _homing_state;
        }
        void pause(void) {
            if (_locking_blocked) return;
            _locking_blocked = true;
            if (_homing_state == HomingState::LockedIn) {
                _homing_state = HomingState::LockingBlocked;
            }
        }
        void resume(void) {
            if (!_locking_blocked) return;
            _locking_blocked = false;
            if (_homing_state == HomingState::LockingBlocked) {
                _homing_state = HomingState::Homed;
            }
        }
    protected:
        float _curr_pos = 0.0;
        bool _curr_pos_valid = false;
        bool _locking_blocked = false;
        State _state = State::Disabled;
        HomingState _homing_state = HomingState::HomeUnknown;
};