#ifndef FASTNONACCELSTEPPER_H
#define FASTNONACCELSTEPPER_H

#include <Arduino.h>

/**
 * @class FastNonAccelStepper
 * @brief A class to control a stepper motor using ESP32's MCPWM and PCNT modules.
 */
class FastNonAccelStepper {
    public:
        /**
         * @brief Constructor to initialize the FastNonAccelStepper.
         */
        FastNonAccelStepper(uint8_t step_pin, uint8_t dir_pin, bool invert_motor_dir);

        /**
         * @brief Set the maximum speed of the stepper motor.
         * @param speed Maximum speed in Hz.
         */
        void set_max_speed(uint32_t speed);

        /**
         * @brief Update the target position for the stepper motor.
         * @param target_pos The desired target position in steps.
         * @param blocking Wait until the move has finished.
         */
        void move_to(long target_pos, bool blocking);

        /**
         * @brief Update the target position for the stepper motor.
         * @param target_pos The desired target position in steps.
         * @param blocking Wait until the move has finished.
         */
        void move_to_verbose(long target_pos, bool blocking);

        /**
         * @brief Read the current position of the stepper motor.
         * @return The current position in steps.
         */
        long get_current_position() const;

        /**
         * @brief Move the stepper motor by a specified number of steps.
         * @param steps_to_move The number of steps to move (positive for forward, negative for backward).
         * @param blocking Wait until the move has finished.
         */
        void move(long steps_to_move, bool blocking);

        /**
         * @brief Force the stepper motor to stop immediately.
         */
        void force_stop();

        /**
         * @brief Set the current position of the stepper motor.
         * @param new_position_i32 The new position in steps.
         */
        void set_current_position(int32_t new_position_i32);

        /**
         * @brief Force the stepper motor to stop and set a new current position.
         * @param new_position_i32 The new position in steps.
         */
        void force_stop_and_new_position(int32_t new_position_i32);

        /**
         * @brief Check if the stepper motor is currently running.
         * @return True if the motor is running, false otherwise.
         */
        bool is_running();

        /**
         * @brief Keep the stepper motor running in a specified direction.
         * @param forward_dir True to run forward, false to run backward.
         * @param speed The speed at which to run the motor in Hz.
         */
        void keep_running_in_dir(bool forward_dir, uint32_t speed);

        /**
         * @brief Keep the stepper motor running forward.
         * @param speed The speed at which to run the motor in Hz.
         */
        void keep_running_forward(uint32_t speed);

        /**
         * @brief Keep the stepper motor running backward.
         * @param speed The speed at which to run the motor in Hz.
         */
        void keep_running_backward(uint32_t speed);

        /**
         * @brief Get the motor's position after all commanded moves are completed.
         * @return The final position in steps.
         */
        int32_t get_position_after_commands_completed();

    private:
        uint8_t _step_pin;                    ///< Step pin number.
        uint8_t _dir_pin;                     ///< Direction pin number.
        long _target_position;                ///< Target position in steps.
        uint32_t _max_speed;                  ///< Maximum speed in Hz.
        volatile int _overflow_count;         ///< Overflow count for multiturn position tracking.
        volatile int _overflow_count_control;  ///< Overflow count for control position tracking.

        int32_t _zero_position_i32 = 0;
        bool _is_running = false;
        bool _invert_motor_direction = false;

        // Variable to store the DIR pin logic
        bool _dir_level_forward_b = true;
        bool _dir_level_backward_b = false;
        uint8_t _dir_pcnt_lctrl_mode_b = 0;
        uint8_t _dir_pcnt_hctrl_mode_b = 0;

        xQueueHandle _pcnt_queue;  ///< Queue to handle PCNT events.

        /**
         * @brief Initialize the MCPWM module for step signal generation.
         */
        void init_mcpwm();

        /**
         * @brief Initialize the PCNT module for multiturn position tracking.
         */
        void init_pcnt_multiturn();

        /**
         * @brief Initialize the PCNT module for position control.
         */
        void init_pcnt_control();

        /**
         * @brief Handle PCNT events for multiturn tracking.
         * @param arg ISR argument.
         */
        static void IRAM_ATTR multiturn_pcnt_isr(void* arg);

        /**
         * @brief Handle PCNT events for position control.
         * @param arg ISR argument.
         */
        static void IRAM_ATTR control_pcnt_isr(void* arg);

        /**
         * @brief Initialize the stepper motor with specified step and direction pins.
         * @param step_pin The GPIO pin connected to the step input of the stepper motor driver.
         * @param dir_pin The GPIO pin connected to the direction input of the stepper motor driver.
         * @param invert_motor_dir Invert the direction pin logic.
         */
        void begin(uint8_t step_pin, uint8_t dir_pin, bool invert_motor_dir);
};

#endif  // FASTNONACCELSTEPPER_H
