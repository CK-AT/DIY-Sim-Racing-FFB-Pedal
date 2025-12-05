#pragma once
#include <Arduino.h>

#include <array>

typedef struct {
        uint32_t t;
        String* name_prefix;
        String* name;
        float value;
} RTDebugSample;

extern QueueHandle_t _queue_data;

template <int NVALS, int FLOAT_PRECISION = 6>
class RTDebugOutput {
    private:
        std::array<String, NVALS> _out_names;
        String _name_prefix;

    public:
        RTDebugOutput(std::array<String, NVALS> out_names = {}, String name_prefix = "") : _out_names(out_names), _name_prefix(name_prefix) {
        }

        void offer_data(std::array<float, NVALS> values) {
            if (!_queue_data) return;
            RTDebugSample sample;
            sample.t = millis();
            sample.name_prefix = &_name_prefix;
            for (int i = 0; i < NVALS; i++) {
                sample.name = &(_out_names[i]);
                sample.value = values[i];
                xQueueSend(_queue_data, &sample, /*xTicksToWait=*/0);
            }
        }
};

class RTDebugOutputService {
    public:
        RTDebugOutputService(bool own_task = false) {
            _queue_data = xQueueCreate(20, sizeof(RTDebugSample));
            if (own_task) {
                xTaskCreatePinnedToCore(this->debug_output_task, "debugOutputTask", 5000, this, 1, nullptr, 0);
            }
        }

        void pump(int max_samples, int timeout = 0) {
            RTDebugSample sample;
            while (max_samples && (pdTRUE == xQueueReceive(_queue_data, &sample, /*xTicksToWait=*/timeout))) {
                Serial.printf(">%s%s:%i:%.6f\n", *sample.name_prefix, *sample.name, sample.t, sample.value);
                max_samples--;
            }
        }

    private:
        static void debug_output_task(void* pv_parameters) {
            RTDebugOutputService* debug_output = (RTDebugOutputService*)pv_parameters;
            for (;;) {
                debug_output->pump(1000, 1000);
            }
        }
};
