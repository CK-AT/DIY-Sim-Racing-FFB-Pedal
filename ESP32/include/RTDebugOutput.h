#pragma once
#include <array>
#include <Arduino.h>

typedef struct {
  uint32_t t;
  String *name_prefix;
  String *name;
  float value;
} RTDebugSample;

extern QueueHandle_t _queue_data;

template <int NVALS, int FLOAT_PRECISION=6>
class RTDebugOutput {
  private:
    std::array<String,NVALS> _outNames;
    String _namePrefix;
    
  public:
    RTDebugOutput(std::array<String,NVALS> outNames = {}, String namePrefix = "")
      : _outNames(outNames), _namePrefix(namePrefix)
    { }

    void offerData(std::array<float,NVALS> values) {
        if (!_queue_data) return;
        RTDebugSample sample;
        sample.t = millis();
        sample.name_prefix = &_namePrefix;
        for (int i=0; i<NVALS; i++) {
          sample.name = &(_outNames[i]);
          sample.value = values[i];
          xQueueSend(_queue_data, &sample, /*xTicksToWait=*/0);
        }
    }
};

class RTDebugOutputService {
  public:
    RTDebugOutputService(bool own_task=false)
    {
      _queue_data = xQueueCreate(20, sizeof(RTDebugSample));
      if (own_task) {
        xTaskCreatePinnedToCore(this->debugOutputTask, "debugOutputTask", 5000, this, 1, NULL, 0);
      }
    }

    void pump(int max_samples, int timeout=0) {
      RTDebugSample sample;
      while (max_samples && (pdTRUE == xQueueReceive(_queue_data, &sample, /*xTicksToWait=*/timeout))) {
          Serial.printf(">%s%s:%i:%.6f\n", *sample.name_prefix, *sample.name, sample.t, sample.value);
          max_samples--;
      }
    }

  private:
    static void debugOutputTask(void* pvParameters) {
      RTDebugOutputService* debugOutput = (RTDebugOutputService*) pvParameters;
      for (;;) {
        debugOutput->pump(1000, 1000);
      }
    }
};
