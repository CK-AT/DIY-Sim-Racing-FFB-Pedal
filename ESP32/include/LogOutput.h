#pragma once
#include <array>
#include <Arduino.h>

extern QueueHandle_t _log_queue_data;

class LogOutput {
  public:
    template <typename ...Params>
    static void printf(const char* fmt, Params&&... params) {
      char buff[100] = {};
      snprintf(buff, 99, fmt, std::forward<Params>(params)...);
      xQueueSend(_log_queue_data, buff, /*xTicksToWait=*/0);
    }
};

class LogOutputService {
  public:
    LogOutputService(bool own_task=false)
    {
      _log_queue_data = xQueueCreate(20, 100);
      if (own_task) {
        xTaskCreatePinnedToCore(this->logOutputTask, "LogOutputTask", 2000, this, 1, NULL, 0);
      }
    }

    void pump(int max_samples, int timeout=0) {
      char buff[100];
      while (max_samples && (pdTRUE == xQueueReceive(_log_queue_data, buff, /*xTicksToWait=*/timeout))) {
          Serial.print(buff);
          max_samples--;
      }
    }

  private:
    static void logOutputTask(void* pvParameters) {
      LogOutputService* logOutput = (LogOutputService*) pvParameters;
      for (;;) {
        logOutput->pump(1000, 1000);
      }
    }
};
