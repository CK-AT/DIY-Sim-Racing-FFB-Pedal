#pragma once
#include <array>
#include <Arduino.h>

#define MAX_LOG_LINE_LENGTH 150

typedef std::function<void(const char *buff)> LogSendFunc;

extern QueueHandle_t _log_queue_data;

class LogOutput {
  public:
    template <typename ...Params>
    static void printf(const char* fmt, Params&&... params) {
      if (!_log_queue_data) return;
      char buff[MAX_LOG_LINE_LENGTH] = {};
      snprintf(buff, sizeof(buff) - 1, fmt, std::forward<Params>(params)...);
      xQueueSend(_log_queue_data, buff, /*xTicksToWait=*/0);
    }
};

class LogOutputService {
  public:
    LogOutputService(LogSendFunc send_func, bool own_task=false)
    {
      _send_func = send_func;
      _log_queue_data = xQueueCreate(20, MAX_LOG_LINE_LENGTH);
      if (own_task) {
        xTaskCreatePinnedToCore(this->log_output_task, "LogOutputTask", 2000, this, 1, nullptr, 0);
      }
    }

    void pump(int max_samples, int timeout=0) {
      char buff[MAX_LOG_LINE_LENGTH];
      while (max_samples && (pdTRUE == xQueueReceive(_log_queue_data, buff, /*xTicksToWait=*/timeout))) {
          _send_func(buff);
          max_samples--;
      }
    }

  private:
    LogSendFunc _send_func;
    static void log_output_task(void* pv_parameters) {
      LogOutputService* log_output = (LogOutputService*) pv_parameters;
      for (;;) {
        log_output->pump(100, 5);
      }
    }
};
