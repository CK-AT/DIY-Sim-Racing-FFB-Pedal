#pragma once

#include "freertos/timers.h"
#include "RTDebugOutput.h"

static const int MAX_CYCLES = 1000;

class CycleTimer {
private:
  RTDebugOutput<2> _rtOutput;
  int64_t _timeFirst;
  int64_t _delta_acc = 0;
  int64_t _timeStart;
  unsigned int _cycleCount;

public:
  CycleTimer(String timerName)
    : _rtOutput({ ".rt", ".per" }, timerName)
  {
    ResetTimer();
  }

  void ResetTimer() {
    _timeFirst = esp_timer_get_time();
    _cycleCount = 0;
    _delta_acc = 0;
  }

  void BumpStart() {
    _cycleCount++;
    _timeStart = esp_timer_get_time();
  }

  void BumpEnd() {
    int64_t timeEnd = esp_timer_get_time();
    _delta_acc += timeEnd - _timeStart;
    if (_cycleCount > MAX_CYCLES) {
      int64_t timeElapsed = timeEnd - _timeFirst;
              
      float averageRuntime = float(_delta_acc) / MAX_CYCLES;
      float averageCycleTime = float(timeElapsed) / MAX_CYCLES;
      _rtOutput.offerData({ averageRuntime, averageCycleTime });

      ResetTimer();
    }
  }
};
