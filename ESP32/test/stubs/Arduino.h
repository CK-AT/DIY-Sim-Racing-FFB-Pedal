#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <type_traits>

using std::abs;

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

template <typename T, typename U>
constexpr auto min(T a, U b) -> std::common_type_t<T, U> {
    using R = std::common_type_t<T, U>;
    return (static_cast<R>(a) < static_cast<R>(b)) ? static_cast<R>(a) : static_cast<R>(b);
}

template <typename T, typename U>
constexpr auto max(T a, U b) -> std::common_type_t<T, U> {
    using R = std::common_type_t<T, U>;
    return (static_cast<R>(a) > static_cast<R>(b)) ? static_cast<R>(a) : static_cast<R>(b);
}

template <typename T, typename U, typename V>
inline T constrain(T value, U min_value, V max_value) {
    const T min_t = static_cast<T>(min_value);
    const T max_t = static_cast<T>(max_value);
    return (value < min_t) ? min_t : (value > max_t ? max_t : value);
}

typedef void *SemaphoreHandle_t;
typedef void *QueueHandle_t;
typedef uint32_t TickType_t;
typedef int BaseType_t;

constexpr int pdTRUE = 1;
constexpr int pdFALSE = 0;

inline SemaphoreHandle_t xSemaphoreCreateMutex(void) {
    return nullptr;
}

inline BaseType_t xSemaphoreTake(SemaphoreHandle_t, TickType_t) {
    return pdFALSE;
}

inline BaseType_t xSemaphoreGive(SemaphoreHandle_t) {
    return pdFALSE;
}

inline QueueHandle_t xQueueCreate(unsigned, unsigned) {
    return nullptr;
}

inline BaseType_t xQueueSend(QueueHandle_t, const void *, TickType_t) {
    return pdFALSE;
}

inline BaseType_t xQueueReceive(QueueHandle_t, void *, TickType_t) {
    return pdFALSE;
}

inline void xTaskCreatePinnedToCore(void (*)(void *), const char *, uint32_t, void *, uint32_t, void *, int) {
}

inline void delay(unsigned long) {
}

inline unsigned long micros(void) {
    return 0;
}
