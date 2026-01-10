#include "LoadCell.h"

#include <ADS1256.h>
#include <SPI.h>
#include <math.h>

#include "LogOutput.h"
#include "Main.h"

namespace {
constexpr float k_adc_clock_mhz = 7.68f;     // crystal frequency used on ADS1256
constexpr float k_adc_vref = 2.5f;          // voltage reference
constexpr float k_default_variance = 0.2f * 0.2f;
constexpr float k_variance_min = 0.0001f;
constexpr uint32_t k_adc_drdy_timeout_us = 500000;
constexpr uint32_t k_adc_reset_delay_ms = 5;

float calc_conversion_factor(const LoadCellConfig &cfg) {
    if ((cfg.excitation_v <= 0.0f) || (cfg.sensitivity_mV_V <= 0.0f)) return 1.0f;
    return cfg.loadcell_rating_kg / (cfg.excitation_v * (cfg.sensitivity_mV_V / 1000.0f));
}

bool wait_drdy_timeout(uint32_t timeout_us) {
    uint32_t start = micros();
    while (digitalRead(PIN_DRDY)) {
        if (timeout_us && (micros() - start) > timeout_us) {
            return false;
        }
        delay(1);
    }
    return true;
}

void send_command_no_wait(uint8_t cmd) {
    digitalWrite(PIN_CS, LOW);
    SPI.transfer(cmd);
    delayMicroseconds(2);
    digitalWrite(PIN_CS, HIGH);
}

// Thin wrapper to centralize ADS1256 lifetime and one-time init.
class LoadCellADC {
    public:
        LoadCellADC() : adc(k_adc_clock_mhz, k_adc_vref, /*useresetpin=*/false, PIN_DRDY, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS) {}

        bool ensure_initialized(const LoadCellConfig &cfg) {
            if (initialized) return true;
            LogOutput::printf("ADS1256: Starting ADC");
            adc.initSpi(k_adc_clock_mhz);
            delay(1000);
            bool drdy_ready = wait_drdy_timeout(k_adc_drdy_timeout_us);
            if (!drdy_ready) {
                LogOutput::printf("ADS1256: DRDY timeout, issuing reset");
                send_command_no_wait(ADS1256_CMD_RESET);
                delay(k_adc_reset_delay_ms);
                drdy_ready = wait_drdy_timeout(k_adc_drdy_timeout_us);
            }
            if (!drdy_ready) {
                LogOutput::printf("ADS1256: DRDY still high, skipping init");
                return false;
            }

            // Start with configured sample rate and gain
            adc.begin(ADC_SAMPLE_RATE, ADS1256_GAIN_64, false);
            LogOutput::printf(" -> started");

            if (!wait_drdy_timeout(k_adc_drdy_timeout_us)) {
                LogOutput::printf("ADS1256: DRDY timeout after start");
                return false;
            }
            float conv = calc_conversion_factor(cfg);
            if (fabs(conv) > 0.01f) {
                adc.setConversionFactor(conv);
            } else {
                adc.setConversionFactor(1.0f);
            }
            initialized = true;
            return true;
        }

        ADS1256 &ref() {
            return adc;
        }

    private:
        ADS1256 adc;
        bool initialized = false;
};

LoadCellADC &adc_instance() {
    static LoadCellADC instance;
    return instance;
}
}  // namespace

bool LoadCellAds1256::begin() const {
    return adc_instance().ensure_initialized(_cfg) && (adc_instance().ref().setChannel(_cfg.channel_p, _cfg.channel_n), true);
}

void LoadCellAds1256::set_loadcell_rating(uint8_t load_cell_rating_u8) {
    if (!begin()) return;
    ADS1256 &adc = adc_instance().ref();
    float original_conversion_factor = calc_conversion_factor(_cfg);

    float updated_conversion_factor = 1.0f;
    if (_cfg.loadcell_rating_kg > 0.0f) {
        updated_conversion_factor = (static_cast<float>(load_cell_rating_u8) * (original_conversion_factor / _cfg.loadcell_rating_kg));
    }
    LogOutput::printf("ADS1256: Updating conversion factor (%.3f -> %.3f)", original_conversion_factor, updated_conversion_factor);

    adc.setConversionFactor(updated_conversion_factor);
    _cfg.loadcell_rating_kg = load_cell_rating_u8;
}

LoadCellAds1256::LoadCellAds1256(const LoadCellConfig &cfg)
    : _cfg(cfg), _zero_point(0.0f), _variance_estimate(k_default_variance), _standard_deviation_estimate(sqrtf(k_default_variance)) {
    begin();
}

bool LoadCellAds1256::try_get_reading_kg(float &reading_kg) const {
    if (!begin()) return false;
    ADS1256 &adc = adc_instance().ref();
    float raw = adc.readCurrentChannel() - _zero_point;
    reading_kg = filter_reading(raw);
    return true;
}

float LoadCellAds1256::get_reading_kg() const {
    float reading = 0.0f;
    if (!try_get_reading_kg(reading)) {
        return _zero_point;  // best effort fallback
    }
    return reading;
}

bool LoadCellAds1256::set_zero_point(uint32_t sample_count) {
    if (!begin()) return false;
    const uint32_t samples = sample_count ? sample_count : _cfg.offset_samples;
    if (samples == 0) return false;

    LogOutput::printf("ADS1256: Identifying loadcell offset (%lu samples)...", static_cast<unsigned long>(samples));

    float loadcell_offset = 0.0f;
    for (uint32_t i = 0; i < samples; i++) {
        loadcell_offset += get_reading_kg();
    }
    loadcell_offset /= static_cast<float>(samples);

    LogOutput::printf(" -> offset = %.3f", loadcell_offset);

    _zero_point = loadcell_offset;
    return true;
}

bool LoadCellAds1256::estimate_variance(uint32_t sample_count) {
    if (!begin()) return false;
    const uint32_t samples = sample_count ? sample_count : _cfg.variance_samples;
    if (samples < 2) return false;

    LogOutput::printf("ADS1256: Identifying loadcell variance (%lu samples)...", static_cast<unsigned long>(samples));
    const float var_normalizer = 1.0f / static_cast<float>(samples - 1);
    float variance = 0.0f;
    for (uint32_t i = 0; i < samples; i++) {
        float loadcell_reading = get_reading_kg();
        variance += sq(loadcell_reading) * var_normalizer;
    }

    if (variance < k_variance_min) {
        variance = k_variance_min;
    }

    _standard_deviation_estimate = sqrtf(variance);

    const float sigma3 = _standard_deviation_estimate * 3.0f;
    const float variance3 = variance * 9.0f;

    LogOutput::printf(" -> variance est. = %.5f", variance);
    LogOutput::printf(" -> stddev est.   = %.5f", _standard_deviation_estimate);
    LogOutput::printf(" -> 3-sigma est.  = %.5f (variance eq.: %.5f)", sigma3, variance3);

    _variance_estimate = variance3;  // keep backward-compatible “3*sigma squared” storage
    return true;
}

float LoadCellAds1256::filter_reading(float raw) const {
    // Spike rejection relative to last filtered sample
    const float min_threshold = 0.5f;  // kg
    float threshold = _standard_deviation_estimate * 4.0f;
    if (threshold < min_threshold) threshold = min_threshold;
    float candidate = raw;
    float delta = candidate - _last_filtered;
    if (fabsf(delta) > threshold) {
        candidate = _last_filtered;
    }

    // Median-of-3 prefilter
    _recent_samples[_recent_idx] = raw;
    _recent_idx = (_recent_idx + 1) % 3;
    if (_recent_count < 3) _recent_count++;

    float a = _recent_samples[0];
    float b = _recent_samples[1];
    float c = _recent_samples[2];
    float median = candidate;
    if (_recent_count == 1) {
        median = a;
    } else if (_recent_count == 2) {
        median = (a + b) * 0.5f;
    } else {
        // median of 3
        if ((a > b) != (a > c)) median = a;
        else if ((b > a) != (b > c)) median = b;
        else median = c;
    }

    _last_filtered = median;
    return median;
}
