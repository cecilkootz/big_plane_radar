#pragma once

#include <stddef.h>
#include <stdint.h>

namespace BatteryGauge {

// Waveshare 7B IO-extension ADC: 10 bits, 3.3V reference, 3:1 divider.
constexpr float ADC_COUNTS_TO_VOLTS = 3.0f * 3.3f / 1023.0f;

inline float voltsFromAdcCounts(uint16_t counts) {
    return static_cast<float>(counts) * ADC_COUNTS_TO_VOLTS;
}

// Single-cell LiPo open-circuit voltage to state of charge.
inline int percentFromVoltage(float volts) {
    struct Point {
        float volts;
        int percent;
    };
    static constexpr Point CURVE[] = {
        {4.20f, 100}, {4.10f, 92}, {4.00f, 80}, {3.92f, 68},
        {3.85f, 56}, {3.78f, 44}, {3.72f, 34}, {3.65f, 22},
        {3.58f, 12}, {3.50f, 6}, {3.40f, 2}, {3.30f, 0},
    };
    constexpr size_t COUNT = sizeof(CURVE) / sizeof(CURVE[0]);
    if (volts >= CURVE[0].volts) return 100;
    if (volts <= CURVE[COUNT - 1].volts) return 0;
    for (size_t i = 1; i < COUNT; i++) {
        if (volts >= CURVE[i].volts) {
            const Point &high = CURVE[i - 1];
            const Point &low = CURVE[i];
            float t = (volts - low.volts) / (high.volts - low.volts);
            return low.percent +
                static_cast<int>(t * static_cast<float>(high.percent - low.percent) + 0.5f);
        }
    }
    return 0;
}

// Infers external power vs battery discharge from voltage alone, because the
// board's CS8501 power-path chip exposes no VBUS signal. Plug/unplug events
// appear as voltage steps between consecutive samples; steady state is
// classified by average level with hysteresis. Level rules stay silent until
// the averaging window has fully turned over after a flip, so they cannot
// flip back based on pre-event samples. The step threshold sits above the
// transient our own backlight dimming causes, so steps stay trusted.
class PowerStateTracker {
public:
    static constexpr size_t WINDOW = 8;
    static constexpr float EXTERNAL_MIN_VOLTS = 4.13f;
    static constexpr float BATTERY_MAX_VOLTS = 4.02f;
    static constexpr float STEP_VOLTS = 0.08f;
    // Approximate IR sag of the pack under the boost converter's load.
    static constexpr float LOAD_SAG_COMPENSATION_VOLTS = 0.08f;

    void addSample(float volts) {
        bool havePrevious = _count > 0;
        float previous = havePrevious
            ? _samples[(_head + WINDOW - 1) % WINDOW]
            : 0.0f;
        _samples[_head] = volts;
        _head = (_head + 1) % WINDOW;
        if (_count < WINDOW) _count++;
        if (_samplesSinceFlip < WINDOW) _samplesSinceFlip++;

        if (havePrevious) {
            float dv = volts - previous;
            if (dv >= STEP_VOLTS) {
                setState(false);
                return;
            }
            if (dv <= -STEP_VOLTS) {
                setState(true);
                return;
            }
        }
        if (!primed() || _samplesSinceFlip < WINDOW) return;
        float average = filteredVolts();
        if (average >= EXTERNAL_MIN_VOLTS) {
            setState(false);
        } else if (average <= BATTERY_MAX_VOLTS) {
            setState(true);
        }
    }

    bool primed() const { return _count >= WINDOW; }
    bool onBattery() const { return _onBattery; }

    float filteredVolts() const {
        if (_count == 0) return 0.0f;
        float sum = 0.0f;
        for (size_t i = 0; i < _count; i++) {
            sum += _samples[i];
        }
        return sum / static_cast<float>(_count);
    }

    // Quantized to 5% steps so ADC noise does not flicker the indicator.
    int displayPercent() const {
        float volts = filteredVolts() +
            (_onBattery ? LOAD_SAG_COMPENSATION_VOLTS : 0.0f);
        int percent = percentFromVoltage(volts);
        return (percent + 2) / 5 * 5;
    }

private:
    void setState(bool onBattery) {
        if (_onBattery == onBattery) return;
        _onBattery = onBattery;
        _samplesSinceFlip = 0;
    }

    float _samples[WINDOW] = {};
    size_t _head = 0;
    size_t _count = 0;
    size_t _samplesSinceFlip = WINDOW;
    bool _onBattery = false;
};

}  // namespace BatteryGauge
