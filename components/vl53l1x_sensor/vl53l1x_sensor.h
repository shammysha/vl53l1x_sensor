#pragma once
#include <list>
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace vl53l1x {

class VL53L1XSensor : public sensor::Sensor, public PollingComponent, public i2c::I2CDevice {
    public:
    VL53L1XSensor();

    void setup() override;

    void update() override;
    void dump_config() override;

    void loop() override;

    void set_timeout_us(uint32_t timeout_us) { this->timeout_us_ = timeout_us; }
    void set_enable_pin(GPIOPin *enable) { this->enable_pin_ = enable; }
    //void set_irq_pin(GPIOPin *irq) { this->irq_pin_ = irq; }
    void set_distance_mode(uint8_t dm) { this->distance_mode_ = dm; };
    void set_timing_budget(uint16_t ms) { this->timing_budget_ms_ = ms; };
    void set_signal_threshold(uint16_t signal) { this->signal_threshold_ = signal; };

    private:
    void setI2CAddress(uint8_t addr);
    void startRanging();
    void stopRanging();
    bool checkForDataReady();
    uint8_t getInterruptPolarity();
    void clearInterrupt();
    uint16_t sensorId();
    uint16_t readWord(uint16_t reg_idx);
    void writeWord(uint16_t reg_idx, uint16_t data);

    int8_t getRangeStatus();
    int16_t distance();
    void set_distance_mode();
    void set_timing_budget();

    static std::list<VL53L1XSensor *> vl53_sensors;
    GPIOPin *enable_pin_{nullptr};
    GPIOPin *irq_pin_{nullptr};
    static bool enable_pin_setup_complete;
    uint16_t timeout_start_us_;
    uint16_t timeout_us_{};
    uint8_t distance_mode_{};
    uint16_t timing_budget_ms_{};
    uint16_t signal_threshold_{};
    uint8_t rangeStatus{};
};

} //namespace vl53l1x
} //namespace esphome
