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
    void set_irq_pin(GPIOPin *irq) { this->irq_pin_ = irq; }

    private:
    void startRanging();
    void stopRanging();
    bool checkForDataReady();
    uint8_t getInterruptPolarity();
    void clearInterrupt();
    uint16_t sensorId();
    uint16_t readWord(uint16_t reg_idx);


    static std::list<VL53L1XSensor *> vl53_sensors;
    GPIOPin *enable_pin_{nullptr};
    GPIOPin *irq_pin_{nullptr};
    static bool enable_pin_setup_complete;
    uint16_t timeout_start_us_;
    uint16_t timeout_us_{};
};

} //namespace vl53l1x
} //namespace esphome
