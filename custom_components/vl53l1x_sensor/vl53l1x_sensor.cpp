#include "esphome/core/log.h"
#include "vl53l1x_sensor.h"

namespace esphome {
namespace vl53l1x {

static const char *const TAG = "vl53l1x";

std::list<VL53L1XSensor *> VL53L1XSensor::vl53_sensors;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

VL53L1XSensor::VL53L1XSensor() { VL53L1XSensor::vl53_sensors.push_back(this); }

void VL53L1XSensor::setup() {
  
}

void VL53L1XSensor::loop() {
  
}

void VL53L1XSensor::update() {

}

void VL53L1XSensor::dump_config() {
    ESP_LOGCONFIG(TAG, "Empty custom sensor");
}

} //namespace vl53l1x
} //namespace esphome
