#include "esphome/core/log.h"
#include "vl53l1x_sensor.h"

namespace esphome {
namespace vl53l1x {

static const char *const TAG = "vl53l1x";

std::list<VL53L1XSensor *> VL53L1XSensor::vl53_sensors;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
bool VL53L1XSensor::enable_pin_setup_complete = false;   // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

const uint8_t VL51L1X_DEFAULT_CONFIGURATION[] =
{
   0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
   0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
   0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
   0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity() */
   0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady() */
   0x00, /* 0x32 : not user-modifiable */
   0x02, /* 0x33 : not user-modifiable */
   0x08, /* 0x34 : not user-modifiable */
   0x00, /* 0x35 : not user-modifiable */
   0x08, /* 0x36 : not user-modifiable */
   0x10, /* 0x37 : not user-modifiable */
   0x01, /* 0x38 : not user-modifiable */
   0x01, /* 0x39 : not user-modifiable */
   0x00, /* 0x3a : not user-modifiable */
   0x00, /* 0x3b : not user-modifiable */
   0x00, /* 0x3c : not user-modifiable */
   0x00, /* 0x3d : not user-modifiable */
   0xff, /* 0x3e : not user-modifiable */
   0x00, /* 0x3f : not user-modifiable */
   0x0F, /* 0x40 : not user-modifiable */
   0x00, /* 0x41 : not user-modifiable */
   0x00, /* 0x42 : not user-modifiable */
   0x00, /* 0x43 : not user-modifiable */
   0x00, /* 0x44 : not user-modifiable */
   0x00, /* 0x45 : not user-modifiable */
   0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
   0x0b, /* 0x47 : not user-modifiable */
   0x00, /* 0x48 : not user-modifiable */
   0x00, /* 0x49 : not user-modifiable */
   0x02, /* 0x4a : not user-modifiable */
   0x0a, /* 0x4b : not user-modifiable */
   0x21, /* 0x4c : not user-modifiable */
   0x00, /* 0x4d : not user-modifiable */
   0x00, /* 0x4e : not user-modifiable */
   0x05, /* 0x4f : not user-modifiable */
   0x00, /* 0x50 : not user-modifiable */
   0x00, /* 0x51 : not user-modifiable */
   0x00, /* 0x52 : not user-modifiable */
   0x00, /* 0x53 : not user-modifiable */
   0xc8, /* 0x54 : not user-modifiable */
   0x00, /* 0x55 : not user-modifiable */
   0x00, /* 0x56 : not user-modifiable */
   0x38, /* 0x57 : not user-modifiable */
   0xff, /* 0x58 : not user-modifiable */
   0x01, /* 0x59 : not user-modifiable */
   0x00, /* 0x5a : not user-modifiable */
   0x08, /* 0x5b : not user-modifiable */
   0x00, /* 0x5c : not user-modifiable */
   0x00, /* 0x5d : not user-modifiable */
   0x01, /* 0x5e : not user-modifiable */
   0xcc, /* 0x5f : not user-modifiable */
   0x0f, /* 0x60 : not user-modifiable */
   0x01, /* 0x61 : not user-modifiable */
   0xf1, /* 0x62 : not user-modifiable */
   0x0d, /* 0x63 : not user-modifiable */
   0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm  */
   0x68, /* 0x65 : Sigma threshold LSB */
   0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold() */
   0x80, /* 0x67 : Min count Rate LSB */
   0x08, /* 0x68 : not user-modifiable */
   0xb8, /* 0x69 : not user-modifiable */
   0x00, /* 0x6a : not user-modifiable */
   0x00, /* 0x6b : not user-modifiable */
   0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs() */
   0x00, /* 0x6d : Intermeasurement period */
   0x0f, /* 0x6e : Intermeasurement period */
   0x89, /* 0x6f : Intermeasurement period LSB */
   0x00, /* 0x70 : not user-modifiable */
   0x00, /* 0x71 : not user-modifiable */
   0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold() */
   0x00, /* 0x73 : distance threshold high LSB */
   0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold() */
   0x00, /* 0x75 : distance threshold low LSB */
   0x00, /* 0x76 : not user-modifiable */
   0x01, /* 0x77 : not user-modifiable */
   0x0f, /* 0x78 : not user-modifiable */
   0x0d, /* 0x79 : not user-modifiable */
   0x0e, /* 0x7a : not user-modifiable */
   0x0e, /* 0x7b : not user-modifiable */
   0x00, /* 0x7c : not user-modifiable */
   0x00, /* 0x7d : not user-modifiable */
   0x02, /* 0x7e : not user-modifiable */
   0xc7, /* 0x7f : ROI center, use SetROI() */
   0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
   0x9B, /* 0x81 : not user-modifiable */
   0x00, /* 0x82 : not user-modifiable */
   0x00, /* 0x83 : not user-modifiable */
   0x00, /* 0x84 : not user-modifiable */
   0x01, /* 0x85 : not user-modifiable */
   0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
   0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87 */
};

VL53L1XSensor::VL53L1XSensor() { VL53L1XSensor::vl53_sensors.push_back(this); }

void VL53L1XSensor::setup() {
    ESP_LOGD(TAG, "'%s' - setup BEGIN", this->name_.c_str());

    if (!esphome::vl53l1x::VL53L1XSensor::enable_pin_setup_complete) {
      for (auto &vl53_sensor : vl53_sensors) {
        if (vl53_sensor->enable_pin_ != nullptr) {
          // Set enable pin as OUTPUT and disable the enable pin to force vl53 to HW Standby mode
          vl53_sensor->enable_pin_->setup();
          vl53_sensor->enable_pin_->digital_write(false);
        }
      }
      esphome::vl53l1x::VL53L1XSensor::enable_pin_setup_complete = true;
    }

    if (this->enable_pin_ != nullptr) {
      // Enable the enable pin to cause FW boot (to get back to 0x29 default address)
      this->enable_pin_->digital_write(true);
      delayMicroseconds(100);
    }

    // Save the i2c address we want and force it to use the default 0x29
    // until we finish setup, then re-address to final desired address.
    uint8_t final_address = address_;
    this->set_i2c_address(0x29);

    {
    uint8_t byteData;
    uint8_t wordData;
      wordData = sensorId(); // VL53L1_IDENTIFICATION__MODEL_ID
      ESP_LOGD(TAG, "VL53L1X Model_ID: 0x%04X", wordData);
      byteData = reg16(0x0110).get(); // VL53L1_IDENTIFICATION__MODULE_TYPE
      ESP_LOGD(TAG, "VL53L1X Module_Type: 0x%02X", byteData);
      wordData = readWord(0x010F); // VL53L1_IDENTIFICATION__MODEL_ID
      ESP_LOGD(TAG, "VL53L1X: 0x%04X", wordData);
    }

      this->timeout_start_us_ = micros();
        while (reg16(0x00E5).get() == 0x00) {
          if (this->timeout_us_ > 0 && ((uint16_t) (micros() - this->timeout_start_us_) > this->timeout_us_)) {
            ESP_LOGE(TAG, "'%s' - setup timeout", this->name_.c_str());
            this->mark_failed();
            return;
          }
          yield();
        }
        ESP_LOGD(TAG, "'%s' - SensorInit", this->name_.c_str());
        // SensorInit
        uint8_t Addr = 0x00;
        for (Addr = 0x2D; Addr <= 0x87; Addr++)
           {
            reg(Addr) = VL51L1X_DEFAULT_CONFIGURATION[Addr - 0x2D];
           }
       ESP_LOGD(TAG, "'%s' - startRanging", this->name_.c_str());
       startRanging();
       bool ready = 0;
       ESP_LOGD(TAG, "'%s' - checkForDataReady", this->name_.c_str());
       while(ready==0)
      {
         ready = checkForDataReady();
      }
      ESP_LOGD(TAG, "'%s' - clearInterrupt", this->name_.c_str());
      clearInterrupt();
      ESP_LOGD(TAG, "'%s' - stopRanging", this->name_.c_str());
      stopRanging();
      ESP_LOGD(TAG, "'%s' - 0x0008=0x09 0x000b=0x00", this->name_.c_str());
      reg(0x0008) = 0x09;
      reg(0x000b) = 0x00;
      uint16_t sensor_id = sensorId();
      if(sensor_id != 0xEACC){
        ESP_LOGE(TAG,"Wrong sensor id for '%s': 0x%04X", this->name_.c_str(), sensor_id);
        this->mark_failed();
        return;
      }
      ESP_LOGI(TAG,"'%s' - Setup completed", this->name_.c_str());
}

void VL53L1XSensor::loop() {
  
}

void VL53L1XSensor::update() {

}

void VL53L1XSensor::dump_config() {
    LOG_SENSOR("", "VL53L1X", this);
    LOG_UPDATE_INTERVAL(this);
    LOG_I2C_DEVICE(this);
    if (this->enable_pin_ != nullptr) {
        LOG_PIN("  Enable Pin: ", this->enable_pin_);
    }
    if (this->irq_pin_ != nullptr) {
        LOG_PIN("  IRQ Pin: ", this->irq_pin_);
    }
    ESP_LOGCONFIG(TAG, "  Timeout: %u%s", this->timeout_us_, this->timeout_us_ > 0 ? "us" : " (no timeout)");
}

void VL53L1XSensor::startRanging(){
    reg16(0x0087) = 0x40;
}

void VL53L1XSensor::stopRanging(){
    reg(0x0087) = 0x00;
}


bool VL53L1XSensor::checkForDataReady(){

   uint8_t intPol = getInterruptPolarity();
   uint8_t temp = reg(0x0031).get();
   /* Read in the register to check if a new value is available */
  if ((temp & 1) == intPol)
     return 1;
  else
     return 0;
}

uint8_t VL53L1XSensor::getInterruptPolarity() {
   uint8_t temp = reg(0x0030).get();
   temp = temp & 0x10;
   return !(temp>>4);
}

void VL53L1XSensor::clearInterrupt(){
    reg(0x0086) = 0x01;
}

uint16_t VL53L1XSensor::sensorId()
{

   return readWord(0x010F);
}

uint16_t VL53L1XSensor::readWord(uint16_t reg_idx){
    uint8_t buffer[2] = {0,0};
    buffer[0] = reg(reg_idx).get();
    buffer[1] = reg(reg_idx + 1).get();
    uint16_t data = (buffer[0] << 8) + buffer[1];
    return data;
}

} //namespace vl53l1x
} //namespace esphome
