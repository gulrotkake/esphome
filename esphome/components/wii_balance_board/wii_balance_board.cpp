#include "esphome/core/log.h"
#include "wii_balance_board.h"

#include "esphome/core/application.h"

#include <numeric>
#include "utils.h"

namespace esphome {
namespace wii_balance_board {

static const char *TAG = "wii_balance_board.component";

uint8_t interpret_battery_level(uint8_t batteryLevel) {
  if (batteryLevel >= 0x8d) {
    return 100;
  } else if (batteryLevel >= 0x7d) {
    return 75;
  } else if (batteryLevel >= 0x78) {
    return 50;
  } else if (batteryLevel >= 0x6A) {
    return 25;
  } else {
    return 0;
  }
}

WiiBalanceBoard::WiiBalanceBoard() : wii(&bluetooth), std_dev_(0.4) {}

void WiiBalanceBoard::board_connected(uint16_t handle) {
  ESP_LOGI(TAG, "Connected board, scheduling disconnect in max 60 seconds");
  // wiimote->set_led(handle, 1);

  if (sampleMap.count(handle) > 0) {
    ESP_LOGE(TAG, "Same handle connected twice, ignoring connection.");
  } else {
    // Queue sampling timeout
    sampleMap.emplace(handle, Sample());

    // Schedule timeout disconnect
    queue.add(handle, millis() + 60000, [this](int handle) {
      ESP_LOGI(TAG, "Scheduled disconnect.");
      wii.disconnect(handle, 0x0011);
      wii.disconnect(handle, 0x0013);
    });
  }
}

void WiiBalanceBoard::board_disconnected(uint16_t handle) {
  ESP_LOGI(TAG, "Board disconnected, uploaded sampled data.");
  if (sampleMap.count(handle) > 0) {
    auto &sample = sampleMap[handle];
    if (sample.referenceTemperature > 0) {
      reference_temperature_sensor_->publish_state(sample.referenceTemperature);
      temperature_sensor_->publish_state(sample.temperature);
      battery_level_->publish_state(sample.battery);
    }
    if (!isnan(sample.measurement)) {
      weight_->publish_state(sample.measurement);
    }
    sampleMap.erase(handle);
  }
}

void WiiBalanceBoard::board_sample(uint16_t handle, uint8_t battery, uint8_t reference_temp, uint8_t temperature,
                                   float topRightLoad, float bottomRightLoad, float topLeftLoad, float bottomLeftLoad) {
  Sample &sample = sampleMap.at(handle);

  // Ignore zero data
  if (reference_temp == 0 || !isnan(sample.measurement)) {
    return;
  }

  sample.referenceTemperature = reference_temp;
  sample.battery = battery;
  sample.temperature = temperature;

  float totalWeight = (topRightLoad + bottomRightLoad + topLeftLoad + bottomLeftLoad) / 1000;
  float adjusted = (.999 * totalWeight * (1.0 - .0007 * (sample.temperature - sample.referenceTemperature)));

  // Ignore small samples (noise), in std dev calculation.
  if (adjusted < 10) {
    return;
  }

  int size = 64;
  sample.samples[sample.sample_count] = adjusted;
  sample.sample_count = (sample.sample_count + 1) % size;

  // Not enough samples yet
  if (isnan(sample.samples[size - 1])) {
    return;
  }

  // For every 16th data point, sample standard deviation.
  if (sample.sample_count % 16 == 0) {
    float mean = 0;
    for (size_t i = 0; i < size; ++i) {
      mean += sample.samples[i];
    }
    mean /= size;

    float variance = std::accumulate(sample.samples, sample.samples + size, 0.0,
                                     [&mean, &size](float accumulator, const float &val) {
                                       return accumulator + ((val - mean) * (val - mean) / (size - 1));
                                     });

    float deviation = std::sqrt(variance);

    if (mean > 10 && deviation < std_dev_) {  // Ignore all means below 10kg.
      sample.measurement = mean;

      // We have a valid sample, schedule board disconnect.
      ESP_LOGD(TAG, "Sample valid, disconnecting");
      queue.reschedule(handle, millis() + 100);
    }
  }
}

void WiiBalanceBoard::setup() {
  if (led_pin_ >= 0) {
    pinMode(led_pin_, OUTPUT);
    digitalWrite(led_pin_, HIGH);
  }
  bluetooth.onReady([](auto) { ESP_LOGI(TAG, "Bluetooth initialized"); });

  wii.onEvent([this](const detail::WiiEvent &event) {
    std::visit(overloaded{
                   [this](const detail::ScanStarted &) {
                     syncing_->publish_state(true);
                     if (led_pin_ >= 0) {
                       digitalWrite(led_pin_, LOW);
                     }
                   },
                   [this](const detail::ScanStopped &) {
                     syncing_->publish_state(false);
                     if (led_pin_ >= 0) {
                       digitalWrite(led_pin_, HIGH);
                     }
                   },
                   [this](const detail::BalanceBoardConnected &board) {
                     syncing_->publish_state(false);
                     if (led_pin_ >= 0) {
                       digitalWrite(led_pin_, HIGH);
                     }
                     sync(false);
                     this->board_connected(board.handle);
                   },
                   [this](const detail::BalanceBoardDisconnected &board) { this->board_disconnected(board.handle); },
                   [this](const detail::BalanceBoardData &data) {
                     this->board_sample(data.handle, interpret_battery_level(data.batteryLevel),
                                        data.referenceTemperature, data.temperature, data.tr, data.br, data.tl,
                                        data.bl);
                   },
               },
               event);
  });
}

void WiiBalanceBoard::loop() {
  wii.step();
  queue.process(millis());
}

void WiiBalanceBoard::sync(bool enable) {
  ESP_LOGI(TAG, enable ? "Starting scan" : "Stopping scan");
  wii.sync(enable);
}

void WiiBalanceBoard::dump_config() { ESP_LOGCONFIG(TAG, "Wii Balance Board"); }

void WiiBalanceBoard::set_temperature_sensor(sensor::Sensor *temperature_sensor) {
  temperature_sensor_ = temperature_sensor;
}
void WiiBalanceBoard::set_reference_temperature_sensor(sensor::Sensor *reference_temperature_sensor) {
  reference_temperature_sensor_ = reference_temperature_sensor;
}
void WiiBalanceBoard::set_battery_level(sensor::Sensor *battery_level) { battery_level_ = battery_level; }
void WiiBalanceBoard::set_weight(sensor::Sensor *weight) { weight_ = weight; }
void WiiBalanceBoard::set_stddev(float stddev) { this->std_dev_ = stddev; }
void WiiBalanceBoard::set_led_pin(int led_pin) { this->led_pin_ = led_pin; }
void WiiBalanceBoard::set_syncing(binary_sensor::BinarySensor *syncing) { this->syncing_ = syncing; }

}  // namespace wii_balance_board
}  // namespace esphome
