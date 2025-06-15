#pragma once
#include <freertos/ringbuf.h>

#include <memory>

namespace esphome::wii_balance_board::detail {

class RingData {
  uint8_t *m_data;
  size_t m_size;
  std::function<void(uint8_t *)> m_completion;

 public:
  RingData(uint8_t *data, size_t size, std::function<void(uint8_t *)> completion)
      : m_data(data), m_size(size), m_completion(completion) {}
  RingData(RingData &) = delete;
  RingData &operator=(const RingData &) = delete;

  ~RingData() { m_completion(m_data); }

  const uint8_t &operator[](size_t idx) const { return m_data[idx]; }

  size_t size() const { return m_size; }
  uint8_t *data() const { return m_data; }

  size_t size() { return m_size; }
  uint8_t *data() { return m_data; }
  uint8_t &operator[](size_t idx) { return m_data[idx]; }

  operator bool() const { return m_data != nullptr; }
};

class RingBuffer {
  RingbufHandle_t buf;

 public:
  RingBuffer(size_t size) : buf(xRingbufferCreate(size, RINGBUF_TYPE_NOSPLIT)) {}

  RingBuffer(RingBuffer &) = delete;
  RingBuffer &operator=(const RingBuffer &) = delete;

  ~RingBuffer() { vRingbufferDelete(buf); }

  const RingData read(int ms) {
    size_t rxSize;
    uint8_t *rxData = (uint8_t *) xRingbufferReceive(buf, &rxSize, pdMS_TO_TICKS(ms));

    return RingData(rxData, rxSize, [this](uint8_t *data) {
      if (data != nullptr) {
        vRingbufferReturnItem(buf, (void *) data);
      }
    });
  }

  void clear() {
    size_t rxSize;
    while (true) {
      auto *data = xRingbufferReceive(buf, &rxSize, 0);
      if (data == nullptr) {
        break;
      }
      vRingbufferReturnItem(buf, data);
    }
    log_d("Buffer flushed");
  }

  RingData allocate(size_t size, int ms = 0) {
    uint8_t *data;
    auto res = xRingbufferSendAcquire(buf, (void **) &data, size, ms);
    if (res != pdTRUE) {
      return RingData(nullptr, 0, [](auto...) {});
    }
    return RingData(data, size, [this](uint8_t *data) { xRingbufferSendComplete(buf, data); });
  }
};

}  // namespace esphome::wii_balance_board::detail
