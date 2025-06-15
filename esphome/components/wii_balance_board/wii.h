#pragma once
#include "bluetooth.h"
#include <unordered_map>
#include <memory>

namespace esphome::wii_balance_board::detail {

struct BalanceBoardConnected {
  uint16_t handle;
};

struct BalanceBoardDisconnected {
  uint16_t handle;
};

struct BalanceBoardData {
  uint16_t handle;
  uint16_t tr;
  uint16_t br;
  uint16_t tl;
  uint16_t bl;
  uint8_t referenceTemperature;
  uint8_t temperature;
  uint8_t batteryLevel;
};

struct ScanStarted {};

struct ScanStopped {};

using WiiEvent =
    std::variant<BalanceBoardConnected, BalanceBoardDisconnected, BalanceBoardData, ScanStarted, ScanStopped>;

class Wii {
  struct BalanceBoard;
  Bluetooth *bluetooth;
  std::unordered_map<uint16_t, std::unique_ptr<BalanceBoard>> connectedBoards;
  std::function<void(const WiiEvent &)> eventListener;

 public:
  Wii(Bluetooth *bluetooth);
  ~Wii();
  Wii(const Wii &) = delete;
  Wii &operator=(const Wii &) = delete;

  void onEvent(std::function<void(const WiiEvent &)> eventListener);
  void sync(bool enable);
  void step();

  void disconnect(uint16_t handle, uint16_t psm);
};

}  // namespace esphome::wii_balance_board::detail
