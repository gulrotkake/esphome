#include "wii.h"
#include "log.h"

#include "bluetooth.h"

#include <bitset>
#include <array>
#include <cstring>
#include "utils.h"

namespace esphome::wii_balance_board::detail {

class Wii::BalanceBoard {
  Bluetooth *bt;
  int queryState;
  uint16_t handle;
  std::array<uint16_t, 12> calibration;

  uint8_t referenceTemperature{0};

 public:
  BalanceBoard(Bluetooth *bt, uint16_t handle) : bt(bt), handle(handle), queryState(0) {}

  void setLeds(Bluetooth *bt, uint16_t handle, const std::bitset<4> &bits) {
    uint8_t ledData[] = {
        0xA2,
        0x11,
        static_cast<uint8_t>(bits.to_ulong() << 4),
    };

    bt->l2send_data(handle, 0x0013, ledData, 3);
  }

  void set_reporting_mode(uint16_t handle, uint8_t reportingMode, bool continuous) {
    uint8_t data[] = {0xA2, 0x12, (uint8_t) (continuous ? 0x04 : 0x00), reportingMode};
    bt->l2send_data(handle, 0x0013, data, 4);
  }

  void write_memory(uint16_t handle, uint8_t addressSpace, uint32_t offset, std::initializer_list<uint8_t> memData) {
    uint8_t data[] = {0xA2,
                      0x16,
                      addressSpace,
                      (uint8_t) ((offset >> 16) & 0xFF),
                      (uint8_t) ((offset >> 8) & 0xFF),
                      (uint8_t) ((offset) &0xFF),
                      (uint8_t) (memData.size()),
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00,
                      0x00};
    std::copy(memData.begin(), memData.end(), data + 7);

    bt->l2send_data(handle, 0x0013, data, 23);
  }

  void read_memory(uint16_t handle, uint8_t addressSpace, uint32_t offset, uint16_t size) {
    uint8_t data[] = {0xA2,
                      0x17,
                      addressSpace,
                      (uint8_t) ((offset >> 16) & 0xFF),
                      (uint8_t) ((offset >> 8) & 0xFF),
                      (uint8_t) ((offset) &0xFF),
                      (uint8_t) ((size >> 8) & 0xFF),
                      (uint8_t) ((size) &0xFF)};
    bt->l2send_data(handle, 0x0013, data, 8);

    uint16_t data_len = 8;
  }

  int32_t interpolate(uint8_t pos, uint16_t *values) {
    uint16_t *cal = calibration.data();
    float weight = 0;

    if (values[pos] < cal[pos]) {  // 0kg
      weight = 0;
    } else if (values[pos] < cal[pos + 4]) {  // 17kg
      weight = 17 * (float) (values[pos] - cal[pos]) / (float) (cal[pos + 4] - cal[pos]);
    } else {  // 34kg
      weight = 17 + 17 * (float) (values[pos] - cal[pos + 4]) / (float) (cal[pos + 8] - cal[pos + 4]);
    }

    return weight * 1000;
  }

  void readCalibrationData(uint8_t *data, size_t len) {
    switch (queryState) {
      case 0:
        if (data[1] == 0x20 && data[4] & 0x02) {
          write_memory(handle, 0x04, 0xA400F0, {0x55});  // Disable encryption 1
          queryState = 1;
        }
        break;
      case 1:
        if (data[1] == 0x22 && data[4] == 0x16) {
          if (data[5] == 0x00) {
            write_memory(handle, 0x04, 0xA400FB, {0x00});  // Disable encryption 2
            queryState = 2;
          } else {
            queryState = 0;
          }
        }
        break;
      case 2:
        if (data[1] == 0x22 && data[4] == 0x16) {
          if (data[5] == 0x00) {
            read_memory(handle, 0x04, 0xA400FA, 6);
            queryState = 3;
          } else {
            queryState = 0;
          }
        }
        break;
      case 3:
        if (data[1] == 0x21) {
          // Wii Balance Board
          if (memcmp(data + 5, (const uint8_t[]){0x00, 0xFA, 0x00, 0x00, 0xA4, 0x20, 0x04, 0x02}, 8) == 0) {
            read_memory(handle, 0x04, 0xA40024, 16);  // read calibration 0 kg and 17kg
            queryState = 4;
          } else {
            queryState = 0;
          }
        }
        break;
      case 4: {
        log_d("Calibration data 0kg and 17kg");
        uint8_t *mem = data + 7;

        calibration[0] = mem[0] * 256 + mem[1];  // Top Right 0kg
        calibration[1] = mem[2] * 256 + mem[3];  // Bottom Right 0kg
        calibration[2] = mem[4] * 256 + mem[5];  // Top Left 0kg
        calibration[3] = mem[6] * 256 + mem[7];  // Bottom Left 0kg

        calibration[4] = mem[8] * 256 + mem[9];    // Top Right 17kg
        calibration[5] = mem[10] * 256 + mem[11];  // Bottom Right 17kg
        calibration[6] = mem[12] * 256 + mem[13];  // Top Left 17kg
        calibration[7] = mem[14] * 256 + mem[15];  // Bottom Left 17kg
      }
        read_memory(handle, 0x04, 0xA40034, 8);  // read calibration 34kg

        queryState = 5;
        break;
      case 5: {
        log_d("Calibration data 34kg");
        uint8_t *mem = data + 7;
        calibration[8] = mem[0] * 256 + mem[1];   // Top Right 34kg
        calibration[9] = mem[2] * 256 + mem[3];   // Bottom Right 34kg
        calibration[10] = mem[4] * 256 + mem[5];  // Top Left 34kg
        calibration[11] = mem[6] * 256 + mem[7];  // Bottom Left 34kg
      }
        read_memory(handle, 0x04, 0xA40060, 2);  // read calibration reference temperature

        queryState = 6;
        break;
      case 6:
        uint8_t *mem = data + 7;
        log_d("Calibration data reference temperature");
        referenceTemperature = mem[0];
        set_reporting_mode(handle, 0x34, false);
        queryState = 0;
        break;
    }
  }

  bool onData(BalanceBoardData *out, uint16_t handle, uint8_t *data, size_t len) {
    if (data[0] == 0xA1) {
      // A non-zero reference temperature means we have calibrated
      if (data[1] == 0x34 && referenceTemperature != 0) {
        uint8_t *mem = data + 4;

        uint16_t values[4] = {
            static_cast<uint16_t>(mem[0] * 256 + mem[1]),  // tr
            static_cast<uint16_t>(mem[2] * 256 + mem[3]),  // br
            static_cast<uint16_t>(mem[4] * 256 + mem[5]),  // tl
            static_cast<uint16_t>(mem[6] * 256 + mem[7]),  // bl
        };
        out->handle = handle;
        out->tr = interpolate(0, values);
        out->br = interpolate(1, values);
        out->tl = interpolate(2, values);
        out->bl = interpolate(3, values);
        out->temperature = mem[8];
        out->batteryLevel = mem[10];
        out->referenceTemperature = referenceTemperature;
        return true;
      } else {
        readCalibrationData(data, len);
      }
    }
    return false;
  }
};

Wii::Wii(Bluetooth *bt) : bluetooth(bt) {
  bt->onHCIConnectionRequest([](Bluetooth *, const HCIConnectionRequest &result) {
    log_i("Received connection request from %s", formatHex((uint8_t *) &result.bdaddr, 6));
    return result.classOfDevice == 0x042500;  // Return true if wiimote
  });

  bt->onHCIEvent([this](Bluetooth *bt, const HCIEvent &event) {
    std::visit(overloaded{
                   [this](const HCIInquiryStarted &) { this->eventListener(ScanStarted{}); },
                   [this](const HCIInquiryComplete &) { this->eventListener(ScanStopped{}); },
                   [bt](const HCIInquiryResult &result) {
                     if (result.classOfDevice == 0x042500) {
                       bt->requestRemoteName(result);
                     }
                   },
                   [bt](const HCIRemoteName &result) {
                     log_i("Found %s %s", result.remoteName.data(), formatHex((uint8_t *) &result.inquiry.bdaddr, 6));
                     if (result.remoteName == "Nintendo RVL-WBC-01") {
                       bt->connect(result.inquiry);
                     }
                   },
                   [bt](const HCIConnectionFailed &result) {
                     log_e("Failed to connect Wiimote %s", formatHex((uint8_t *) &result.bdaddr, 6));
                   },
                   [bt](const HCIConnectionEstablished &result) {
                     log_i("Wiimote connection %s, handle: %d", result.accepted ? "accepted" : "established",
                           result.handle);

                     if (result.accepted) {
                       // We accepted a connection from an authenticated Wiimote.
                       // Return early, as it will establish the L2 connections.
                       return;
                     }

                     // Send auth request if pairing
                     log_i("Initiating auth");
                     bt->auth(result.handle);

                     // Establish L2CAP connections
                     // PSM: HID_Control=0x0011, HID_Interrupt=0x0013
                     // MTU: 672
                     bt->l2cap_connect(result.handle, 0x0011, 0x40);
                     bt->l2cap_connect(result.handle, 0x0013, 0x40);
                   },
                   [bt](const HCILinkKeyRequest &result) {
                     log_i("Negative link reply");
                     bt->negativeReply(result.bdaddr);
                   },
                   [bt](const HCIPINRequest &result) {
                     uint8_t pin_data[6];
                     // The pin is the mac of the host controller reversed
                     auto mac = bt->macAddress();
                     for (size_t i = 0; i < 6; ++i) {
                       pin_data[i] = mac[5 - i];
                     }

                     log_i("Sending pin reply");
                     bt->sendPinReply(result.bdaddr, pin_data, 6);
                   },
                   [bt](const HCIDisconnected &result) { log_i("Disconnected %d", result.handle); },
               },
               event);
  });

  bt->onACLConnectionRequest([](Bluetooth *, const ACLConnectionRequest &req) {
    log_i("Received ACL connection request from %d, psm %02X", req.handle, req.psm);
    return (req.psm == 0x0011 || req.psm == 0x0013);
  });

  bt->onACLEvent([this](Bluetooth *bt, const ACLEvent &event) {
    std::visit(overloaded{
                   [this](const ACLConnectionFailed &) {},
                   [this](const ACLDisconnected &info) {
                     if (info.psm == 0x13) {
                       this->eventListener(BalanceBoardDisconnected{
                           .handle = info.handle,
                       });
                       connectedBoards.erase(info.handle);
                       bluetooth->disconnect(info.handle);
                     }
                   },
                   [this](const ACLConnectionEstablished &conn) {
                     if (conn.psm == 0x0013) {
                       connectedBoards.emplace(conn.handle, std::make_unique<BalanceBoard>(bluetooth, conn.handle));
                       connectedBoards[conn.handle]->setLeds(bluetooth, conn.handle, std::bitset<4>(0b0001));
                       this->eventListener(BalanceBoardConnected{
                           .handle = conn.handle,
                       });
                     }
                   },
                   [this](const ACLData &data) {
                     BalanceBoardData out;
                     if (connectedBoards.at(data.handle)->onData(&out, data.handle, data.data, data.len)) {
                       this->eventListener(out);
                     }
                   },
               },
               event);
  });
}

Wii::~Wii() {}

void Wii::sync(bool enable) { bluetooth->scan(enable); }

void Wii::step() { bluetooth->process(); }

void Wii::onEvent(std::function<void(const WiiEvent &)> eventListener) {
  this->eventListener = std::move(eventListener);
}

void Wii::disconnect(uint16_t handle, uint16_t psm) { bluetooth->l2cap_disconnect(handle, psm); }

}  // namespace esphome::wii_balance_board::detail
