#pragma once

#include <algorithm>
#include <memory>
#include <variant>
#include <span>
#include <functional>

namespace esphome::wii_balance_board::detail {

struct HCIInquiryResult {
  uint64_t bdaddr;
  uint8_t psrm;
  uint32_t classOfDevice;
  uint16_t clkOffset;
};

struct HCIInquiryStarted {};

struct HCIInquiryComplete {};

struct HCIConnectionEstablished {
  uint64_t bdaddr;
  uint16_t handle;
  bool accepted;
};

struct HCIConnectionFailed {
  uint64_t bdaddr;
  uint16_t handle;
  uint8_t reason;
  bool accepted;
};

struct HCIDisconnected {
  uint16_t handle;
  uint8_t reason;
};

struct HCIRemoteName {
  HCIInquiryResult inquiry;
  std::string_view remoteName;
};

struct HCIConnectionRequest {
  uint64_t bdaddr;
  uint32_t classOfDevice;
};

struct HCILinkKeyRequest {
  uint64_t bdaddr;
  uint8_t keyType;
  uint8_t *linkKeyData;
  size_t size;
};

struct HCIPINRequest {
  uint64_t bdaddr;
};

struct ACLConnectionRequest {
  uint16_t handle;
  uint16_t sourceCid;
  uint16_t psm;
};

struct ACLConnectionFailed {
  uint16_t handle;
  uint16_t sourceCid;
  uint16_t psm;
};

struct ACLDisconnected {
  uint16_t handle;
  uint16_t psm;
};

struct ACLConnectionEstablished {
  uint16_t handle;
  uint16_t sourceCid;
  uint16_t psm;
  bool accepted;
};

struct ACLData {
  uint16_t handle;
  uint16_t channelId;
  uint8_t *data;
  size_t len;
};

using HCIEvent = std::variant<HCIInquiryStarted, HCIInquiryComplete, HCIInquiryResult, HCIConnectionEstablished,
                              HCIConnectionFailed, HCIDisconnected, HCIRemoteName, HCILinkKeyRequest, HCIPINRequest>;
using ACLEvent = std::variant<ACLDisconnected, ACLConnectionFailed, ACLConnectionEstablished, ACLData>;

class Bluetooth {
  struct Impl;
  std::unique_ptr<Impl> m_impl;

 public:
  Bluetooth();
  ~Bluetooth();

  // Device
  std::span<uint8_t, 6> macAddress();

  void onReady(const std::function<void(Bluetooth *)> &);
  void process();

  // HCI
  void onHCIEvent(const std::function<void(Bluetooth *, const HCIEvent &)> &);
  void onHCIConnectionRequest(const std::function<bool(Bluetooth *, const HCIConnectionRequest &)> &listener);

  void scan(bool enable);
  void requestRemoteName(const HCIInquiryResult &result);
  void connect(const HCIInquiryResult &result);
  void auth(uint16_t handle);
  void negativeReply(uint64_t bdaddr);
  void disconnect(uint16_t handle);
  void sendPinReply(uint64_t bdaddr, uint8_t *pinData, size_t len);

  // ACL
  void onACLEvent(const std::function<void(Bluetooth *, const ACLEvent &)> &acl);
  void onACLConnectionRequest(const std::function<bool(Bluetooth *, const ACLConnectionRequest &)> &listener);
  void l2cap_connect(uint16_t handle, uint16_t psm, uint16_t mtu);
  void l2cap_disconnect(uint16_t handle, uint16_t psm);
  void l2send_data(uint16_t handle, uint16_t psm, uint8_t *data, size_t len);
};

}  // namespace esphome::wii_balance_board::detail
