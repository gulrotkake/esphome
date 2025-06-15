#pragma once
#include <cstdint>

#include "ring_buffer.h"

#define HCI_H4_CMD_PREAMBLE_SIZE (4)
#define HCI_H4_ACL_PREAMBLE_SIZE (5)

/*  HCI Command opcode group field(OGF) */
#define HCI_GRP_LINK_CONT_CMDS (0x01 << 10)          /* 0x0400 */
#define HCI_GRP_HOST_CONT_BASEBAND_CMDS (0x03 << 10) /* 0x0C00 */
#define HCI_GRP_INFO_PARAMS_CMDS (0x04 << 10)

// OGF + OCF
#define HCI_RESET (0x0003 | HCI_GRP_HOST_CONT_BASEBAND_CMDS)
#define HCI_READ_BD_ADDR (0x0009 | HCI_GRP_INFO_PARAMS_CMDS)
#define HCI_WRITE_LOCAL_NAME (0x0013 | HCI_GRP_HOST_CONT_BASEBAND_CMDS)
#define HCI_WRITE_CLASS_OF_DEVICE (0x0024 | HCI_GRP_HOST_CONT_BASEBAND_CMDS)
#define HCI_WRITE_SCAN_ENABLE (0x001A | HCI_GRP_HOST_CONT_BASEBAND_CMDS)
#define HCI_INQUIRY (0x0001 | HCI_GRP_LINK_CONT_CMDS)
#define HCI_INQUIRY_CANCEL (0x0002 | HCI_GRP_LINK_CONT_CMDS)
#define HCI_REMOTE_NAME_REQUEST (0x0019 | HCI_GRP_LINK_CONT_CMDS)
#define HCI_CREATE_CONNECTION (0x0005 | HCI_GRP_LINK_CONT_CMDS)
#define HCI_AUTHENTICATION (0x0011 | HCI_GRP_LINK_CONT_CMDS)
#define HCI_NEGATIVE_REPLY (0x000C | HCI_GRP_LINK_CONT_CMDS)
#define HCI_PIN_REPLY (0x000D | HCI_GRP_LINK_CONT_CMDS)
#define HCI_ACCEPT_CONNECTION (0x0009 | HCI_GRP_LINK_CONT_CMDS)
#define HCI_DISCONNECT (0x0006 | HCI_GRP_LINK_CONT_CMDS)

#define BD_ADDR_LEN (6)

#define UINT16_TO_STREAM(p, u16) \
  { \
    *(p)++ = (uint8_t) (u16); \
    *(p)++ = (uint8_t) ((u16) >> 8); \
  }
#define UINT8_TO_STREAM(p, u8) \
  { *(p)++ = (uint8_t) (u8); }
#define ARRAY_TO_STREAM(p, a, len) \
  { \
    int ijk; \
    for (ijk = 0; ijk < len; ijk++) \
      *(p)++ = (uint8_t) a[ijk]; \
  }

#define U64_ADDR_TO_STREAM(buf, addr) \
  for (size_t i = 0; i < BD_ADDR_LEN; ++i) { \
    UINT8_TO_STREAM(buf, (addr >> i * 8) & 0xFF); \
  }

namespace esphome::wii_balance_board::detail {

enum { H4_TYPE_COMMAND = 1, H4_TYPE_ACL = 2, H4_TYPE_SCO = 3, H4_TYPE_EVENT = 4 };

static bool enqueue_cmd_reset(RingBuffer &buffer) {
  if (auto out = buffer.allocate(HCI_H4_CMD_PREAMBLE_SIZE)) {
    uint8_t *buf = out.data();
    UINT8_TO_STREAM(buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM(buf, HCI_RESET);
    UINT8_TO_STREAM(buf, 0);
    return true;
  }
  return false;
}

static bool enqueue_cmd_read_bd_addr(RingBuffer &buffer) {
  if (auto out = buffer.allocate(HCI_H4_CMD_PREAMBLE_SIZE)) {
    uint8_t *buf = out.data();
    UINT8_TO_STREAM(buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM(buf, HCI_READ_BD_ADDR);
    UINT8_TO_STREAM(buf, 0);
    return true;
  }
  return false;
}

static bool enqueue_cmd_write_local_name(RingBuffer &buffer, uint8_t *name, uint8_t len) {
  if (auto out = buffer.allocate(HCI_H4_CMD_PREAMBLE_SIZE + 248)) {
    uint8_t *buf = out.data();
    // name ends with null. TODO check len<=248
    UINT8_TO_STREAM(buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM(buf, HCI_WRITE_LOCAL_NAME);
    UINT8_TO_STREAM(buf, 248);
    ARRAY_TO_STREAM(buf, name, len);
    for (uint8_t i = len; i < 248; i++) {
      UINT8_TO_STREAM(buf, 0);
    }
    return true;
  }
  return false;
}

static bool enqueue_cmd_write_class_of_device(RingBuffer &buffer, uint8_t *cod) {
  if (auto out = buffer.allocate(HCI_H4_CMD_PREAMBLE_SIZE + 3)) {
    uint8_t *buf = out.data();

    UINT8_TO_STREAM(buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM(buf, HCI_WRITE_CLASS_OF_DEVICE);
    UINT8_TO_STREAM(buf, 3);
    for (uint8_t i = 0; i < 3; i++) {
      UINT8_TO_STREAM(buf, cod[i]);
    }
    return true;
  }
  return false;
}

static bool enqueue_cmd_write_scan_enable(RingBuffer &buffer, uint8_t mode) {
  if (auto out = buffer.allocate(HCI_H4_CMD_PREAMBLE_SIZE + 1)) {
    uint8_t *buf = out.data();

    UINT8_TO_STREAM(buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM(buf, HCI_WRITE_SCAN_ENABLE);
    UINT8_TO_STREAM(buf, 1);

    UINT8_TO_STREAM(buf, mode);
    return true;
  }
  return false;
}

static bool enqueue_cmd_inquiry(RingBuffer &buffer, uint32_t lap, uint8_t len, uint8_t num) {
  if (auto out = buffer.allocate(HCI_H4_CMD_PREAMBLE_SIZE + 5)) {
    uint8_t *buf = out.data();

    UINT8_TO_STREAM(buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM(buf, HCI_INQUIRY);
    UINT8_TO_STREAM(buf, 5);

    UINT8_TO_STREAM(buf, (lap & 0xFF));          // lap 0x33 <- 0x9E8B33
    UINT8_TO_STREAM(buf, ((lap >> 8) & 0xFF));   // lap 0x8B
    UINT8_TO_STREAM(buf, ((lap >> 16) & 0xFF));  // lap 0x9E
    UINT8_TO_STREAM(buf, len);
    UINT8_TO_STREAM(buf, num);
    return true;
  }
  return false;
}

static bool enqueue_cmd_inquiry_cancel(RingBuffer &buffer) {
  if (auto out = buffer.allocate(HCI_H4_CMD_PREAMBLE_SIZE)) {
    uint8_t *buf = out.data();

    UINT8_TO_STREAM(buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM(buf, HCI_INQUIRY_CANCEL);
    UINT8_TO_STREAM(buf, 0);
    return true;
  }
  return false;
}

static bool enqueue_cmd_remote_name_request(RingBuffer &buffer, uint64_t bdaddr, uint8_t psrm, uint16_t clkofs) {
  if (auto out = buffer.allocate(HCI_H4_CMD_PREAMBLE_SIZE + 10)) {
    uint8_t *buf = out.data();

    UINT8_TO_STREAM(buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM(buf, HCI_REMOTE_NAME_REQUEST);
    UINT8_TO_STREAM(buf, 6 + 1 + 1 + 2);

    U64_ADDR_TO_STREAM(buf, bdaddr);

    UINT8_TO_STREAM(buf, psrm);     // Page_Scan_Repetition_Mode
    UINT8_TO_STREAM(buf, 0);        // Reserved
    UINT16_TO_STREAM(buf, clkofs);  // Clock_Offset
    return true;
  }
  return false;
}

static bool enqueue_cmd_create_connection(RingBuffer &buffer, uint64_t bd_addr, uint16_t pt, uint8_t psrm,
                                          uint16_t clkofs, uint8_t ars) {
  if (auto out = buffer.allocate(HCI_H4_CMD_PREAMBLE_SIZE + 13)) {
    uint8_t *buf = out.data();

    UINT8_TO_STREAM(buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM(buf, HCI_CREATE_CONNECTION);
    UINT8_TO_STREAM(buf, 6 + 2 + 1 + 1 + 2 + 1);

    U64_ADDR_TO_STREAM(buf, bd_addr);
    UINT16_TO_STREAM(buf, pt);      // Packet_Type
    UINT8_TO_STREAM(buf, psrm);     // Page_Scan_Repetition_Mode
    UINT8_TO_STREAM(buf, 0);        // Reserved
    UINT16_TO_STREAM(buf, clkofs);  // Clock_Offset
    UINT8_TO_STREAM(buf, ars);      // Allow_Role_Switch
    return true;
  }
  return false;
}

static bool enqueue_cmd_auth_request(RingBuffer &buffer, uint16_t connection_handle) {
  if (auto out = buffer.allocate(HCI_H4_CMD_PREAMBLE_SIZE + 2)) {
    uint8_t *buf = out.data();

    UINT8_TO_STREAM(buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM(buf, HCI_AUTHENTICATION);
    UINT8_TO_STREAM(buf, 2);

    UINT8_TO_STREAM(buf, connection_handle & 0xFF);
    UINT8_TO_STREAM(buf, (connection_handle >> 8) & 0x0F);
    return true;
  }
  return false;
}

static bool enqueue_cmd_negative_reply(RingBuffer &buffer, uint64_t bdaddr) {
  if (auto out = buffer.allocate(HCI_H4_CMD_PREAMBLE_SIZE + 6)) {
    uint8_t *buf = out.data();
    UINT8_TO_STREAM(buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM(buf, HCI_NEGATIVE_REPLY);
    UINT8_TO_STREAM(buf, 6);

    U64_ADDR_TO_STREAM(buf, bdaddr);
    return true;
  }
  return false;
}

static bool enqueue_cmd_pin_reply(RingBuffer &buffer, uint64_t bdaddr, uint8_t *pin, size_t len) {
  if (auto out = buffer.allocate(HCI_H4_CMD_PREAMBLE_SIZE + 23)) {
    uint8_t *buf = out.data();

    UINT8_TO_STREAM(buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM(buf, HCI_PIN_REPLY);
    UINT8_TO_STREAM(buf, 6 + 1 + 16);  // 23

    U64_ADDR_TO_STREAM(buf, bdaddr);
    UINT8_TO_STREAM(buf, len);  // Pin length

    for (uint8_t i = 0; i < len; i++) {
      UINT8_TO_STREAM(buf, pin[i]);
    }

    for (uint8_t i = 0; i < 16 - len; i++) {
      UINT8_TO_STREAM(buf, 0);
    }
    return true;
  }
  return false;
}

static bool enqueue_cmd_accept_connection(RingBuffer &buffer, uint64_t bd_addr) {
  if (auto out = buffer.allocate(HCI_H4_CMD_PREAMBLE_SIZE + 7)) {
    uint8_t *buf = out.data();

    UINT8_TO_STREAM(buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM(buf, HCI_ACCEPT_CONNECTION);
    UINT8_TO_STREAM(buf, 6 + 1);
    U64_ADDR_TO_STREAM(buf, bd_addr);
    UINT8_TO_STREAM(buf, 0);
    return true;
  }
  return false;
}

static bool enqueue_cmd_reject_connection(RingBuffer &buffer, uint64_t bd_addr, uint8_t rejReason) {
  if (auto out = buffer.allocate(HCI_H4_CMD_PREAMBLE_SIZE + 7)) {
    uint8_t *buf = out.data();

    UINT8_TO_STREAM(buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM(buf, HCI_ACCEPT_CONNECTION);
    UINT8_TO_STREAM(buf, 6 + 1);
    U64_ADDR_TO_STREAM(buf, bd_addr);
    UINT8_TO_STREAM(buf, rejReason);
    return true;
  }
  return false;
}

static bool enqueue_cmd_disconnect(RingBuffer &buffer, uint16_t connection_handle) {
  if (auto out = buffer.allocate(HCI_H4_CMD_PREAMBLE_SIZE + 3)) {
    uint8_t *buf = out.data();
    UINT8_TO_STREAM(buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM(buf, HCI_DISCONNECT);
    UINT8_TO_STREAM(buf, 3);

    UINT16_TO_STREAM(buf, connection_handle);
    UINT8_TO_STREAM(buf, 0x15);
    return true;
  }
  return false;
}

// TODO long data is split to multi packets
static void make_l2cap_single_packet(uint8_t *buf, uint16_t channel_id, uint8_t *data, uint16_t len) {
  UINT16_TO_STREAM(buf, len);
  UINT16_TO_STREAM(buf, channel_id);  // 0x0001=Signaling channel
  ARRAY_TO_STREAM(buf, data, len);
}

static bool enqueue_acl_l2cap_single_packet(RingBuffer &buffer, uint16_t connection_handle,
                                            uint8_t packet_boundary_flag, uint8_t broadcast_flag, uint16_t channel_id,
                                            uint8_t *data, uint8_t len) {
  if (auto out = buffer.allocate(HCI_H4_ACL_PREAMBLE_SIZE + len + 4)) {
    uint8_t *buf = out.data();
    uint8_t *l2cap_buf = buf + HCI_H4_ACL_PREAMBLE_SIZE;
    make_l2cap_single_packet(l2cap_buf, channel_id, data, len);

    UINT8_TO_STREAM(buf, H4_TYPE_ACL);
    UINT8_TO_STREAM(buf, connection_handle & 0xFF);
    UINT8_TO_STREAM(buf, ((connection_handle >> 8) & 0x0F) | packet_boundary_flag << 4 | broadcast_flag << 6);
    UINT16_TO_STREAM(buf, len + 4);
    return true;
  }
  return false;
}

}  // namespace esphome::wii_balance_board::detail
