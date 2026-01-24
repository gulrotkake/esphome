# Wii Balance Board ESPHome component

Use a Wii Balance Board as a smart scale in Home Assistant.

<img width="329" height="583" alt="image" src="https://github.com/user-attachments/assets/92ac038c-ad78-400b-9394-d109598193c5" />

More in depth documentation available [here](https://tightloop.io/homeassistant+balanceboard/index.html).

## Requirements

1. A balance board
2. A home assistant setup
3. An ESP32 device with support for BR/EDR.

## Sample Configuration

```
esp32:
  variant: esp32
  board: esp32dev # Replace with your esp32 board
  framework:
    type: arduino
    sdkconfig_options:
      CONFIG_BT_ENABLED: y
      CONFIG_BT_CLASSIC_ENABLED: y

external_components:
  - source:
      type: git
      url: https://github.com/gulrotkake/esphome
      ref: balance-board
    components: [ wii_balance_board ]

wii_balance_board:
    id: board
    standard_deviation: 0.3

button:
  - platform: template
    name: "Start Sync"
    on_press:
      then:
        - lambda: 'id(board)->sync(true);'
```

## Contributions

- The original wiimote code (wiimote_bt.h, Wiimote.h and Wiimot.cpp) was from https://github.com/takeru/Wiimote, and was extended in https://github.com/gulrotkake/esp32_wiimote .
