# Wii Balance Board component

## Sample Configuration

```
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
