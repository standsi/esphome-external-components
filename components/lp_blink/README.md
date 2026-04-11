# esphome component for using low power core in esp32c6 for gpio blink

This component offers a low power way of pulsing a gpio on the esp32c6 for both awake and sleep modes.  The lp core c program is compiled separately with the assembly byte stream embedded in `ulp_main_binary.h`.  The source for the lp core program along with a script to rebuild this file are in the repo:

**TBD**

Parameters for the program including the gpio pin (must be an RTC pin), the pulse duration and the pulse frequency (delay) are passed through RTC shared memory.  The fixed addresses for the parameter locations are embedded in `ulp_main_shared.h`.  Instructions for updating this file are also included in the repo given above.

## yaml template for esphome

```yaml
substitutions:
  lp_blink_pin: "4"

external_components:
  - source: github://standsi/esphome-external-components@main
    components: [ lp_blink ]
    refresh: 5min

esphome:
  name: esp32c6_ulp_blink

esp32:
  variant: esp32c6
  board: esp32-c6-devkitc-1
  framework:
    type: esp-idf
    sdkconfig_options:
      # NOTE: Ensure sdk configs are set for lp core, Without this option it may not compile
      CONFIG_ULP_COPROC_ENABLED: "y"
      CONFIG_ULP_COPROC_TYPE_LP_CORE: "y"
      CONFIG_ULP_COPROC_RESERVE_MEM: "4096"
      CONFIG_ULP_SHARED_MEM: "0x10"

logger:
  level: DEBUG

wifi:
  ssid: "your_ssid"
  password: "your_password"

lp_blink:
  id: lp_blink_component
  gpio_num: ${lp_blink_pin}
  init_state: last
  pulse_width_us: 30000
  wake_period_ms: 1000

number:
  - platform: template
    name: "LP Pulse Width (us)"
    id: lp_pulse_width_us
    min_value: 1000
    max_value: 100000
    step: 1000
    optimistic: true
    initial_value: 30000
    set_action:
      lambda: |-
        id(lp_blink_component).set_pulse_width_us((uint32_t) x);

  - platform: template
    name: "LP Wake Period (ms)"
    id: lp_wake_period_ms
    min_value: 100
    max_value: 5000
    step: 100
    optimistic: true
    initial_value: 1000
    set_action:
      lambda: |-
        id(lp_blink_component).set_wakeup_period_ms((uint32_t) x);

switch:
  - platform: template
    name: "LP Blink Enabled"
    lambda: |-
      return id(lp_blink_component).is_running();
    turn_on_action:
      lambda: |-
        id(lp_blink_component).start_blink();
    turn_off_action:
      lambda: |-
        id(lp_blink_component).stop_blink();
```