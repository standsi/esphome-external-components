# riscv_blink ESPHome component

This component targets the ESP32-S2 and ESP32-S3 ULP RISC-V coprocessor and offers a low power way of pulsing a gpio for both awake and sleep modes.  The risc v  core c program is compiled separately with the assembly byte stream embedded in `ulp_artifacts_esp32s2.h` or `ulp_artifacts_esp32s3.h`.  The source for the risc v core program along with a script to rebuild these files are in the repo:

**https://github.com/standsi/ULPBlinkEspHomeDev**

Parameters for the program including the gpio pin (must be an RTC pin), the pulse duration, the pulse frequency (delay) and whether to invert the state of the pin are passed through RTC shared memory.  The fixed addresses for the parameter locations are embedded in the artifacts files listed above.  Instructions for updating this file are also included in the repo given above.

### Sample power usage on ESP32S2
* For a 17ms high-state pulse into high impedence the power consumption has been measured at an average of about 87.6 microamps in a 1 second period.
* With a standard 3mm red LED and a 150 ohm current limit resistor attached to the pin a 17ms pulse consumes about 207 microamps average over a 1 second interval (this was with the LED cathode grounded).  The LED current peaks at a little less than 5 milliamps which the S2 can easily handle either in a common anode or cathode wiring.  The 17ms pulse is bright enough to be seen across a large room.
* NOTE: the S2 is not considered one of the lower power versions of the ESP32 line, as can be seen from the measurements above.  However, the WIFI operation is fairly efficent compared to, for example, the C6, when communicating to the Home Assistant native API.  The S2 peaks between 70 and 130mA wheras the C6 peaks between 120 and 260mA.  So if your esphome project needs to communicate frequently with HA between sleep cycles the S2 may be just as efficient in power usage. 

## sample yaml template for esphome

```yaml

substitutions:
  riscv_blink_pin: "4"

external_components:
  - source:
      type: local
      path: my_components

esphome:
  name: esp32s2_riscv_blink
  #name: esp32s3_riscv_blink

esp32:
  variant: esp32s2
  #variant: esp32s3
  board: esp32-s2-saola-1
  # board: esp32-s3-devkitc-1
  framework:
    type: esp-idf
    sdkconfig_options:
      # NOTE: Ensure sdk configs are set for riscv core, Without this option it may not compile
      CONFIG_ULP_COPROC_ENABLED: "y"
      # CONFIG_ULP_COPROC_TYPE_FSM is not set
      CONFIG_ULP_COPROC_TYPE_RISCV: "y"
      CONFIG_ULP_COPROC_RESERVE_MEM: "4096"

logger:
  level: DEBUG

wifi:
  ssid: "your_ssid"
  password: "your_password"

riscv_blink:
  id: riscv_blink_component
  gpio_num: ${riscv_blink_pin}
  init_state: last
  pulse_width_us: 30000
  wake_period_ms: 1000

number:
  - platform: template
    name: "ULP Pulse Width (us)"
    id: ulp_pulse_width_us
    min_value: 1000
    max_value: 100000
    step: 1000
    optimistic: true
    initial_value: 30000
    set_action:
      lambda: |-
        id(riscv_blink_component).set_pulse_width_us((uint32_t) x);

  - platform: template
    name: "ULP Wake Period (ms)"
    id: ulp_wake_period_ms
    min_value: 100
    max_value: 5000
    step: 100
    optimistic: true
    initial_value: 1000
    set_action:
      lambda: |-
        id(riscv_blink_component).set_wakeup_period_ms((uint32_t) x);

# switch:
#   - platform: template
#     name: "ULP Blink Enabled"
#     lambda: |-
#       return id(riscv_blink_component).is_running();
#     turn_on_action:
#       lambda: |-
#         id(riscv_blink_component).start_blink();
#     turn_off_action:
#       lambda: |-
#         id(riscv_blink_component).stop_blink();

binary_sensor:
  - platform: template
    name: "ULP Blink Running"
    id: blink_on
    lambda: |-
      return id(riscv_blink_component).is_running();

button:
  - platform: template
    name: "Toggle blink"
    id: toggle_blink
    on_press:
      then:
      lambda: |-
          if (id(riscv_blink_component).is_running()) {
            id(riscv_blink_component).stop_blink();
          } else {
        id(riscv_blink_component).start_blink();
          }

deep_sleep:
  id: sleep1
  run_duration: 1min
  sleep_duration: 30s

interval:
  - interval: 30s
    startup_delay: 15s
    then:
      - if:
          condition:
            lambda: "return id(riscv_blink_component).is_running();"
          then:
            logger.log: "riscv blink is running"
          else:
            logger.log: "riscv blink is NOT running"

```

### Modifying the component using custom ESPHome development
See the [README-dev.md](README-dev.md) file for details.

## note on the ulp risc v code

The runtime code is in this component, but the generated ULP artifacts are  placeholders that are filled by a powershell utility. To finish the component:

1. Build the sibling riscv_blink ESP-IDF project for esp32s2.
2. Run `pwsh ./esphome/update_riscv_ulp_artifacts.ps1 -Target esp32s2`.
3. Build for esp32s3.
4. Run `pwsh ./esphome/update_riscv_ulp_artifacts.ps1 -Target esp32s3`.

The component will log an explicit runtime error until those target-specific artifacts are populated.

See the esp idf project for generating the risc v code and using the utility to update  the binary into this component.
