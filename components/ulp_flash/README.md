# example configuration:

```yaml
esp32:
  board: esp32dev
  framework:
    type: esp-idf
    sdkconfig_options:
      # NOTE: Without this option it will not compile
      CONFIG_ULP_COPROC_ENABLED: "y"
      CONFIG_ULP_COPROC_TYPE_FSM: "y"
      CONFIG_ULP_COPROC_RESERVE_MEM: "1024"

external_components:
  - source: github://standsi/esphome-external-components@main
    components: [ ulp_flash ]
    refresh: 5min

# pulse widths from narrow (8ms) to medium (16ms) to wide (24ms)
ulp_flash:
  pin: GPIO2
  interval: 1000ms
  pulse_width: narrow
  init_state: last
  id: flash1
  pin_invert: "no"

# Use deep sleep to prove that ulp runs even when in deep sleep
# The led will blink very faintly and quickly in sleep mode, watch closely to see it
deep_sleep:
  run_duration: 10s
  sleep_duration: 10s
```
