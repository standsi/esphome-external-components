# esphome-external-components

### This collection of modified and custom esphome components focus on low power functionalities using the ESP32 microcontrollers.  See the README file in each component directory for usage details.

* mlx90393 is a modified version of the standard magnetometer component with wake on change operations for low power measurement against a threshold during sleep.  Tested with ESP32C6, ESP32S2 and ESP32S3.
* ulp_flash is a low power pin pulse for esp32s2/s3 using fsm so **only supported through esphome 2025.12 DEPRECATED**
* lp_blink is a low power pin pulse for esp32c6 using lp core, supported on latest esphome.
* risv_blink is a low power pin pulse for esp32s2 and esp32s3 risc-v ULPs, supported on latest esphome.

