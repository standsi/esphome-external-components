# MLX90393 Triple-axis Magnetometer with Wake On Change operation

### This custom component extends the [base ESPHome component](https://esphome.io/components/sensor/mlx90393/) with ability to configure the Wake On Change function in the chip.  WOC is enabled through a custom action on the sensor including specifying the threshold which allows for dynamic settings based on conditions (such as a calibration path).  The WOC mode enabled during deep sleep saves substantial power drain vs full I2C operation during a polling wake cycle.  For example on an ESP32C6 in deep sleep with WOC checking about 2 times per second the average power consumption over 1 second is less than 50 microamps.  Tested with ESP32C6, ESP32S2 and ESP32S3.

### Wiring: connect the I2C and power connections per your board instructions.  Connect the Trigger output to a digitial input GPIO that is in the RTC domain so it stays active during sleep.  NOTE that the standard MLX trigger pin is a push-pull active high.

A sample ESPHome yaml configuration using this repo is:
```yaml
esphome:
  name: testMlx
  friendly_name: testMlx

external_components:
  - source: github://standsi/esphome-external-components@main
    components: [ mlx90393 ]
    refresh: 5min

esp32:
  variant: esp32c6
  framework:
    type: esp-idf

# Enable logging
logger:
  level: DEBUG

# Enable Home Assistant API
api:
  encryption:
    key: "<your api key>"

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

captive_portal:

i2c:
  scl: <SCL gpio>
  sda: <SDA gpio>
  scan: True
  id: bus_conn

sensor:
  - platform: mlx90393
    id: mlx
    x_axis:
      name: "x"
    y_axis:
      name: "y"
    z_axis:
      name: "z"
    address: <specify if not default 0x18>
    update_interval: 2s

deep_sleep:
  id: sleep1
  wakeup_pin:
    number: <GPIO wired to Trigger pin on MLX>
    mode:
      input: true

# for demo only, may want to have other way of starting sleep
interval:
  - interval: 20s
    startup_delay: 5s
    then:
      - mlx90393.enable_woc:
          id: mlx
          woxy_threshold: 500
      - deep_sleep.enter:
          id: sleep1
          sleep_duration: 25min

```

### Modifying the component using custom ESPHome development
If you want to use this component as the basis for further custom development, follow the instructions for [setting up a ESPHome environment](https://developers.esphome.io/contributing/development-environment/).  Create a directory called `config` in the root of the repo, then create a directory `external_components` in the `config` directory.  Copy the `components\mlx90393` folder from this repo to a `mlx90393` directory in the `external_components` directory.  Now create a test yaml file similar to the one above, replacing the external_components section with:
```yaml
external_components:
  - source:
      type: local
      path: external_components
```
To test the code with the esphome runtime use a command line like (making sure the python virtual environment is set, see the instructions):

```
esphome run ./config/test_mlx.yaml --device <serial port name>
```
