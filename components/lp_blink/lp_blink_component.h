#pragma once

#include <cstdint>

#include "esphome/core/component.h"

namespace esphome {
namespace lp_blink {

enum class InitState : uint8_t {
  RUNNING = 0,
  STOPPED = 1,
  LAST = 2,
};

class LPBlinkComponent : public Component {
 public:
  explicit LPBlinkComponent(uint8_t gpio_num) : gpio_num_(gpio_num) {}

  void setup() override;
  void loop() override {}
  float get_setup_priority() const override { return setup_priority::DATA; }

  void start_blink();
  void stop_blink();
  bool is_running() const { return this->running_; }
  void set_init_state(InitState init_state) { this->init_state_ = init_state; }
  void set_pulse_width_us(uint32_t pulse_width_us);
  void set_wakeup_period_ms(uint32_t wakeup_period_ms);
  void set_flash_lp_io_inverted(bool inverted);

 protected:
  bool should_start_on_boot_() const;
  void save_last_state_(bool running);
  void drive_pin_low_();

  bool start_lp_core_();

  uint8_t gpio_num_;
  uint32_t pulse_width_us_{30000};
  uint32_t wakeup_period_us_{1000000};
  bool flash_lp_io_inverted_{false};
  InitState init_state_{InitState::RUNNING};
  bool running_{false};
};

}  // namespace lp_blink
}  // namespace esphome
