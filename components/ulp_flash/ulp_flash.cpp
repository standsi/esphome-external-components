#include "esphome/core/log.h"
#include "ulp_flash.h"
#include "driver/rtc_io.h"
#include "soc/rtc_io_reg.h"
#include "esphome.h"
#include "esphome/core/gpio.h"
#include "esp_sleep.h"

#if defined(SOC_ULP_FSM_SUPPORTED)
// ESP32, ESP32-S2, ESP32-S3 with ULP-FSM enabled
#if __has_include("esp32/ulp.h")
#include "esp32/ulp.h"
#elif __has_include("ulp/ulp.h")
#include "ulp/ulp.h"
#else
#error "ULP-FSM header not found for this platform"
#endif
// ...existing ULP-FSM code...
#elif defined(SOC_ULP_RISCV_SUPPORTED)
// ESP32-S2/S3 ULP-RISC-V (different API)
#include "ulp_riscv/ulp_riscv.h"
// ...ULP-RISC-V code...
#else
#error "ULP not supported on this ESP32 variant"
#endif

namespace esphome {
namespace ulp_flash {

static const char *TAG = "ulp_flash.component";

// states of flasher
const bool FLASH_OFF = false;
const bool FLASH_ON = true;

const LogString *flash_command_to_str(bool state) {
  if (state == FLASH_ON) {
    return LOG_STR("ON");
  } else if (state == FLASH_OFF) {
    return LOG_STR("OFF");
  } else {
    return LOG_STR("UNKNOWN");
  }
}

ULPFlash::ULPFlash() : flash_state{FLASH_OFF} {
  ESP_LOGD(TAG, "ctor initial flash_state: %s", LOG_STR_ARG(flash_command_to_str(this->flash_state)));
}

ULPFlashCall::ULPFlashCall(ULPFlash *parent) : parent_(parent), flash_state_(parent->flash_state) {}
ULPFlashCall &ULPFlashCall::set_command(const char *command) {
  if (strcmp(command, "on") == 0) {
    return this->set_command_on();
  } else if (strcmp(command, "off") == 0) {
    return this->set_command_off();
  } else {
    ESP_LOGE(TAG, "Invalid command string: %s", command);
  }
  return *this;
}

ULPFlashCall &ULPFlashCall::set_command_on() {
  // this->parent_->flash_state = FLASH_ON;
  this->flash_state_ = FLASH_ON;
  return *this;
}
ULPFlashCall &ULPFlashCall::set_command_off() {
  // this->parent_->flash_state = FLASH_OFF;
  this->flash_state_ = FLASH_OFF;
  return *this;
}

ULPFlashCall &ULPFlashCall::set_flash_state(bool state) {
  this->flash_state_ = state;
  return *this;
}

ULPFlashCall ULPFlash::make_call() { return {this}; }

void ULPFlash::disconnect_led_pin() {
  if (this->pin_ != nullptr) {
    gpio_num_t gpio = (gpio_num_t) pin_->get_pin();
    rtc_gpio_deinit(gpio);
    ESP_LOGD(TAG, "ULPFlash LED pin disconnected");
  }
}

void ULPFlash::reconnect_led_pin() {
  if (this->pin_ != nullptr) {
    gpio_num_t gpio = (gpio_num_t) pin_->get_pin();
    rtc_gpio_deinit(gpio);
    rtc_gpio_init(gpio);
    rtc_gpio_set_direction(gpio, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(gpio, 0);
    ESP_LOGD(TAG, "ULPFlash LED pin reconnected");
  }
}

void ULPFlashCall::perform() {
  this->validate_();
  // Execute the flash command by updating the ULP state
  this->parent_->flash_state = this->flash_state_;
  ESP_LOGD(TAG, "ULPFlash command set to %s", LOG_STR_ARG(flash_command_to_str(this->flash_state_)));
  if (this->flash_state_ == FLASH_ON) {
    // reconnect the led pin in case it was disconnected
    this->parent_->reconnect_led_pin();
    // Start ULP timer
    ulp_timer_resume();
  } else {
    // disconnect the led pin to allow other components to use it
    this->parent_->disconnect_led_pin();
    // Stop ULP timer
    ulp_timer_stop();
  }
  // Publish the new state and save it
  this->parent_->publish_state(true);
}
void ULPFlashCall::validate_() {
  if (this->flash_state_ != FLASH_ON && this->flash_state_ != FLASH_OFF) {
    ESP_LOGE(TAG, "Invalid flash state command");
  }
}

void ULPFlash::add_on_state_callback(std::function<void()> &&f) { this->state_callback_.add(std::move(f)); }

void ULPFlash::publish_state(bool save) {
  ESP_LOGD(TAG, "Publishing ULPFlash state: %s", LOG_STR_ARG(flash_command_to_str(this->flash_state)));
  this->state_callback_.call();
  if (save) {
    ULPFlashRestoreState restore{};
    memset(&restore, 0, sizeof(restore));
    restore.flash_state = this->flash_state;
    this->rtc_.save(&restore);
  }
}

optional<ULPFlashRestoreState> ULPFlash::restore_state_() {
  this->rtc_ = global_preferences->make_preference<ULPFlashRestoreState>(this->get_preference_hash());
  ULPFlashRestoreState recovered{};
  if (!this->rtc_.load(&recovered))
    return {};
  return recovered;
}

ULPFlashCall ULPFlashRestoreState::to_call(ULPFlash *ulpflash) {
  auto call = ulpflash->make_call();
  call.set_flash_state(this->flash_state);
  return call;
}
void ULPFlashRestoreState::apply(ULPFlash *ulpflash) {
  ESP_LOGD(TAG, "restoring flash_state: %s", LOG_STR_ARG(flash_command_to_str(this->flash_state)));
  ulpflash->flash_state = this->flash_state;
  ulpflash->publish_state();
}

void ULP_FLASH_RUN(uint32_t us, uint32_t bit, FlashPulseWidth pulse_width_, FlashPinInvert pin_invert_);

void ULPFlash::setup() {
  // Stop any previously running ULP program
  // ulp_timer_stop();
  // delay(3000);

  esp_log_level_set(TAG, ESP_LOG_DEBUG);
  ESP_LOGD(TAG, "flash state during setup: %s", LOG_STR_ARG(flash_command_to_str(this->flash_state)));

  // Convert GPIO pin to RTC IO index
  gpio_num_t gpio = (gpio_num_t) pin_->get_pin();
  int rtc_num = rtc_io_number_get(gpio);
  if (rtc_num < 0) {
    mark_failed(LOG_STR("Specified pin is not RTC-capable"));
    return;
  }
  rtc_bit_ = RTC_GPIO_OUT_DATA_S + rtc_num;

  // Init LED pin
  rtc_gpio_init(gpio);
  rtc_gpio_set_direction(gpio, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_level(gpio, 0);
  // keep the rtc io live during deep sleep
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);

  // Run ULP
  int delay_us = interval_ * 1000;  // interval_ is in ms
  ULP_FLASH_RUN(delay_us, rtc_bit_, pulse_width_, pin_invert_);
  // decide if init to on or off, or restore from last published to rtc
  if (this->init_state_ == FlashInitState::FLASH_INIT_ON) {
    this->flash_state = FLASH_ON;
    ESP_LOGD(TAG, "Initial flash state set to ON");
  } else if (this->init_state_ == FlashInitState::FLASH_INIT_OFF) {
    this->flash_state = FLASH_OFF;
    ESP_LOGD(TAG, "Initial flash state set to OFF");
  } else {
    // Restore previous state if any
    auto restored = this->restore_state_();
    if (restored.has_value()) {
      restored->apply(this);
    } else {
      ESP_LOGD(TAG, "No previous state to restore");
    }
    ESP_LOGD(TAG, "Initial flash state set to LAST (restore from RTC)");
  }
  // if flash off turn off ulp timer
  if (this->flash_state == FLASH_OFF) {
    ESP_LOGD(TAG, "Initial flash state is OFF, stopping ULP timer");
    ulp_timer_stop();
  } else {
    ESP_LOGD(TAG, "Initial flash state is ON, starting ULP timer");
    // Start ULP timer
    ulp_timer_resume();
  }
}

void ULPFlash::loop() {
  // Nothing here — ULP runs independently
}

void ULP_FLASH_RUN(uint32_t us, uint32_t bit, FlashPulseWidth pulse_width_, FlashPinInvert pin_invert_) {
  RTC_SLOW_MEM[12] = 0;
  // this is the default narrow pulse width
  ulp_insn_t ulp_flash[] = {
      // I_MOVI(R3, 12),                         // #12 -> R3
      // I_LD(R0, R3, 0),                        // R0 = RTC_SLOW_MEM[R3(#12)]
      // M_BL(1, 1),                             // GOTO M_LABEL(1) IF R0 < 1
      I_WR_REG(RTC_GPIO_OUT_REG, bit, bit,
               static_cast<uint32_t>(pin_invert_ == FlashPinInvert::FLASH_PIN_INVERT_ON ? 0u : 1u)),  // set pin on
      // I_SUBI(R0, R0, 1),                      // R0 = R0 - 1, R0 = 1, R0 = 0
      // I_ST(R0, R3, 0),                        // RTC_SLOW_MEM[R3(#12)] = R0
      // M_BX(2),                                // GOTO M_LABEL(2)
      I_DELAY(65000),
      // M_LABEL(1),                             // M_LABEL(1)
      I_WR_REG(RTC_GPIO_OUT_REG, bit, bit,
               static_cast<uint32_t>(pin_invert_ == FlashPinInvert::FLASH_PIN_INVERT_ON ? 1u : 0u)),  // set pin off
      //    I_ADDI(R0, R0, 1),                    // R0 = R0 + 1, R0 = 0, R0 = 1
      //    I_ST(R0, R3, 0),                      // RTC_SLOW_MEM[R3(#12)] = R0
      // M_LABEL(2),                             // M_LABEL(2)
      I_HALT()  // HALT COPROCESSOR
  };
  // modify delay for medium pulse width (~16ms)
  ulp_insn_t ulp_flash_medium[] = {
      I_WR_REG(RTC_GPIO_OUT_REG, bit, bit,
               static_cast<uint32_t>(pin_invert_ == FlashPinInvert::FLASH_PIN_INVERT_ON ? 0u : 1u)),  // set pin on
      I_DELAY(65000), I_DELAY(65000),
      I_WR_REG(RTC_GPIO_OUT_REG, bit, bit,
               static_cast<uint32_t>(pin_invert_ == FlashPinInvert::FLASH_PIN_INVERT_ON ? 1u : 0u)),  // set pin off
      I_HALT()  // HALT COPROCESSOR
  };
  // modify delay for wide pulse width (~24ms)
  ulp_insn_t ulp_flash_wide[] = {
      I_WR_REG(RTC_GPIO_OUT_REG, bit, bit,
               static_cast<uint32_t>(pin_invert_ == FlashPinInvert::FLASH_PIN_INVERT_ON ? 0u : 1u)),  // set pin on
      I_DELAY(65000),
      I_DELAY(65000),
      I_DELAY(65000),
      I_WR_REG(RTC_GPIO_OUT_REG, bit, bit,
               static_cast<uint32_t>(pin_invert_ == FlashPinInvert::FLASH_PIN_INVERT_ON ? 1u : 0u)),  // set pin off
      I_HALT()  // HALT COPROCESSOR
  };

  // Microseconds to delay between halt and wake states
  ulp_set_wakeup_period(0, us);

  // Load and start proper ULP program
  if (pulse_width_ == FlashPulseWidth::MEDIUM) {
    size_t size = sizeof(ulp_flash_medium) / sizeof(ulp_insn_t);
    ulp_process_macros_and_load(0, ulp_flash_medium, &size);
  } else if (pulse_width_ == FlashPulseWidth::WIDE) {
    size_t size = sizeof(ulp_flash_wide) / sizeof(ulp_insn_t);
    ulp_process_macros_and_load(0, ulp_flash_wide, &size);
  } else {
    size_t size = sizeof(ulp_flash) / sizeof(ulp_insn_t);
    ulp_process_macros_and_load(0, ulp_flash, &size);
  }
  // now run it!!
  ulp_run(0);
}

void ULPFlash::dump_config() {
  if (rtc_bit_ < 0) {
    ESP_LOGE(TAG, "Pin %d is not RTC-capable", pin_->get_pin());
    return;
  }
  ESP_LOGCONFIG(TAG, "ULPFlash-Stan:");
  ESP_LOGCONFIG(TAG, "  Interval: %u ms", interval_);
  LOG_PIN("  Pin: ", pin_);
  ESP_LOGCONFIG(TAG, "  RTC Bit is %d", rtc_bit_);
  ESP_LOGCONFIG(TAG, "  Pulse Width: %d", static_cast<uint8_t>(pulse_width_));
  ESP_LOGCONFIG(TAG, "  Initial State: %d", static_cast<uint8_t>(init_state_));
  ESP_LOGCONFIG(TAG, "  Pin Invert: %d", static_cast<uint8_t>(pin_invert_));
  ESP_LOGCONFIG(TAG, "  Current Flash State: %s", LOG_STR_ARG(flash_command_to_str(this->flash_state)));
}

}  // namespace ulp_flash
}  // namespace esphome
