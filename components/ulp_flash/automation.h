#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "ulp_flash.h"

namespace esphome {
namespace ulp_flash {

template<typename... Ts> class OnAction : public Action<Ts...> {
 public:
  explicit OnAction(ULPFlash *parent) : parent_(parent) {}

  void play(const Ts &...x) override { this->parent_->make_call().set_command_on().perform(); }

 protected:
  ULPFlash *parent_;
};

template<typename... Ts> class OffAction : public Action<Ts...> {
 public:
  explicit OffAction(ULPFlash *parent) : parent_(parent) {}

  void play(const Ts &...x) override { this->parent_->make_call().set_command_off().perform(); }

 protected:
  ULPFlash *parent_;
};

}  // namespace ulp_flash
}  // namespace esphome
