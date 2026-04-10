#pragma once

#include <cstdint>

// These RTC_SLOW_MEM addresses are synchronized to the current LP binary build.
// Source of truth: lp_blink/build/esp-idf/ulp_blink/ulp_main/ulp_main.ld
namespace esphome {
namespace lp_blink {
namespace ulp_main_shared {

constexpr uintptr_t kRunCountAddr = 0x5000051cu;
constexpr uintptr_t kFlashLpIoInverted = 0x50000520u;
constexpr uintptr_t kPulseWidthUsAddr = 0x50000524u;
constexpr uintptr_t kFlashLpIoAddr = 0x50000528u;

inline volatile uint32_t &run_count() { return *reinterpret_cast<volatile uint32_t *>(kRunCountAddr); }

inline volatile uint32_t &flash_lp_io_inverted() { return *reinterpret_cast<volatile uint32_t *>(kFlashLpIoInverted); }

inline volatile uint32_t &pulse_width_us() { return *reinterpret_cast<volatile uint32_t *>(kPulseWidthUsAddr); }

inline volatile uint32_t &flash_lp_io() { return *reinterpret_cast<volatile uint32_t *>(kFlashLpIoAddr); }

}  // namespace ulp_main_shared
}  // namespace lp_blink
}  // namespace esphome
