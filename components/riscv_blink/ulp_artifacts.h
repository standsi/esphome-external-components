#pragma once

#include <cstddef>
#include <cstdint>

#include "ulp_artifacts_esp32s2.h"
#include "ulp_artifacts_esp32s3.h"

namespace esphome {
namespace riscv_blink {

struct ULPMainArtifact {
  const uint8_t *binary;
  size_t binary_size;
  uintptr_t run_count_addr;
  uintptr_t flash_lp_io_inverted_addr;
  uintptr_t pulse_width_us_addr;
  uintptr_t flash_lp_io_addr;
  const char *target_name;
  bool valid;
};

inline const ULPMainArtifact &get_ulp_main_artifact() {
#if defined(USE_ESP32_VARIANT_ESP32S2)
  static const ULPMainArtifact artifact = {
      .binary = kUlpMainBinaryEsp32S2.data(),
      .binary_size = kUlpMainBinaryEsp32S2.size(),
      .run_count_addr = kUlpMainRunCountAddrEsp32S2,
      .flash_lp_io_inverted_addr = kUlpMainFlashLpIoInvertedAddrEsp32S2,
      .pulse_width_us_addr = kUlpMainPulseWidthUsAddrEsp32S2,
      .flash_lp_io_addr = kUlpMainFlashLpIoAddrEsp32S2,
      .target_name = "esp32s2",
      .valid = kUlpMainArtifactEsp32S2Valid,
  };
  return artifact;
#elif defined(USE_ESP32_VARIANT_ESP32S3)
  static const ULPMainArtifact artifact = {
      .binary = kUlpMainBinaryEsp32S3.data(),
      .binary_size = kUlpMainBinaryEsp32S3.size(),
      .run_count_addr = kUlpMainRunCountAddrEsp32S3,
      .flash_lp_io_inverted_addr = kUlpMainFlashLpIoInvertedAddrEsp32S3,
      .pulse_width_us_addr = kUlpMainPulseWidthUsAddrEsp32S3,
      .flash_lp_io_addr = kUlpMainFlashLpIoAddrEsp32S3,
      .target_name = "esp32s3",
      .valid = kUlpMainArtifactEsp32S3Valid,
  };
  return artifact;
#else
  static const ULPMainArtifact artifact = {
      .binary = nullptr,
      .binary_size = 0,
      .run_count_addr = 0,
      .flash_lp_io_inverted_addr = 0,
      .pulse_width_us_addr = 0,
      .flash_lp_io_addr = 0,
      .target_name = "unsupported",
      .valid = false,
  };
  return artifact;
#endif
}

}  // namespace riscv_blink
}  // namespace esphome
