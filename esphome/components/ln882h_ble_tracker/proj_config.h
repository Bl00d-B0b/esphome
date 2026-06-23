#pragma once

// Minimal project config for ln882h_ble_tracker.
// Mirrors project/ble_mcu_data_trans/cfg/proj_config.h — only the defines
// needed by the BLE SDK sources we compile (ble_port.c, ble_arch_main.c, etc.)

#define DISABLE (0)  // NOLINT(cppcoreguidelines-macro-usage)
#define ENABLE (1)   // NOLINT(cppcoreguidelines-macro-usage)

// SDK-required macros — names are defined by the LN882H BLE SDK and cannot be changed.
#ifndef __CONFIG_OS_KERNEL
#define __CONFIG_OS_KERNEL  // NOLINT(bugprone-reserved-identifier,cert-dcl37-c,cert-dcl51-cpp)
#endif
#define __CFG_OS_TICKLESS_EN (0)  // NOLINT(bugprone-reserved-identifier,cert-dcl37-c,cert-dcl51-cpp)

// Clock — LN882H PLL at 160 MHz
#define XTAL_CLOCK (40000000)     // NOLINT(cppcoreguidelines-macro-usage)
#define RCO_CLOCK (32000)         // NOLINT(cppcoreguidelines-macro-usage)
#define PLL_CLOCK (160000000)     // NOLINT(cppcoreguidelines-macro-usage)
#define SYSTEM_CLOCK (160000000)  // NOLINT(cppcoreguidelines-macro-usage)

// BLE compilation gate — same default as BK7231N/T sys_config.h (CFG_SUPPORT_BLE = 1).
// LN882H always has BLE hardware, so BLE is compiled by default just like BK7231N/T.
// 1 = BLE stack compiled and linked (default; chip has BLE hardware).
// 0 = BLE stack excluded (explicit override only, e.g. for BLE-less build variants).
// Can be overridden at build time via -DCFG_SUPPORT_BLE=0.
#ifndef CFG_SUPPORT_BLE
#define CFG_SUPPORT_BLE 1  // NOLINT(cppcoreguidelines-macro-usage)
#endif

// Module flags
#define FLASH_XIP ENABLE      // NOLINT(cppcoreguidelines-macro-usage)
#define LN_ASSERT_EN ENABLE   // NOLINT(cppcoreguidelines-macro-usage)
#define HAL_ASSERT_EN ENABLE  // NOLINT(cppcoreguidelines-macro-usage)
#define PRINTF_OMIT DISABLE   // NOLINT(cppcoreguidelines-macro-usage)

// Endian check
#if defined(__GNUC__)
#if (__BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__)
#error "LN882H BLE requires little-endian mode"
#endif
#endif

#ifndef LITTLE_ENDIAN
#define LITTLE_ENDIAN 1234  // NOLINT(cppcoreguidelines-macro-usage)
#endif

#ifdef __cplusplus
namespace esphome::ln882h_ble_tracker {}  // namespace esphome::ln882h_ble_tracker
#endif
