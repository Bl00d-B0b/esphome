"""
PlatformIO pre-build extra script for bluetooth_proxy_base.

Patches bluetooth_proxy.h in the ESPHome build tree to include
bluetooth_proxy_compat.h, which provides the bluetooth_proxy::BluetoothProxy
abstract interface that api_connection.cpp requires on non-ESP32 targets (the
real bluetooth_proxy.h is guarded by #ifdef USE_ESP32 and is empty on LibreTiny).

Shared by every LibreTiny BLE hub that enables a Bluetooth proxy (bk72xx_ble_tracker,
ln882h_ble_tracker, …) so the shim is defined exactly once.
"""
# ruff: noqa: F821, BLE001, PTH103, PTH111, PTH112, PTH113, PTH118, PTH119, PTH120, PTH122, PTH123, PTH207
# flake8: noqa: F821

# pylint: disable=undefined-variable  # env is a SCons/PlatformIO-injected global
import contextlib
import os

# Import PlatformIO SCons environment.  Outside PlatformIO this is silently skipped;
# _env stays None and the rest of the script does nothing.
_env = None
with contextlib.suppress(NameError):
    Import("env")  # noqa: F821  (SCons global, injected at runtime by PlatformIO)
    _env = env  # noqa: F821

if _env is None:
    pass  # Running in lint / import context — nothing to do
else:
    TAG = "bluetooth_proxy_base patch"

    src_dir = _env.subst("$PROJECT_SRC_DIR")

    bluetooth_proxy_h = os.path.join(
        src_dir, "esphome", "components", "bluetooth_proxy", "bluetooth_proxy.h"
    )

    INCLUDE_LINE = (
        '#include "esphome/components/bluetooth_proxy_base/bluetooth_proxy_compat.h"'
    )

    os.makedirs(os.path.dirname(bluetooth_proxy_h), exist_ok=True)

    if not os.path.isfile(bluetooth_proxy_h):
        with open(bluetooth_proxy_h, "w", encoding="utf-8") as fh:
            fh.write(f"#pragma once\n// Stub created by {TAG}\n{INCLUDE_LINE}\n")
        print(f"{TAG}: created bluetooth_proxy.h stub")
    else:
        with open(bluetooth_proxy_h, encoding="utf-8") as fh:
            content = fh.read()
        if INCLUDE_LINE in content:
            print(f"{TAG}: bluetooth_proxy.h already patched")
        else:
            with open(bluetooth_proxy_h, "a", encoding="utf-8") as fh:
                fh.write(f"\n// Non-ESP32 compat added by {TAG}\n{INCLUDE_LINE}\n")
            print(f"{TAG}: patched bluetooth_proxy.h for non-ESP32")
