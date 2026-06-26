"""
ble_device_base — shared BLE-hub abstraction for ESPHome.

Lets BLE sensor components (ble_presence, ble_rssi, ble_scanner,
bthome_mithermometer, …) attach to ANY BLE tracker hub without per-platform
branches.  Supported hubs are enumerated once in _BLE_HUBS; adding a new BLE
platform is a single entry here plus a hub component that exposes
register_ble_device().

Each hub component must provide, at module level:
  - a class <HubClass> (the tracker hub component class),
  - async def register_ble_device(var, config): register `var` as a listener.
esp32_ble_tracker already satisfies this contract; ln882h_ble_tracker and
bk72xx_ble_tracker add the two-line helper.

C++ side: the neutral advertisement types (ESPBTUUID / ESPBTDevice / ServiceData
/ ESPBLEiBeacon / ESPBTDeviceListener) live in ble_device/ble_device.h and, on
non-ESP32 targets, are aliased into the esp32_ble_tracker namespace so existing
sensor C++ compiles unchanged. On ESP32 the real esp32_ble_tracker types are used.
"""

import importlib

import esphome.config_validation as cv
from esphome.core import CORE

CODEOWNERS = ["@Bl00d-B0b"]

# CORE.target_platform -> (component module, hub class attribute, config id key).
# To support another BLE platform, add ONE line here (and ship a hub component
# that provides a matching register_ble_device()).  Nothing else changes.
_BLE_HUBS = {
    "esp32": ("esp32_ble_tracker", "ESP32BLETracker", "esp32_ble_id"),
    "ln882x": ("ln882h_ble_tracker", "LN882HBLETracker", "ln882h_ble_id"),
    "bk72xx": ("bk72xx_ble_tracker", "BK72xxBLETracker", "bk72xx_ble_id"),
}


def _import_hub(module_name):
    """Import a hub component module, or None if it is not installed."""
    try:
        return importlib.import_module(f"esphome.components.{module_name}")
    except ImportError:
        return None


def _active_hub():
    """Return (module, hub_class, conf_id) for the current target platform, or None."""
    entry = _BLE_HUBS.get(CORE.target_platform or "")
    if entry is None:
        return None
    module_name, class_name, conf_id = entry
    module = _import_hub(module_name)
    if module is None:
        return None
    return module, getattr(module, class_name), conf_id


def ble_device_schema():
    """Schema fragment: an optional id for every installed BLE hub.

    Static (platform-independent) so it can be built at import time; the active
    hub is chosen later by inject_ble_hub() / register_ble_device().
    """
    schema = {}
    for module_name, class_name, conf_id in _BLE_HUBS.values():
        module = _import_hub(module_name)
        if module is None:
            continue
        schema[cv.Optional(conf_id)] = cv.use_id(getattr(module, class_name))
    return cv.Schema(schema)


def inject_ble_hub(config):
    """Validator: auto-fill the active platform's hub id if the user omitted it."""
    active = _active_hub()
    if active is None:
        return config
    _module, hub_class, conf_id = active
    if conf_id in config:
        return config
    config = dict(config)
    config[conf_id] = cv.use_id(hub_class)(None)
    return config


async def register_ble_device(var, config):
    """Register `var` as a BLE device listener on the active platform's hub."""
    active = _active_hub()
    if active is None:
        raise cv.Invalid(
            f"No BLE tracker hub is available for platform "
            f"'{CORE.target_platform}'. Supported platforms: {', '.join(_BLE_HUBS)}."
        )
    module, _hub_class, _conf_id = active
    return await module.register_ble_device(var, config)
