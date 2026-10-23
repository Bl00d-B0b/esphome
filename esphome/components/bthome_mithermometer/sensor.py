"""Deprecated alias of the bthome receiver: rewrites its entries into bthome config.

Remove after 2027.5.0.
"""

import logging
from typing import Any

from esphome import yaml_util
from esphome.components.ble_device_base import CONF_BLE_HUB_ID
from esphome.components.bthome import CONF_REMOTE_DEVICES, DOMAIN as BTHOME_DOMAIN
import esphome.config_validation as cv
from esphome.const import (
    CONF_ACCURACY_DECIMALS,
    CONF_BATTERY_LEVEL,
    CONF_BATTERY_VOLTAGE,
    CONF_BINDKEY,
    CONF_ENTITY_CATEGORY,
    CONF_HUMIDITY,
    CONF_ICON,
    CONF_ID,
    CONF_KEY,
    CONF_MAC_ADDRESS,
    CONF_PLATFORM,
    CONF_SENSOR,
    CONF_SETUP_PRIORITY,
    CONF_SIGNAL_STRENGTH,
    CONF_SOURCE_ID,
    CONF_TEMPERATURE,
    CONF_TYPE,
    ENTITY_CATEGORY_DIAGNOSTIC,
)
from esphome.types import ConfigType

from . import DOMAIN

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@nagyrobi"]

REMOVAL_VERSION = "2027.5.0"
# Pre-migration binding key, still accepted by the bthome hub schema.
CONF_ESP32_BLE_ID = "esp32_ble_id"
# Either spelling of the binding counts as the hub already being bound.
_HUB_ID_KEYS = {
    CONF_BLE_HUB_ID: CONF_ESP32_BLE_ID,
    CONF_ESP32_BLE_ID: CONF_BLE_HUB_ID,
}

# Legacy sub-sensor -> bthome object type and the entity defaults the bthome
# object type table does not set, so migrated entities keep their attributes.
SUB_SENSORS: dict[str, tuple[str, dict[str, Any]]] = {
    CONF_TEMPERATURE: ("TEMPERATURE_C_E2", {}),
    CONF_HUMIDITY: ("HUMIDITY_PCT_E2", {}),
    CONF_BATTERY_LEVEL: (
        "BATTERY_PCT",
        {CONF_ACCURACY_DECIMALS: 0, CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_DIAGNOSTIC},
    ),
    CONF_BATTERY_VOLTAGE: (
        "VOLTAGE_V_E3",
        {
            CONF_ACCURACY_DECIMALS: 3,
            CONF_ICON: "mdi:battery-plus",
            CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_DIAGNOSTIC,
        },
    ),
    CONF_SIGNAL_STRENGTH: (
        "SIGNAL_STRENGTH",
        {CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_DIAGNOSTIC},
    ),
}

# The sub-sensor entries are validated by the bthome sensor platform after the rewrite.
_SUB_SENSOR_SCHEMA = cv.Schema({}, extra=cv.ALLOW_EXTRA)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Optional(CONF_ID): cv.string_strict,
            cv.Required(CONF_MAC_ADDRESS): cv.mac_address,
            cv.Optional(CONF_BINDKEY): cv.bind_key,
            cv.Optional(CONF_BLE_HUB_ID): cv.string_strict,
            cv.Optional(CONF_ESP32_BLE_ID): cv.string_strict,
            cv.Optional(CONF_SETUP_PRIORITY): cv.float_,
            **{cv.Optional(key): _SUB_SENSOR_SCHEMA for key in SUB_SENSORS},
        }
    ),
    cv.has_at_most_one_key(CONF_BLE_HUB_ID, CONF_ESP32_BLE_ID),
)


def _hub_config(config: dict[str, Any]) -> ConfigType:
    """Return the raw bthome hub entry, creating it when the config has none."""
    hub = config.get(BTHOME_DOMAIN)
    if hub is None:
        hub = config[BTHOME_DOMAIN] = {}
    if not isinstance(hub, dict):
        raise cv.Invalid(
            f"'{BTHOME_DOMAIN}:' must be a mapping to receive the "
            f"'{DOMAIN}' remote devices"
        )
    devices = hub.setdefault(CONF_REMOTE_DEVICES, [])
    if not isinstance(devices, list):
        raise cv.Invalid(f"'{BTHOME_DOMAIN}: {CONF_REMOTE_DEVICES}' must be a list")
    return hub


def _merge_hub_options(hub: ConfigType, legacy: ConfigType) -> ConfigType:
    """Copy the hub binding and setup priority onto the bthome entry; reject conflicts."""
    options: ConfigType = {}
    for key in (CONF_BLE_HUB_ID, CONF_ESP32_BLE_ID, CONF_SETUP_PRIORITY):
        if (value := legacy.get(key)) is None:
            continue
        options[key] = value
        current = hub.get(key)
        if current is None and key in _HUB_ID_KEYS:
            current = hub.get(_HUB_ID_KEYS[key])
        if current is None:
            hub[key] = value
        elif str(current) != str(value):
            raise cv.Invalid(
                f"'{key}: {value}' differs from the '{BTHOME_DOMAIN}' component "
                f"({current})",
                path=[key],
            )
    return options


def _migrate(entry: ConfigType, config: dict[str, Any]) -> list[ConfigType]:
    raw = dict(entry)
    raw.pop(CONF_PLATFORM, None)
    legacy = CONFIG_SCHEMA(raw)

    mac = legacy[CONF_MAC_ADDRESS]
    device_id = legacy.get(CONF_ID) or f"{DOMAIN}_{str(mac).replace(':', '').lower()}"
    device: ConfigType = {CONF_ID: device_id, CONF_MAC_ADDRESS: str(mac)}
    if (bindkey := legacy.get(CONF_BINDKEY)) is not None:
        device[CONF_KEY] = bindkey

    sensors: list[ConfigType] = []
    for key, (object_type, defaults) in SUB_SENSORS.items():
        if (sub := legacy.get(key)) is None:
            continue
        sensors.append(
            {
                CONF_PLATFORM: BTHOME_DOMAIN,
                CONF_SOURCE_ID: device_id,
                CONF_TYPE: object_type,
                **defaults,
                **sub,
            }
        )

    hub = _hub_config(config)
    hub_options = _merge_hub_options(hub, legacy)
    hub[CONF_REMOTE_DEVICES].append(device)

    _LOGGER.warning(
        "[%s] The '%s' sensor platform is deprecated, use '%s'. Will be removed in %s. "
        "The entry for %s is read as:\n\n%s",
        DOMAIN,
        DOMAIN,
        BTHOME_DOMAIN,
        REMOVAL_VERSION,
        mac,
        yaml_util.dump(
            {
                BTHOME_DOMAIN: {**hub_options, CONF_REMOTE_DEVICES: [device]},
                CONF_SENSOR: sensors,
            }
        ),
    )
    return sensors


LEGACY_PLATFORM_MIGRATE = _migrate
