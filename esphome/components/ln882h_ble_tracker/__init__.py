"""
LN882H BLE Tracker — ESPHome component for LibreTiny LN882H (lightning-ln882h family).

Scans for BLE 5.1 advertisements using the LN882H BLE SDK and optionally forwards
them to Home Assistant as a Bluetooth proxy (passive advertisements only).

Scan modes:
  continuous: true  — scan runs forever; never stops automatically.
                      Use this when the radio is dedicated to BLE (e.g. LN882H
                      board without WiFi in use, or with coexistence handled by HW).
  continuous: false — scan runs for `duration` ms, fires on_scan_end, then stops.
                      Does not restart automatically; pair with api: on_client_connected
                      to start scanning only while Home Assistant is connected, allowing
                      the single-core LN882H radio to service WiFi in between scans.

Typical usage — continuous proxy (radio always on BLE):

    ln882h_ble_tracker:
      scan_parameters:
        active: true
        continuous: true
    bluetooth_proxy:     # forward advertisements to Home Assistant (separate component)

Single-core WiFi/BLE time-share (scan only while HA is connected):

    ln882h_ble_tracker:
      scan_parameters:
        active: true
        continuous: false         # do not auto-start on boot

    api:
      on_client_connected:
        - ln882h_ble_tracker.start_scan:
            continuous: true      # keep scanning for the whole connection
      on_client_disconnected:
        - ln882h_ble_tracker.stop_scan:

Requires:
  - LibreTiny platform (lightning-ln882h family)
  - ESPHome Native API (api: component)
"""

import re

from esphome import automation
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import (
    CONF_ACTIVE,
    CONF_CONTINUOUS,
    CONF_DURATION,
    CONF_ID,
    CONF_INTERVAL,
    CONF_MAC_ADDRESS,
    CONF_MANUFACTURER_ID,
    CONF_ON_BLE_ADVERTISE,
    CONF_ON_BLE_MANUFACTURER_DATA_ADVERTISE,
    CONF_ON_BLE_SERVICE_DATA_ADVERTISE,
    CONF_SERVICE_UUID,
    CONF_TRIGGER_ID,
)

CONF_WINDOW = "window"
CONF_SCAN_PARAMETERS = "scan_parameters"
CONF_ON_SCAN_END = "on_scan_end"
CONF_LN882H_BLE_ID = "ln882h_ble_id"

# LN882H-only component: requires the ln882x platform. Fail fast (clear config
# error) instead of a cryptic link error against the LN BLE SDK on other targets.
DEPENDENCIES = ["ln882x"]
# The tracker's C++ uses the shared ble_device_base advertisement types (ESPBTDevice, …),
# and BLE sensor components register through ble_device_base, so pull it into every build.
AUTO_LOAD = ["ble_device_base"]
CODEOWNERS = ["@Bl00d-B0b"]

# BleProxyHub: the neutral hub interface (bluetooth_proxy) that the tracker
# implements so the generic Bluetooth proxy can attach via cv.use_id. Declared by
# namespace path (no module import) to avoid a hard Python dependency.
ble_proxy_hub = cg.esphome_ns.namespace("bluetooth_proxy").class_("BleProxyHub")

ln882h_ble_tracker_ns = cg.esphome_ns.namespace("ln882h_ble_tracker")
LN882HBLETracker = ln882h_ble_tracker_ns.class_(
    "LN882HBLETracker", cg.Component, ble_proxy_hub
)


async def register_ble_device(var, config):
    """Register `var` as a BLE-advertisement listener on this hub.

    This is the ble_device_base hub contract: ble_device_base.register_ble_device()
    dispatches here for the ln882x platform, and inject_ble_hub() has already filled
    config[CONF_LN882H_BLE_ID] with the hub id (auto-resolved when a single hub exists).
    """
    paren = await cg.get_variable(config[CONF_LN882H_BLE_ID])
    cg.add(paren.register_listener(var))
    return var


# NOTE: this component is the BLE scanner only. Local BLE sensor support
# (ble_presence/ble_rssi/ble_scanner/bthome) and the Bluetooth proxy
# (bluetooth_proxy) are separate components that attach to this tracker —
# they are intentionally NOT pulled in here, so the scanner stands on its own.
StartScanAction = ln882h_ble_tracker_ns.class_("StartScanAction", automation.Action)
StopScanAction = ln882h_ble_tracker_ns.class_("StopScanAction", automation.Action)

# Trigger classes — use the shared esp32_ble_tracker class names so that BLE
# sensor components (ble_presence, ble_rssi, bthome_mithermometer, …) compile
# on LN882H without modification.
ESPBTAdvertiseTrigger = ln882h_ble_tracker_ns.class_(
    "ESPBTAdvertiseTrigger",
    automation.Trigger.template(
        cg.esphome_ns.namespace("esp32_ble_tracker")
        .class_("ESPBTDevice")
        .operator("ref")
        .operator("const")
    ),
)
adv_data_t = cg.std_vector.template(cg.uint8)
adv_data_t_const_ref = adv_data_t.operator("ref").operator("const")
BLEServiceDataAdvertiseTrigger = ln882h_ble_tracker_ns.class_(
    "BLEServiceDataAdvertiseTrigger", automation.Trigger.template(adv_data_t_const_ref)
)
BLEManufacturerDataAdvertiseTrigger = ln882h_ble_tracker_ns.class_(
    "BLEManufacturerDataAdvertiseTrigger",
    automation.Trigger.template(adv_data_t_const_ref),
)
BLEEndOfScanTrigger = ln882h_ble_tracker_ns.class_(
    "BLEEndOfScanTrigger", automation.Trigger.template()
)

# ---------------------------------------------------------------------------
# Bluetooth UUID validator (replicated from esp32_ble component to avoid
# importing an ESP32-only module on a LibreTiny build).
# ---------------------------------------------------------------------------

bt_uuid16_format = "XXXX"
bt_uuid32_format = "XXXXXXXX"
bt_uuid128_format = "XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX"

_BT_UUID16_RE = re.compile(r"^[0-9A-F]{4}$")
_BT_UUID32_RE = re.compile(r"^[0-9A-F]{8}$")
_BT_UUID128_RE = re.compile(
    r"^[0-9A-F]{8}-[0-9A-F]{4}-[0-9A-F]{4}-[0-9A-F]{4}-[0-9A-F]{12}$"
)


def bt_uuid(value):
    """Validate a Bluetooth UUID (16-bit, 32-bit, or 128-bit)."""
    s = cv.string_strict(value).upper()
    if _BT_UUID16_RE.match(s):
        return s
    if _BT_UUID32_RE.match(s):
        return s
    if _BT_UUID128_RE.match(s):
        return s
    # Accept 32-char hex without dashes and reformat
    stripped = s.replace("-", "")
    if re.match(r"^[0-9A-F]{32}$", stripped):
        return (
            f"{stripped[0:8]}-{stripped[8:12]}-{stripped[12:16]}"
            f"-{stripped[16:20]}-{stripped[20:32]}"
        )
    raise cv.Invalid(
        f"Invalid Bluetooth UUID '{value}'. "
        "Expected 4 hex chars (16-bit), 8 hex chars (32-bit), "
        "or 'XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX' (128-bit)."
    )


def _as_hex(value):
    """Format a 4- or 8-char UUID string as a C++ ULL hex literal."""
    return cg.RawExpression(f"0x{value}ULL")


def _as_reversed_hex_array(value):
    """Format a 128-bit UUID string as a little-endian uint8_t array expression."""
    value = value.replace("-", "")
    parts = [f"0x{value[i : i + 2]}" for i in range(0, len(value), 2)]
    return cg.RawExpression(
        f"(uint8_t*)(const uint8_t[16]){{{','.join(reversed(parts))}}}"
    )


# ---------------------------------------------------------------------------
# scan_parameters schema - all defaults match esp32_ble_tracker
# ---------------------------------------------------------------------------
def validate_scan_parameters(config):
    """Reject impossible window/interval/duration combinations at config time.

    Mirrors esp32_ble_tracker: the controller cannot scan for longer than the
    interval, and a too-short duration would fire on_scan_end almost immediately.
    Catching it here gives a clear error instead of a runtime ln_ble_scan_start
    failure and the 1/sec retry loop.
    """
    duration = config[CONF_DURATION]
    interval = config[CONF_INTERVAL]
    window = config[CONF_WINDOW]

    if window > interval:
        raise cv.Invalid(
            f"Scan window ({window}) needs to be smaller than scan interval ({interval})"
        )

    if interval.total_milliseconds * 3 > duration.total_milliseconds:
        raise cv.Invalid(
            "Scan duration needs to be at least three times the scan interval to"
            " cover at least three scan intervals."
        )

    return config


SCAN_PARAMETERS_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Optional(CONF_DURATION, default="5min"): cv.positive_time_period_seconds,
            # interval/window defaults are the LN882H SDK's own recommended values
            # (ln_ble_scan.h: SCAN_INTERVAL_DEF 0xA0 = 100 ms, SCAN_WINDOW_DEF 0x50 =
            # 50 ms; 50 % duty cycle) — the value the chip is designed to scan at.
            cv.Optional(CONF_INTERVAL, default="100ms"): cv.positive_time_period,
            cv.Optional(CONF_WINDOW, default="50ms"): cv.positive_time_period,
            # Default matches esp32_ble_tracker. Active scanning solicits scan
            # responses, which are forwarded to the Bluetooth proxy (device names).
            cv.Optional(CONF_ACTIVE, default=True): cv.boolean,
            cv.Optional(CONF_CONTINUOUS, default=True): cv.boolean,
        }
    ),
    validate_scan_parameters,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(LN882HBLETracker),
        cv.Optional(CONF_SCAN_PARAMETERS, default={}): SCAN_PARAMETERS_SCHEMA,
        # ---------------------------------------------------------------------------
        # Automation triggers — mirror esp32_ble_tracker for drop-in compatibility
        # ---------------------------------------------------------------------------
        cv.Optional(CONF_ON_BLE_ADVERTISE): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(ESPBTAdvertiseTrigger),
                cv.Optional(CONF_MAC_ADDRESS): cv.ensure_list(cv.mac_address),
            }
        ),
        cv.Optional(CONF_ON_BLE_SERVICE_DATA_ADVERTISE): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    BLEServiceDataAdvertiseTrigger
                ),
                cv.Optional(CONF_MAC_ADDRESS): cv.mac_address,
                cv.Required(CONF_SERVICE_UUID): bt_uuid,
            }
        ),
        cv.Optional(
            CONF_ON_BLE_MANUFACTURER_DATA_ADVERTISE
        ): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    BLEManufacturerDataAdvertiseTrigger
                ),
                cv.Optional(CONF_MAC_ADDRESS): cv.mac_address,
                cv.Required(CONF_MANUFACTURER_ID): bt_uuid,
            }
        ),
        cv.Optional(CONF_ON_SCAN_END): automation.validate_automation(
            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(BLEEndOfScanTrigger)}
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


# ---------------------------------------------------------------------------
# Automation actions: ln882h_ble_tracker.start_scan / .stop_scan
# ---------------------------------------------------------------------------
# Single-core WiFi/BLE workaround - scan only while HA is connected:
#
#   api:
#     on_client_connected:
#       - ln882h_ble_tracker.start_scan:
#           continuous: true
#     on_client_disconnected:
#       - ln882h_ble_tracker.stop_scan:


@automation.register_action(
    "ln882h_ble_tracker.start_scan",
    StartScanAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(LN882HBLETracker),
            cv.Optional(CONF_CONTINUOUS, default=True): cv.boolean,
        }
    ),
    synchronous=True,
)
async def start_scan_action_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg)
    cg.add(var.set_parent(paren))
    cg.add(var.set_continuous(config[CONF_CONTINUOUS]))
    return var


@automation.register_action(
    "ln882h_ble_tracker.stop_scan",
    StopScanAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(LN882HBLETracker),
        }
    ),
    synchronous=True,
)
async def stop_scan_action_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg)
    cg.add(var.set_parent(paren))
    return var


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    scan = config[CONF_SCAN_PARAMETERS]
    cg.add(var.set_scan_active(scan[CONF_ACTIVE]))
    cg.add(var.set_scan_interval(int(scan[CONF_INTERVAL].total_milliseconds / 0.625)))
    cg.add(var.set_scan_window(int(scan[CONF_WINDOW].total_milliseconds / 0.625)))
    cg.add(var.set_scan_duration(scan[CONF_DURATION].total_milliseconds))
    cg.add(var.set_scan_continuous(scan[CONF_CONTINUOUS]))

    # The LN882H BLE SDK is compiled only when CFG_SUPPORT_BLE=1. The LibreTiny
    # lightning-ln882h builder loads the stock proj_config.h (CFG_SUPPORT_BLE
    # defaults to 0) and merges custom_options.proj_config into its build CONFIG.
    # Enable BLE there automatically so simply adding this component is enough —
    # users never need to set a manual build flag.
    cg.add_platformio_option("custom_options.proj_config", "CFG_SUPPORT_BLE=1")

    # ---------------------------------------------------------------------------
    # Automation trigger code generation
    # ---------------------------------------------------------------------------

    ESPBTDeviceConstRef = (
        cg.esphome_ns.namespace("esp32_ble_tracker")
        .class_("ESPBTDevice")
        .operator("ref")
        .operator("const")
    )

    for conf in config.get(CONF_ON_BLE_ADVERTISE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        if CONF_MAC_ADDRESS in conf:
            addr_list = [it.as_hex for it in conf[CONF_MAC_ADDRESS]]
            cg.add(trigger.set_addresses(addr_list))
        await automation.build_automation(trigger, [(ESPBTDeviceConstRef, "x")], conf)

    for conf in config.get(CONF_ON_BLE_SERVICE_DATA_ADVERTISE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        uuid = conf[CONF_SERVICE_UUID]
        if len(uuid) == len(bt_uuid16_format):
            cg.add(trigger.set_service_uuid16(_as_hex(uuid)))
        elif len(uuid) == len(bt_uuid32_format):
            cg.add(trigger.set_service_uuid32(_as_hex(uuid)))
        else:
            cg.add(trigger.set_service_uuid128(_as_reversed_hex_array(uuid)))
        if CONF_MAC_ADDRESS in conf:
            cg.add(trigger.set_address(conf[CONF_MAC_ADDRESS].as_hex))
        await automation.build_automation(trigger, [(adv_data_t_const_ref, "x")], conf)

    for conf in config.get(CONF_ON_BLE_MANUFACTURER_DATA_ADVERTISE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        uuid = conf[CONF_MANUFACTURER_ID]
        if len(uuid) == len(bt_uuid16_format):
            cg.add(trigger.set_manufacturer_uuid16(_as_hex(uuid)))
        elif len(uuid) == len(bt_uuid32_format):
            cg.add(trigger.set_manufacturer_uuid32(_as_hex(uuid)))
        else:
            cg.add(trigger.set_manufacturer_uuid128(_as_reversed_hex_array(uuid)))
        if CONF_MAC_ADDRESS in conf:
            cg.add(trigger.set_address(conf[CONF_MAC_ADDRESS].as_hex))
        await automation.build_automation(trigger, [(adv_data_t_const_ref, "x")], conf)

    for conf in config.get(CONF_ON_SCAN_END, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
