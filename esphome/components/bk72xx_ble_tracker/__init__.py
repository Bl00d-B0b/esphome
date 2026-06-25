"""
BK72xx BLE Tracker — ESPHome BLE 5.x scanner for the BLE-5.x-capable LibreTiny
Beken chips (beken-72xx family): BK7231N/BK7236 (BLE 5.1), BK7238/BK7252N/BK7253
(BLE 5.2), and any future BLE-5.x SoC.

Scans for BLE 5.1 advertisements using the Beken BDK BLE SDK.

Unlike the LN882H sibling, no framework patch is needed: the LibreTiny beken-72xx
builder already compiles and links the BLE 5.x stack (CFG_SUPPORT_BLE=1 +
CFG_BLE_VERSION=BLE_VERSION_5_x; prebuilt libble_<chip>.a per SoC). This component
only calls into it via the public ble_api.h.

Capability is detected at compile time, not by a chip list: the C++ guards on
`__has_include("ble_api.h")` — the Beken BLE 5.x public API header, which the
LibreTiny beken-72xx builder ships only for BLE-5.x SoCs. Any BLE-5.x SoC builds;
BK7231T/BK7251/BK7271 (BLE 4.2) and BK7231Q (no BLE) fail with a clear #error.

Scan modes:
  continuous: true  — scan runs forever; never stops automatically.
                      Use this when the radio is dedicated to BLE.
  continuous: false — scan runs for `duration` ms, fires on_scan_end, then stops.
                      Does not restart automatically; pair with api: on_client_connected
                      to start scanning only while Home Assistant is connected, allowing
                      the single-core BK7231N radio to service WiFi in between scans.

Typical usage — continuous proxy (radio always on BLE):

    bk72xx_ble_tracker:
      scan_parameters:
        continuous: true

    bluetooth_proxy:       # separate component: forwards advertisements to Home Assistant

Single-core WiFi/BLE time-share (scan only while HA is connected):

    bk72xx_ble_tracker:
      scan_parameters:
        continuous: false         # do not auto-start on boot

    api:
      on_client_connected:
        - bk72xx_ble_tracker.start_scan:
            continuous: true      # keep scanning for the whole connection
      on_client_disconnected:
        - bk72xx_ble_tracker.stop_scan:

Requires:
  - LibreTiny platform (beken-72xx family, a BLE-5.x SoC)
  - ESPHome Native API (api: component)
"""

import re

from esphome import automation
import esphome.codegen as cg
from esphome.components import libretiny
from esphome.components.libretiny.const import FAMILY_BK7231N
import esphome.config_validation as cv
from esphome.const import (
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
CONF_BK72XX_BLE_ID = "bk72xx_ble_id"

# Requires the bk72xx platform. BLE-5.x capability is enforced at compile time in
# the C++ via a `__has_include("ble_api.h")` guard — not by a chip allowlist here —
# so any BLE-5.x Beken SoC (present or future: BK7231N/BK7236 at 5.1, BK7238/
# BK7252N/BK7253 at 5.2, …) is supported automatically. A non-5.x SoC fails the
# build with a clear #error instead of a cryptic missing-header/link failure.
DEPENDENCIES = ["bk72xx"]
CODEOWNERS = ["@Bl00d-B0b"]


# BleProxyHub: the neutral hub interface (bluetooth_proxy) that the tracker
# implements so the generic Bluetooth proxy can attach via cv.use_id. Declared by
# namespace path (no module import) to avoid a hard Python dependency.
ble_proxy_hub = cg.esphome_ns.namespace("bluetooth_proxy").class_("BleProxyHub")

bk72xx_ble_tracker_ns = cg.esphome_ns.namespace("bk72xx_ble_tracker")
BK72xxBLETracker = bk72xx_ble_tracker_ns.class_(
    "BK72xxBLETracker", cg.Component, ble_proxy_hub
)

# NOTE: this component is the BLE scanner only. Local BLE sensor support
# (ble_presence/ble_rssi/ble_scanner/bthome) and the Bluetooth proxy
# (bluetooth_proxy) are separate components that attach to this tracker —
# they are intentionally NOT pulled in here, so the scanner stands on its own.
StartScanAction = bk72xx_ble_tracker_ns.class_("StartScanAction", automation.Action)
StopScanAction = bk72xx_ble_tracker_ns.class_("StopScanAction", automation.Action)

# Trigger classes — use the shared esp32_ble_tracker class names so that BLE
# sensor components (ble_presence, ble_rssi, bthome_mithermometer, …) compile
# on BK7231N without modification.
ESPBTAdvertiseTrigger = bk72xx_ble_tracker_ns.class_(
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
BLEServiceDataAdvertiseTrigger = bk72xx_ble_tracker_ns.class_(
    "BLEServiceDataAdvertiseTrigger", automation.Trigger.template(adv_data_t_const_ref)
)
BLEManufacturerDataAdvertiseTrigger = bk72xx_ble_tracker_ns.class_(
    "BLEManufacturerDataAdvertiseTrigger",
    automation.Trigger.template(adv_data_t_const_ref),
)
BLEEndOfScanTrigger = bk72xx_ble_tracker_ns.class_(
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
    Catching it here gives a clear error instead of a runtime bk_ble_scan_start
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
            # interval/window default to the BK reference scan rate — 100 ms / 30 ms,
            # a 30 % duty cycle. Converted to the controller's 0.625 ms BLE units in
            # to_code(). (LN882H's SDK recommends a different 100 / 50 ms = 50 %.)
            cv.Optional(CONF_INTERVAL, default="100ms"): cv.positive_time_period,
            cv.Optional(CONF_WINDOW, default="30ms"): cv.positive_time_period,
            cv.Optional(CONF_CONTINUOUS, default=True): cv.boolean,
        }
    ),
    validate_scan_parameters,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BK72xxBLETracker),
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
# Automation actions: bk72xx_ble_tracker.start_scan / .stop_scan
# ---------------------------------------------------------------------------
# Single-core WiFi/BLE workaround - scan only while HA is connected:
#
#   api:
#     on_client_connected:
#       - bk72xx_ble_tracker.start_scan:
#           continuous: true
#     on_client_disconnected:
#       - bk72xx_ble_tracker.stop_scan:


@automation.register_action(
    "bk72xx_ble_tracker.start_scan",
    StartScanAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(BK72xxBLETracker),
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
    "bk72xx_ble_tracker.stop_scan",
    StopScanAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(BK72xxBLETracker),
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
    cg.add(var.set_scan_interval(int(scan[CONF_INTERVAL].total_milliseconds / 0.625)))
    cg.add(var.set_scan_window(int(scan[CONF_WINDOW].total_milliseconds / 0.625)))
    cg.add(var.set_scan_duration(scan[CONF_DURATION].total_milliseconds))
    cg.add(var.set_scan_continuous(scan[CONF_CONTINUOUS]))

    # Enable the BLE stack. ESPHome's libretiny platform deliberately sets
    # CFG_SUPPORT_BLE=0 on BK7231N/BK7238 ("saves ~21KB RAM/~200KB Flash; ESPHome
    # doesn't use BLE on LibreTiny") via custom_options.sys_config#h. We need BLE,
    # so override that SAME key (the '#h' maps to sys_config.h; value is a list).
    # This component's to_code runs after the bk72xx platform's, so this wins.
    cg.add_platformio_option("custom_options.sys_config#h", ["CFG_SUPPORT_BLE=1"])

    # The BDK exposes the controller's BLE address as `common_default_bdaddr` on
    # BK7231N, but NOT on BK7238 (its BLE stack has no such symbol; the address is
    # derived from the WiFi MAC instead — same as OpenBeken's BK7238 path). Tell the
    # C++ which path is available so it doesn't reference a missing symbol.
    if libretiny.get_libretiny_family() == FAMILY_BK7231N:
        cg.add_define("BK72XX_BLE_HAS_COMMON_BDADDR")

    # The Bluetooth proxy is the separate bluetooth_proxy component, added
    # independently (e.g. when testing proxy mode) — nothing to wire here.

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
