"""
bluetooth_proxy_base — generic Bluetooth proxy for LibreTiny BLE hubs
(bk72xx_ble_tracker, ln882h_ble_tracker, …).

A single proxy works with any tracker hub: the tracker implements the C++
`BleProxyHub` interface (bluetooth_proxy_hub.h), and this component is configured
with a reference to that hub (auto-resolved when exactly one is present, like
`esp32_ble_id`). The proxy forwards raw advertisements to Home Assistant.

    bk72xx_ble_tracker:        # or ln882h_ble_tracker
    bluetooth_proxy_base:      # attaches to the tracker above

This also ships the off-ESP32 `bluetooth_proxy::BluetoothProxy` shim that
api_connection.cpp needs, and the `BleProxyHub` interface header (always available,
so a tracker can implement it whether or not a proxy is configured).
"""

import pathlib
import shutil

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.core import CORE

CODEOWNERS = ["@Bl00d-B0b"]
DEPENDENCIES = ["api"]

CONF_BLE_ID = "ble_id"

bluetooth_proxy_base_ns = cg.esphome_ns.namespace("bluetooth_proxy_base")
BluetoothProxyBase = bluetooth_proxy_base_ns.class_("BluetoothProxyBase", cg.Component)
# Interface implemented by tracker hubs; used by cv.use_id to find the hub.
BleProxyHub = bluetooth_proxy_base_ns.class_("BleProxyHub")

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BluetoothProxyBase),
        # The tracker hub to attach to (auto-injected when only one is declared).
        cv.GenerateID(CONF_BLE_ID): cv.use_id(BleProxyHub),
    }
).extend(cv.COMPONENT_SCHEMA)


def _stage_patch_script():
    """Copy patch_bluetooth_proxy.py into the PlatformIO build tree.

    The script must live under the build src dir so the PlatformIO extra_scripts
    pre-build hook can find and execute it.
    """
    src = pathlib.Path(__file__).parent / "patch_bluetooth_proxy.py"
    dst_dir = (
        pathlib.Path(CORE.build_path)
        / "src"
        / "esphome"
        / "components"
        / "bluetooth_proxy_base"
    )
    dst_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst_dir / "patch_bluetooth_proxy.py")


async def to_code(config):
    # Stage the pre-build patch (idempotent) so the bluetooth_proxy::BluetoothProxy
    # shim is injected into bluetooth_proxy.h on this non-ESP32 build.
    _stage_patch_script()
    cg.add_platformio_option(
        "extra_scripts",
        ["pre:src/esphome/components/bluetooth_proxy_base/patch_bluetooth_proxy.py"],
    )

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    hub = await cg.get_variable(config[CONF_BLE_ID])
    cg.add(var.set_hub(hub))

    # ESPHome-internal proxy defines consumed by api_pb2.h (fixed-size advertisement
    # arrays) and api_connection.cpp (BLE subscription routing). 0 connection slots =
    # advertisement proxy only (no GATT connections on LibreTiny).
    cg.add_define("USE_BLUETOOTH_PROXY")
    cg.add_define("BLUETOOTH_PROXY_MAX_CONNECTIONS", 0)
    cg.add_define("BLUETOOTH_PROXY_ADVERTISEMENT_BATCH_SIZE", 8)
