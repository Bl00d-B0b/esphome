import esphome.codegen as cg
from esphome.components import ble_device_base
import esphome.config_validation as cv
from esphome.const import CONF_BINDKEY, CONF_ID, CONF_MAC_ADDRESS

AUTO_LOAD = ["ble_device_base", "xiaomi_ble"]
CODEOWNERS = ["@jesserockz"]
MULTI_CONF = True

xiaomi_rtcgq02lm_ns = cg.esphome_ns.namespace("xiaomi_rtcgq02lm")
XiaomiRTCGQ02LM = xiaomi_rtcgq02lm_ns.class_(
    "XiaomiRTCGQ02LM", ble_device_base.ESPBTDeviceListener, cg.Component
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(XiaomiRTCGQ02LM),
            cv.Required(CONF_BINDKEY): cv.bind_key,
            cv.Required(CONF_MAC_ADDRESS): cv.mac_address,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    ble_device_base.inject_ble_hub,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await ble_device_base.register_ble_device(var, config)

    cg.add(var.set_address(config[CONF_MAC_ADDRESS].as_hex))
    cg.add(var.set_bindkey(config[CONF_BINDKEY]))
