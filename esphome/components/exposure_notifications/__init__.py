from esphome import automation
import esphome.codegen as cg
from esphome.components import ble_device_base
import esphome.config_validation as cv
from esphome.const import CONF_TRIGGER_ID

CODEOWNERS = ["@OttoWinter"]
AUTO_LOAD = ["ble_device_base"]

exposure_notifications_ns = cg.esphome_ns.namespace("exposure_notifications")
ExposureNotification = exposure_notifications_ns.struct("ExposureNotification")
ExposureNotificationTrigger = exposure_notifications_ns.class_(
    "ExposureNotificationTrigger",
    ble_device_base.ESPBTDeviceListener,
    automation.Trigger.template(ExposureNotification),
)

CONF_ON_EXPOSURE_NOTIFICATION = "on_exposure_notification"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ON_EXPOSURE_NOTIFICATION): automation.validate_automation(
            cv.Schema(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        ExposureNotificationTrigger
                    ),
                }
            ),
            # the trigger is the BLE listener, so the hub id lives on it
            extra_validators=ble_device_base.inject_ble_hub,
        ),
    }
)


async def to_code(config):
    for conf in config.get(CONF_ON_EXPOSURE_NOTIFICATION, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        await automation.build_automation(trigger, [(ExposureNotification, "x")], conf)
        await ble_device_base.register_ble_device(trigger, conf)
