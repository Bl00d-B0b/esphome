import re

from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import time, uart
from esphome.components.sensor import Sensor
from esphome.components.text_sensor import TextSensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ACTION,
    CONF_ID,
    CONF_NUMBER,
    CONF_SENSOR_DATAPOINT,
    CONF_TIME_ID,
    CONF_TRIGGER_ID,
    CONF_TX_PIN,
    CONF_UART_ID,
    CONF_UPDATE_INTERVAL,
    PLATFORM_BK72XX,
    PLATFORM_ESP32,
    PLATFORM_ESP8266,
    PLATFORM_LN882X,
    PLATFORM_RTL87XX,
)
import esphome.final_validate as fv
from esphome.types import ConfigType

DEPENDENCIES = ["uart"]

CONF_IGNORE_MCU_UPDATE_ON_DATAPOINTS = "ignore_mcu_update_on_datapoints"

CONF_ON_DATAPOINT_UPDATE = "on_datapoint_update"
CONF_DATAPOINT_TYPE = "datapoint_type"
CONF_STATUS_PIN = "status_pin"
CONF_HEARTBEAT_INTERVAL = "heartbeat_interval"
CONF_INIT_HEARTBEAT_INTERVAL = "init_heartbeat_interval"
CONF_MCU_RECOVERY = "mcu_recovery"
CONF_MISSED_HEARTBEATS = "missed_heartbeats"
CONF_BREAK_TIME = "break_time"
CONF_WEATHER = "weather"
CONF_ON_WEATHER_REQUEST = "on_weather_request"

ACTION_REBOOT = "reboot"
ACTION_BREAK = "break"
BREAK_PLATFORMS = [
    PLATFORM_BK72XX,
    PLATFORM_LN882X,
    PLATFORM_RTL87XX,
    PLATFORM_ESP8266,
    PLATFORM_ESP32,
]

tuya_ns = cg.esphome_ns.namespace("tuya")
TuyaDatapointType = tuya_ns.enum("TuyaDatapointType", is_class=True)
Tuya = tuya_ns.class_("Tuya", cg.Component, uart.UARTDevice)
TuyaWeatherRequestTrigger = tuya_ns.class_(
    "TuyaWeatherRequestTrigger",
    automation.Trigger.template(cg.std_vector.template(cg.std_string)),
)
TuyaSendWeatherAction = tuya_ns.class_("TuyaSendWeatherAction", automation.Action)

# Tuya weather keys and the type of their value on the wire (Tuya MCU SDK, Weather service, appendix 3).
# w.windSpeed is a string with one decimal ("2.7"). Forecast entries carry the day after the name
# (w.temp.1 or w.temp1) and take the base key's type.
WEATHER_INT_KEYS = {
    "w.temp",
    "w.humidity",
    "w.pressure",
    "w.realFeel",
    "w.uvi",
    "w.windLevel",
    "w.aqi",
    "w.qualityLevel",
    "w.pm10",
    "w.pm25",
    "w.o3",
    "w.no2",
    "w.co",
    "w.so2",
    "w.thigh",
    "w.tlow",
}
WEATHER_STRING_KEYS = {
    "w.conditionNum",
    "w.windSpeed",
    "w.windDir",
    "w.sunrise",
    "w.sunset",
    "w.currdate",
    "w.rank",
    "w.quality",
    "c.area",
    "c.city",
    "c.province",
}
WEATHER_KEYS = WEATHER_INT_KEYS | WEATHER_STRING_KEYS
# Forecast values go on the wire as w.<key>.<day>, day 0 = today .. 6 (Tuya MCU SDK weather_data_raw_handle)
WEATHER_FORECAST_RE = re.compile(r"^(w\.[A-Za-z]+)\.([0-6])$")


def weather_base_key(key):
    """The Tuya key without a forecast day suffix, or None for an unknown key."""
    if key in WEATHER_KEYS:
        return key
    m = WEATHER_FORECAST_RE.match(key)
    if m and m.group(1) in WEATHER_KEYS:
        return m.group(1)
    return None


def weather_key_is_string(key):
    return weather_base_key(key) in WEATHER_STRING_KEYS


def weather_key_takes_sensor(key):
    base = weather_base_key(key)
    return base == "w.windSpeed" or base in WEATHER_INT_KEYS


DPTYPE_ANY = "any"
DPTYPE_RAW = "raw"
DPTYPE_BOOL = "bool"
DPTYPE_INT = "int"
DPTYPE_UINT = "uint"
DPTYPE_STRING = "string"
DPTYPE_ENUM = "enum"
DPTYPE_BITMASK = "bitmask"

DATAPOINT_TYPES = {
    DPTYPE_ANY: tuya_ns.struct("TuyaDatapoint"),
    DPTYPE_RAW: cg.std_vector.template(cg.uint8),
    DPTYPE_BOOL: cg.bool_,
    DPTYPE_INT: cg.int_,
    DPTYPE_UINT: cg.uint32,
    DPTYPE_STRING: cg.std_string,
    DPTYPE_ENUM: cg.uint8,
    DPTYPE_BITMASK: cg.uint32,
}

DATAPOINT_TRIGGERS = {
    DPTYPE_ANY: tuya_ns.class_(
        "TuyaDatapointUpdateTrigger",
        automation.Trigger.template(DATAPOINT_TYPES[DPTYPE_ANY]),
    ),
    DPTYPE_RAW: tuya_ns.class_(
        "TuyaRawDatapointUpdateTrigger",
        automation.Trigger.template(DATAPOINT_TYPES[DPTYPE_RAW]),
    ),
    DPTYPE_BOOL: tuya_ns.class_(
        "TuyaBoolDatapointUpdateTrigger",
        automation.Trigger.template(DATAPOINT_TYPES[DPTYPE_BOOL]),
    ),
    DPTYPE_INT: tuya_ns.class_(
        "TuyaIntDatapointUpdateTrigger",
        automation.Trigger.template(DATAPOINT_TYPES[DPTYPE_INT]),
    ),
    DPTYPE_UINT: tuya_ns.class_(
        "TuyaUIntDatapointUpdateTrigger",
        automation.Trigger.template(DATAPOINT_TYPES[DPTYPE_UINT]),
    ),
    DPTYPE_STRING: tuya_ns.class_(
        "TuyaStringDatapointUpdateTrigger",
        automation.Trigger.template(DATAPOINT_TYPES[DPTYPE_STRING]),
    ),
    DPTYPE_ENUM: tuya_ns.class_(
        "TuyaEnumDatapointUpdateTrigger",
        automation.Trigger.template(DATAPOINT_TYPES[DPTYPE_ENUM]),
    ),
    DPTYPE_BITMASK: tuya_ns.class_(
        "TuyaBitmaskDatapointUpdateTrigger",
        automation.Trigger.template(DATAPOINT_TYPES[DPTYPE_BITMASK]),
    ),
}


def weather_key(value):
    value = cv.string_strict(value)
    if weather_base_key(value) is None:
        raise cv.Invalid(
            f"'{value}' is not a Tuya weather key; known keys: "
            f"{', '.join(sorted(WEATHER_KEYS))}, with an optional forecast day 0-6 (w.temp.1 = tomorrow)"
        )
    return value


def validate_weather_sources(config):
    out = {}
    sources = 0
    for key, value in config.items():
        if key in (CONF_UPDATE_INTERVAL, CONF_ON_WEATHER_REQUEST):
            out[key] = value
            continue
        try:
            weather_key(key)
            if weather_key_takes_sensor(key):
                out[key] = cv.use_id(Sensor)(value)
            else:
                out[key] = cv.use_id(TextSensor)(value)
        except cv.Invalid as err:
            err.prepend([key])
            raise
        sources += 1
    if sources == 0 and CONF_ON_WEATHER_REQUEST not in config:
        raise cv.Invalid(
            "weather needs at least one Tuya key with an entity id, or on_weather_request"
        )
    interval = out[CONF_UPDATE_INTERVAL].total_milliseconds
    if interval != cv.SCHEDULER_DONT_RUN and interval < 60000:
        raise cv.Invalid(
            "update_interval must be at least 1min, or never", [CONF_UPDATE_INTERVAL]
        )
    return out


def assign_declare_id(value):
    value = value.copy()
    value[CONF_TRIGGER_ID] = cv.declare_id(
        DATAPOINT_TRIGGERS[value[CONF_DATAPOINT_TYPE]]
    )(value[CONF_TRIGGER_ID].id)
    return value


def _validate_mcu_recovery(config: ConfigType) -> ConfigType:
    if config[CONF_ACTION] != ACTION_BREAK:
        return config
    cv.only_on(BREAK_PLATFORMS)(config)
    if CONF_TX_PIN not in config:
        raise cv.Invalid(
            f"{CONF_ACTION}: {ACTION_BREAK} requires {CONF_TX_PIN}", path=[CONF_ACTION]
        )
    return config


CONF_TUYA_ID = "tuya_id"
CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Tuya),
            cv.Optional(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
            cv.Optional(CONF_IGNORE_MCU_UPDATE_ON_DATAPOINTS): cv.ensure_list(
                cv.uint8_t
            ),
            cv.Optional(CONF_STATUS_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_HEARTBEAT_INTERVAL, default="15s"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(seconds=1), max=cv.TimePeriod(seconds=60)),
            ),
            cv.Optional(CONF_INIT_HEARTBEAT_INTERVAL, default="3s"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(seconds=1), max=cv.TimePeriod(seconds=60)),
            ),
            cv.Optional(CONF_MCU_RECOVERY): cv.All(
                cv.Schema(
                    {
                        cv.Optional(CONF_ACTION, default=ACTION_REBOOT): cv.one_of(
                            ACTION_REBOOT, ACTION_BREAK, lower=True
                        ),
                        cv.Optional(CONF_MISSED_HEARTBEATS, default=6): cv.int_range(
                            min=1, max=20
                        ),
                        cv.Optional(CONF_TX_PIN): pins.internal_gpio_output_pin_schema,
                        cv.Optional(CONF_BREAK_TIME, default="500ms"): cv.All(
                            cv.positive_time_period_milliseconds,
                            cv.Range(
                                min=cv.TimePeriod(milliseconds=50),
                                max=cv.TimePeriod(seconds=5),
                            ),
                        ),
                    }
                ),
                _validate_mcu_recovery,
            ),
            # Tuya weather service: the MCU enables it (0x20) and asks for data (0x34/0x03); the component
            # answers with the states of the entities given per Tuya key (sensor ids for numeric keys and
            # w.windSpeed, text_sensor ids for string keys), skipping keys without a value, and fires
            # on_weather_request (x = requested keys) for custom answers. update_interval (default never)
            # adds pushes on a timer while the service is enabled, as Tuya's module does every 30 min.
            cv.Optional(CONF_WEATHER): cv.All(
                cv.Schema(
                    {
                        cv.Optional(
                            CONF_UPDATE_INTERVAL, default="never"
                        ): cv.update_interval,
                        cv.Optional(
                            CONF_ON_WEATHER_REQUEST
                        ): automation.validate_automation(
                            {
                                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                                    TuyaWeatherRequestTrigger
                                ),
                            }
                        ),
                        cv.string_strict: cv.string_strict,
                    }
                ),
                validate_weather_sources,
            ),
            cv.Optional(CONF_ON_DATAPOINT_UPDATE): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        DATAPOINT_TRIGGERS[DPTYPE_ANY]
                    ),
                    cv.Required(CONF_SENSOR_DATAPOINT): cv.uint8_t,
                    cv.Optional(CONF_DATAPOINT_TYPE, default=DPTYPE_ANY): cv.one_of(
                        *DATAPOINT_TRIGGERS, lower=True
                    ),
                },
                extra_validators=assign_declare_id,
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


def _final_validate(config: ConfigType) -> ConfigType:
    recovery = config.get(CONF_MCU_RECOVERY)
    if recovery is None or recovery[CONF_ACTION] != ACTION_BREAK:
        return config
    path = [CONF_MCU_RECOVERY, CONF_TX_PIN]
    fconf = fv.full_config.get()
    bus = fconf.get_config_for_path(fconf.get_path_for_id(config[CONF_UART_ID])[:-1])
    if str(bus[CONF_ID].type) not in uart.NATIVE_UART_CLASSES:
        raise cv.Invalid(
            f"{CONF_ACTION}: {ACTION_BREAK} requires a UART bus of the microcontroller itself",
            path=[CONF_MCU_RECOVERY, CONF_ACTION],
        )
    if (
        CONF_TX_PIN not in bus
        or bus[CONF_TX_PIN][CONF_NUMBER] != recovery[CONF_TX_PIN][CONF_NUMBER]
    ):
        raise cv.Invalid(
            f"{CONF_TX_PIN} must be the {CONF_TX_PIN} of the UART bus used by tuya",
            path=path,
        )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    if CONF_TIME_ID in config:
        time_ = await cg.get_variable(config[CONF_TIME_ID])
        cg.add(var.set_time_id(time_))
    if CONF_STATUS_PIN in config:
        status_pin_ = await cg.gpio_pin_expression(config[CONF_STATUS_PIN])
        cg.add(var.set_status_pin(status_pin_))
    cg.add(
        var.set_heartbeat_interval(
            config[CONF_HEARTBEAT_INTERVAL].total_milliseconds,
            config[CONF_INIT_HEARTBEAT_INTERVAL].total_milliseconds,
        )
    )
    if (recovery := config.get(CONF_MCU_RECOVERY)) is not None:
        cg.add_define("USE_TUYA_MCU_RECOVERY")
        cg.add(var.set_mcu_recovery(recovery[CONF_MISSED_HEARTBEATS]))
        if recovery[CONF_ACTION] == ACTION_BREAK:
            cg.add_define("USE_TUYA_MCU_RECOVERY_BREAK")
            tx_pin = await cg.gpio_pin_expression(recovery[CONF_TX_PIN])
            cg.add(
                var.set_mcu_recovery_break(
                    tx_pin, recovery[CONF_BREAK_TIME].total_milliseconds
                )
            )
    if CONF_IGNORE_MCU_UPDATE_ON_DATAPOINTS in config:
        for dp in config[CONF_IGNORE_MCU_UPDATE_ON_DATAPOINTS]:
            cg.add(var.add_ignore_mcu_update_on_datapoints(dp))
    for conf in config.get(CONF_ON_DATAPOINT_UPDATE, []):
        trigger = cg.new_Pvariable(
            conf[CONF_TRIGGER_ID], var, conf[CONF_SENSOR_DATAPOINT]
        )
        await automation.build_automation(
            trigger, [(DATAPOINT_TYPES[conf[CONF_DATAPOINT_TYPE]], "x")], conf
        )
    if CONF_WEATHER in config:
        weather = config[CONF_WEATHER]
        interval = weather[CONF_UPDATE_INTERVAL].total_milliseconds
        if interval == cv.SCHEDULER_DONT_RUN:
            interval = 0
        cg.add(var.set_weather_service(interval))
        for key, value in weather.items():
            if key in (CONF_UPDATE_INTERVAL, CONF_ON_WEATHER_REQUEST):
                continue
            source = await cg.get_variable(value)
            if weather_key_takes_sensor(key):
                cg.add(var.add_weather_sensor(key, source))
            else:
                cg.add(var.add_weather_text_sensor(key, source))
        for conf in weather.get(CONF_ON_WEATHER_REQUEST, []):
            trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(
                trigger, [(cg.std_vector.template(cg.std_string), "x")], conf
            )


def validate_send_weather(config):
    # the key decides the type: string keys take a string (or lambda), the others an int32 (or lambda)
    out = {CONF_ID: config[CONF_ID]}
    for key, value in config.items():
        if key == CONF_ID:
            continue
        try:
            weather_key(key)
            if weather_key_is_string(key):
                out[key] = cv.templatable(cv.string)(value)
            else:
                out[key] = cv.templatable(cv.int_range(min=-(2**31), max=2**31 - 1))(
                    value
                )
        except cv.Invalid as err:
            err.prepend([key])
            raise
    if len(out) == 1:
        raise cv.Invalid("tuya.send_weather needs at least one Tuya key")
    return out


SEND_WEATHER_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(Tuya),
            cv.string_strict: cv.valid,
        }
    ),
    validate_send_weather,
)


@automation.register_action(
    "tuya.send_weather", TuyaSendWeatherAction, SEND_WEATHER_SCHEMA, synchronous=True
)
async def tuya_send_weather_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    for key, value in config.items():
        if key == CONF_ID:
            continue
        if weather_key_is_string(key):
            template_ = await cg.templatable(value, args, cg.std_string)
            cg.add(var.add_string(key, template_))
        else:
            template_ = await cg.templatable(value, args, cg.int32)
            cg.add(var.add_int(key, template_))
    return var
