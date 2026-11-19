import re

from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import time, uart
from esphome.components.sensor import Sensor
from esphome.components.text_sensor import TextSensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ACTION,
    CONF_HEARTBEAT,
    CONF_ID,
    CONF_SENSOR_DATAPOINT,
    CONF_TIME_ID,
    CONF_UPDATE_INTERVAL,
)
from esphome.core import ID
from esphome.cpp_generator import MockObj
from esphome.types import ConfigType, TemplateArgsType

DEPENDENCIES = ["uart"]
DOMAIN = "tuya"

CONF_IGNORE_MCU_UPDATE_ON_DATAPOINTS = "ignore_mcu_update_on_datapoints"

CONF_ON_DATAPOINT_UPDATE = "on_datapoint_update"
CONF_DATAPOINT_TYPE = "datapoint_type"
CONF_STATUS_PIN = "status_pin"
CONF_FORCE_TIME_SYNC = "force_time_sync"
CONF_MCU_WATCHDOG = "mcu_watchdog"
CONF_MISSED_HEARTBEATS = "missed_heartbeats"
CONF_DATAPOINT_ACK = "datapoint_ack"

CONF_RESET_TIME = "reset_time"
ACTION_REBOOT = "reboot"
ACTION_RESET = "reset"
ACTION_DISABLE = "disable"
CONF_WEATHER = "weather"
CONF_ON_WEATHER_REQUEST = "on_weather_request"

tuya_ns = cg.esphome_ns.namespace("tuya")
TuyaDatapointType = tuya_ns.enum("TuyaDatapointType", is_class=True)
Tuya = tuya_ns.class_("Tuya", cg.Component, uart.UARTDevice)

TuyaDatapoint = tuya_ns.struct("TuyaDatapoint")

DPTYPE_ANY = "any"
DPTYPE_RAW = "raw"
DPTYPE_BOOL = "bool"
DPTYPE_INT = "int"
DPTYPE_UINT = "uint"
DPTYPE_STRING = "string"
DPTYPE_ENUM = "enum"
DPTYPE_BITMASK = "bitmask"

# Automation argument type, plus the expected TuyaDatapointType and the field forwarded
DATAPOINT_TYPES = {
    DPTYPE_ANY: (TuyaDatapoint, None, None),
    DPTYPE_RAW: (cg.std_vector.template(cg.uint8), TuyaDatapointType.RAW, "value_raw"),
    DPTYPE_BOOL: (cg.bool_, TuyaDatapointType.BOOLEAN, "value_bool"),
    DPTYPE_INT: (cg.int_, TuyaDatapointType.INTEGER, "value_int"),
    DPTYPE_UINT: (cg.uint32, TuyaDatapointType.INTEGER, "value_uint"),
    DPTYPE_STRING: (cg.std_string, TuyaDatapointType.STRING, "value_string"),
    DPTYPE_ENUM: (cg.uint8, TuyaDatapointType.ENUM, "value_enum"),
    DPTYPE_BITMASK: (cg.uint32, TuyaDatapointType.BITMASK, "value_bitmask"),
}

TuyaSendWeatherAction = tuya_ns.class_("TuyaSendWeatherAction", automation.Action)

# Tuya weather keys by the type of their value on the wire
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
    "w.condition",
    "w.tips",
    "w.windSpeed",
    "w.windDir",
    "w.sunrise",
    "w.sunset",
    "w.sunRise",
    "w.sunSet",
    "w.currdate",
    "w.rank",
    "w.quality",
    "c.area",
    "c.city",
    "c.province",
}
WEATHER_KEYS = WEATHER_INT_KEYS | WEATHER_STRING_KEYS
# Forecast values are sent as w.<key>.<day>, day 0 = today
WEATHER_FORECAST_RE = re.compile(r"^(w\.[A-Za-z0-9]+)\.([0-6])$")
# Every key a config can hold: the keys and the forecast keys of the w. ones
WEATHER_CONFIG_KEYS = sorted(WEATHER_KEYS) + [
    f"{key}.{day}"
    for key in sorted(WEATHER_KEYS)
    if key.startswith("w.")
    for day in range(7)
]
WEATHER_INT_RANGE = cv.int_range(min=-(2**31), max=2**31 - 1)


def weather_base_key(key: str) -> str | None:
    """The Tuya key without a forecast day suffix, or None for an unknown key."""
    if key in WEATHER_KEYS:
        return key
    m = WEATHER_FORECAST_RE.match(key)
    if m and m.group(1) in WEATHER_KEYS:
        return m.group(1)
    return None


def weather_key_is_string(key: str) -> bool:
    return weather_base_key(key) in WEATHER_STRING_KEYS


def weather_key_takes_sensor(key: str) -> bool:
    base = weather_base_key(key)
    return base == "w.windSpeed" or base in WEATHER_INT_KEYS


CONF_TUYA_ID = "tuya_id"


def _validate_time_options(config: ConfigType) -> ConfigType:
    if config[CONF_FORCE_TIME_SYNC] and CONF_TIME_ID not in config:
        raise cv.Invalid(
            f"{CONF_FORCE_TIME_SYNC} requires {CONF_TIME_ID}",
            path=[CONF_FORCE_TIME_SYNC],
        )
    return config


def _validate_heartbeat(config: ConfigType) -> ConfigType:
    # runs before the defaults: only a watchdog set in the config is rejected
    if not isinstance(config, dict):
        return config  # the schema reports the wrong type
    try:
        if cv.boolean(config.get(CONF_HEARTBEAT, True)):
            return config
    except cv.Invalid:
        return config  # the schema reports it at the key
    watchdog = config.get(CONF_MCU_WATCHDOG) or {}
    if not isinstance(watchdog, dict):
        return config
    action = str(watchdog.get(CONF_ACTION, "")).lower()
    if CONF_MCU_WATCHDOG in config and action != ACTION_DISABLE:
        raise cv.Invalid(
            f"{CONF_MCU_WATCHDOG} requires {CONF_HEARTBEAT}",
            path=[CONF_MCU_WATCHDOG],
        )
    return config


_MISSED_HEARTBEATS_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_MISSED_HEARTBEATS, default=6): cv.int_range(min=1, max=20),
    }
)

# the options of each action: one without effect is rejected
MCU_WATCHDOG_SCHEMA = cv.typed_schema(
    {
        ACTION_REBOOT: _MISSED_HEARTBEATS_SCHEMA,
        # holds TX low for reset_time, which restarts the MCU's UART
        ACTION_RESET: _MISSED_HEARTBEATS_SCHEMA.extend(
            {
                cv.Optional(CONF_RESET_TIME, default="500ms"): cv.All(
                    cv.positive_time_period_milliseconds,
                    cv.Range(
                        min=cv.TimePeriod(milliseconds=50),
                        max=cv.TimePeriod(seconds=5),
                    ),
                ),
            }
        ),
        ACTION_DISABLE: cv.Schema({}),
    },
    key=CONF_ACTION,
    default_type=ACTION_REBOOT,
    lower=True,
)


def weather_key(value: str) -> str:
    value = cv.string_strict(value)
    if weather_base_key(value) is None:
        raise cv.Invalid(
            f"'{value}' is not a Tuya weather key; known keys: "
            f"{', '.join(sorted(WEATHER_KEYS))}, with an optional forecast day 0-6 (w.temp.1 = tomorrow)"
        )
    return value


def validate_weather_sources(config: ConfigType) -> ConfigType:
    out = {}
    sources = 0
    for key, value in config.items():
        if key in (CONF_UPDATE_INTERVAL, CONF_ON_WEATHER_REQUEST):
            out[key] = value
            continue
        # the schema has validated the value of every known key
        try:
            weather_key(key)
        except cv.Invalid as err:
            err.prepend([key])
            raise
        out[key] = value
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


CONFIG_SCHEMA = cv.All(
    _validate_heartbeat,
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Tuya),
            cv.Optional(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
            cv.Optional(CONF_FORCE_TIME_SYNC, default=False): cv.boolean,
            cv.Optional(CONF_IGNORE_MCU_UPDATE_ON_DATAPOINTS): cv.ensure_list(
                cv.uint8_t
            ),
            cv.Optional(CONF_STATUS_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_HEARTBEAT, default=True): cv.boolean,
            cv.Optional(CONF_DATAPOINT_ACK, default=False): cv.boolean,
            cv.Optional(CONF_MCU_WATCHDOG, default={}): MCU_WATCHDOG_SCHEMA,
            cv.Optional(CONF_ON_DATAPOINT_UPDATE): automation.validate_automation(
                {
                    cv.Required(CONF_SENSOR_DATAPOINT): cv.uint8_t,
                    cv.Optional(CONF_DATAPOINT_TYPE, default=DPTYPE_ANY): cv.one_of(
                        *DATAPOINT_TYPES, lower=True
                    ),
                }
            ),
            cv.Optional(CONF_WEATHER): cv.All(
                cv.Schema(
                    {
                        cv.Optional(
                            CONF_UPDATE_INTERVAL, default="30min"
                        ): cv.update_interval,
                        cv.Optional(
                            CONF_ON_WEATHER_REQUEST
                        ): automation.validate_synchronous_automation(
                            CONF_ON_WEATHER_REQUEST, "the answer to the MCU"
                        ),
                        **{
                            cv.Optional(key): cv.use_id(
                                Sensor if weather_key_takes_sensor(key) else TextSensor
                            )
                            for key in WEATHER_CONFIG_KEYS
                        },
                    },
                    # an unknown key gets the message of validate_weather_sources
                    extra=cv.ALLOW_EXTRA,
                ),
                validate_weather_sources,
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA),
    _validate_time_options,
)


async def to_code(config):
    var = await uart.new_uart_component(config)
    if (time_id := config.get(CONF_TIME_ID)) is not None:
        await cg.add_variable(var.set_time_id, time_id)
        if config[CONF_FORCE_TIME_SYNC]:
            cg.add(var.set_force_time_sync(True))
    await cg.add_pin(var.set_status_pin, config.get(CONF_STATUS_PIN))
    if not config[CONF_HEARTBEAT]:
        cg.add(var.set_heartbeat(False))
    if config[CONF_DATAPOINT_ACK]:
        cg.add(var.set_datapoint_ack(True))
    watchdog = config[CONF_MCU_WATCHDOG]
    if config[CONF_HEARTBEAT] and watchdog[CONF_ACTION] != ACTION_DISABLE:
        cg.add_define("USE_TUYA_MCU_WATCHDOG")
        cg.add(var.set_mcu_watchdog(watchdog[CONF_MISSED_HEARTBEATS]))
        if watchdog[CONF_ACTION] == ACTION_RESET:
            cg.add_define("USE_TUYA_MCU_WATCHDOG_RESET")
            uart.request_tx_break()
            cg.add(
                var.set_mcu_watchdog_reset(watchdog[CONF_RESET_TIME].total_milliseconds)
            )
    if CONF_IGNORE_MCU_UPDATE_ON_DATAPOINTS in config:
        for dp in config[CONF_IGNORE_MCU_UPDATE_ON_DATAPOINTS]:
            cg.add(var.add_ignore_mcu_update_on_datapoints(dp))
    for conf in config.get(CONF_ON_DATAPOINT_UPDATE, []):
        type_, expected, field = DATAPOINT_TYPES[conf[CONF_DATAPOINT_TYPE]]
        forward = None
        if expected is not None:
            forward = [getattr(MockObj("x", ".").expect_type(expected), field)]
        callback = await automation.build_trigger_callback(
            [(type_, "x")], conf, params=[(TuyaDatapoint, "x")], forward=forward
        )
        cg.add(var.register_listener(conf[CONF_SENSOR_DATAPOINT], callback))
    if (weather := config.get(CONF_WEATHER)) is not None:
        cg.add_define("USE_TUYA_WEATHER")
        interval = weather[CONF_UPDATE_INTERVAL].total_milliseconds
        if interval == cv.SCHEDULER_DONT_RUN:
            interval = 0
        cg.add(var.set_weather_update_interval(interval))
        sources = [
            key
            for key in weather
            if key not in (CONF_UPDATE_INTERVAL, CONF_ON_WEATHER_REQUEST)
        ]
        sensors = sum(1 for key in sources if weather_key_takes_sensor(key))
        cg.add(var.init_weather_sources(sensors, len(sources) - sensors))
        for key in sources:
            value = weather[key]
            source = await cg.get_variable(value)
            if weather_key_takes_sensor(key):
                cg.add(var.add_weather_sensor(key, source))
            else:
                cg.add(var.add_weather_text_sensor(key, source))
        for conf in weather.get(CONF_ON_WEATHER_REQUEST, []):
            await automation.build_callback_automation(
                var,
                "add_on_weather_request_callback",
                [(cg.std_vector.template(cg.std_string), "x")],
                conf,
            )


def validate_send_weather(config: ConfigType) -> ConfigType:
    # The key decides the type: string keys take a string, the others an int32
    out = {CONF_ID: config[CONF_ID]}
    for key, value in config.items():
        if key == CONF_ID:
            continue
        # the schema has validated the value of every known key
        try:
            weather_key(key)
        except cv.Invalid as err:
            err.prepend([key])
            raise
        out[key] = value
    if len(out) == 1:
        raise cv.Invalid("tuya.send_weather needs at least one Tuya key")
    return out


SEND_WEATHER_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(Tuya),
            # the key decides the type
            **{
                cv.Optional(key): cv.templatable(
                    cv.string if weather_key_is_string(key) else WEATHER_INT_RANGE
                )
                for key in WEATHER_CONFIG_KEYS
            },
        },
        # an unknown key gets the message of validate_send_weather
        extra=cv.ALLOW_EXTRA,
    ),
    validate_send_weather,
)


@automation.register_action(
    "tuya.send_weather", TuyaSendWeatherAction, SEND_WEATHER_SCHEMA, synchronous=True
)
async def tuya_send_weather_to_code(
    config: ConfigType,
    action_id: ID,
    template_arg: cg.TemplateArguments,
    args: TemplateArgsType,
) -> MockObj:
    cg.add_define("USE_TUYA_WEATHER")
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent, len(config) - 1)
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
