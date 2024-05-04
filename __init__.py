import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import core
from esphome.components import sensor

CODEOWNERS = ["@dhahaj"]

AUTO_LOAD = ["sensor"]

# This should match the class name in your C++ header
AcuriteSensor = cg.global_ns.class_('AcuriteSensor', cg.Component)

CONFIG_SCHEMA = sensor.sensor_schema(AcuriteSensor).extend({
    # Here you can define configuration parameters that are read from YAML
    # For example, if your sensor supports configuring a measurement interval
    cg.Optional('measurement_interval'): cv.positive_time_period_milliseconds,
}).extend(cg.polling_component_schema('60s'))  # Default polling interval

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    if 'measurement_interval' in config:
        cg.add(var.set_measurement_interval(config['measurement_interval']))
    yield cg.register_component(var, config)
    yield sensor.register_sensor(var, config)


# import esphome.codegen as cg
# from esphome.components import ble_client
# import esphome.config_validation as cv
# from esphome.const import CONF_ID, CONF_THROTTLE

# CODEOWNERS = ["@dhahaj"]

# AUTO_LOAD = ["sensor"]
# MULTI_CONF = True

# CONF_HELTEC_BALANCER_BLE_ID = "heltec_balancer_ble_id"

# heltec_balancer_ble_ns = cg.esphome_ns.namespace("heltec_balancer_ble")
# HeltecBalancerBle = heltec_balancer_ble_ns.class_(
#     "HeltecBalancerBle", ble_client.BLEClientNode, cg.PollingComponent
# )

# HELTEC_BALANCER_BLE_COMPONENT_SCHEMA = cv.Schema(
#     {
#         cv.GenerateID(CONF_HELTEC_BALANCER_BLE_ID): cv.use_id(HeltecBalancerBle),
#     }
# )

# CONFIG_SCHEMA = (
#     cv.Schema(
#         {
#             cv.GenerateID(): cv.declare_id(HeltecBalancerBle),
#             cv.Optional(
#                 CONF_THROTTLE, default="2s"
#             ): cv.positive_time_period_milliseconds,
#         }
#     )
#     .extend(ble_client.BLE_CLIENT_SCHEMA)
#     .extend(cv.polling_component_schema("5s"))
# )


# async def to_code(config):
#     var = cg.new_Pvariable(config[CONF_ID])
#     await cg.register_component(var, config)
#     await ble_client.register_ble_node(var, config)

#     cg.add(var.set_throttle(config[CONF_THROTTLE]))