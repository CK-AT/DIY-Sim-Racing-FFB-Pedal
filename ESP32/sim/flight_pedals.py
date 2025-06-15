import diy_ffb_protocol_pb2 as ffb_protocol
import asyncio

async def load_config(serial, store=False):
    msg = ffb_protocol.Message()
    msg.function_config.base.function_id = ffb_protocol.FUNCTION_ID_FLIGHT_PEDALS
    msg.function_config.base.linked_axes.append(ffb_protocol.AXIS_ID_3)
    msg.function_config.base.linked_axes.append(ffb_protocol.AXIS_ID_2 + ffb_protocol.AXIS_SUBTRACTIVE)
    msg.function_config.base.linked_axes.append(ffb_protocol.AXIS_UNDEFINED)
    msg.function_config.base.linked_axes.append(ffb_protocol.AXIS_UNDEFINED)
    msg.function_config.base.store = store
    msg.function_config.base.controller_output_axis = ffb_protocol.CONTROLLER_AXIS_X
    msg.function_config.base.output_mode = ffb_protocol.OUTPUT_MODE_TRAVEL
    msg.function_config.base.output_min = 30.0
    msg.function_config.base.output_max = 70.0
    
    msg.function_config.flight_pedals.pos_near_lim = 30
    msg.function_config.flight_pedals.pos_far_lim = 70
    msg.function_config.flight_pedals.damping = 0.15
    msg.function_config.flight_pedals.centering_spring_const = 1.5
    serial.send_message(msg)
    await asyncio.sleep(0.5)
    msg = ffb_protocol.Message()
    msg.axis_action.axis_id = ffb_protocol.AXIS_ID_2
    msg.axis_action.return_function_config = True
    serial.send_message(msg)
