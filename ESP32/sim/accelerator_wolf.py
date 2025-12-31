import diy_ffb_protocol_pb2 as ffb_protocol
import asyncio

async def load_config(serial, store=False):
    msg = ffb_protocol.Message()
    msg.function_config.base.function_id = ffb_protocol.FUNCTION_ID_ACCELERATOR_PEDAL
    msg.function_config.base.linked_axes.append(ffb_protocol.AXIS_ID_3)
    msg.function_config.base.linked_axes.append(ffb_protocol.AXIS_UNDEFINED)
    msg.function_config.base.linked_axes.append(ffb_protocol.AXIS_UNDEFINED)
    msg.function_config.base.linked_axes.append(ffb_protocol.AXIS_UNDEFINED)
    msg.function_config.base.store = store
    msg.function_config.base.controller_output_axis = ffb_protocol.CONTROLLER_AXIS_R_Z
    msg.function_config.base.output_mode = ffb_protocol.OUTPUT_MODE_TRAVEL
    msg.function_config.base.output_min = 34.0
    msg.function_config.base.output_max = 51.0
    
    msg.function_config.automotive_pedal.force_curve_config.pos_min = 34
    msg.function_config.automotive_pedal.force_curve_config.pos_max = 51
    msg.function_config.automotive_pedal.force_curve_config.f_min = 25.0
    msg.function_config.automotive_pedal.force_curve_config.f_max = 70.0
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(0)
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(20)
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(40)
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(60)
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(80)
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(100)
    msg.function_config.automotive_pedal.force_curve_config.force_direction = ffb_protocol.FORCE_DIRECTION_SUBTRACT
    msg.function_config.automotive_pedal.damper_config.positive_factor = 0.05
    msg.function_config.automotive_pedal.damper_config.negative_factor = 0.05
    msg.function_config.automotive_pedal.pos_idle = 34
    msg.function_config.automotive_pedal.pos_end = 51
    serial.send_message(msg)
    await asyncio.sleep(0.5)
    msg = ffb_protocol.Message()
    msg.axis_action.axis_id = ffb_protocol.AXIS_ID_3
    msg.axis_action.return_function_config = True
    serial.send_message(msg)
