import diy_ffb_protocol_pb2 as ffb_protocol
import asyncio

async def load_config(serial, store=False):
    msg = ffb_protocol.Message()
    msg.function_config.base.function_id = ffb_protocol.FUNCTION_ID_BRAKE_PEDAL
    msg.function_config.base.linked_axes.append(ffb_protocol.AXIS_ID_2)
    msg.function_config.base.linked_axes.append(ffb_protocol.AXIS_UNDEFINED)
    msg.function_config.base.linked_axes.append(ffb_protocol.AXIS_UNDEFINED)
    msg.function_config.base.linked_axes.append(ffb_protocol.AXIS_UNDEFINED)
    msg.function_config.base.store = store
    msg.function_config.base.controller_output_axis = ffb_protocol.CONTROLLER_AXIS_R_X
    msg.function_config.base.output_mode = ffb_protocol.OUTPUT_MODE_FORCE
    msg.function_config.base.output_min = 30.0
    msg.function_config.base.output_max = 150.0
    
    msg.function_config.automotive_pedal.force_curve_config.pos_min = -14
    msg.function_config.automotive_pedal.force_curve_config.pos_max = 25
    msg.function_config.automotive_pedal.force_curve_config.f_min = 30.0
    msg.function_config.automotive_pedal.force_curve_config.f_max = 150.0
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(0)
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(20)
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(40)
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(60)
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(80)
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(100)
    msg.function_config.automotive_pedal.force_curve_config.force_direction = ffb_protocol.FORCE_DIRECTION_SUBTRACT
    msg.function_config.automotive_pedal.damper_config.positive_factor = 0.1
    msg.function_config.automotive_pedal.damper_config.negative_factor = 0.1
    msg.function_config.automotive_pedal.pos_idle = -14
    msg.function_config.automotive_pedal.pos_end = 25
    msg.function_config.automotive_pedal.abs_effect_config.freq = 15
    msg.function_config.automotive_pedal.abs_effect_config.ampl = 20
    msg.function_config.automotive_pedal.abs_effect_config.pattern = ffb_protocol.ABS_PATTERN_SAWTOOTH
    msg.function_config.automotive_pedal.abs_effect_config.mode = ffb_protocol.ABS_MODE_FORCE
    serial.send_message(msg)
    await asyncio.sleep(0.5)
    msg = ffb_protocol.Message()
    msg.axis_action.axis_id = ffb_protocol.AXIS_ID_2
    msg.axis_action.return_function_config = True
    serial.send_message(msg)
