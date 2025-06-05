import asyncio
import serial_asyncio
from cobs import cobs
import modbus_crc
import diy_ffb_protocol_pb2 as ffb_protocol

class OutputProtocol(asyncio.Protocol):
    def __init__(self):
        super().__init__()
        self.current_frame = bytearray()
        self.msg_queue = asyncio.Queue()

    def connection_made(self, transport):
        self.transport = transport
        print('port opened', transport)

    def data_received(self, data):
        last_idx = 0
        idx = data.find(b'\x00', last_idx)
        while (idx >= 0):
            self.current_frame.extend(data[last_idx:idx])
            self.parse_frame(self.current_frame)
            last_idx = idx + 1
            self.current_frame = bytearray()
            idx = data.find(b'\x00', last_idx)
        self.current_frame.extend(data[last_idx:])

    def parse_frame(self, frame):
        try:
            decoded = cobs.decode(frame)
            if modbus_crc.check_crc(decoded):
                try:
                    msg = ffb_protocol.Message().FromString(decoded[:-2])
                    self.msg_queue.put_nowait(msg)
                except:
                    print(f'unknown frame received: {decoded[:-2].hex(' ')}')
        except cobs.DecodeError:
            pass

    def connection_lost(self, exc):
        print('port closed')
        self.transport.loop.stop()

    def pause_writing(self):
        print('pause writing')
        print(self.transport.get_write_buffer_size())

    def resume_writing(self):
        print(self.transport.get_write_buffer_size())
        print('resume writing')
    
    async def get_messages(self):
        while True:
            yield await self.msg_queue.get()

    def send_message(self, message):
        data = message.SerializeToString()
        data = modbus_crc.add_crc(data)
        data = cobs.encode(data)
        data += b'\x00'
        self.transport.write(data)

async def request_configs(protocol):
    await asyncio.sleep(2)
    msg = ffb_protocol.Message()
    msg.axis_action.return_axis_config = True
    protocol.send_message(msg)
    msg = ffb_protocol.Message()
    msg.axis_action.return_function_config = True
    protocol.send_message(msg)
    await asyncio.sleep(3)
    msg = ffb_protocol.Message()
    msg.function_config.base.function = ffb_protocol.FUNCTION_BRAKE
    msg.function_config.base.linked_axes.append(ffb_protocol.AXIS_2)
    msg.function_config.base.linked_axes.append(ffb_protocol.AXIS_UNDEFINED)
    msg.function_config.base.linked_axes.append(ffb_protocol.AXIS_UNDEFINED)
    msg.function_config.base.linked_axes.append(ffb_protocol.AXIS_UNDEFINED)
    msg.function_config.base.store = False
    msg.function_config.base.controller_output_axis = ffb_protocol.CONTROLLER_AXIS_BRK
    msg.function_config.base.output_mode = ffb_protocol.OUTPUT_MODE_FORCE
    msg.function_config.base.output_min = 75.0
    msg.function_config.base.output_max = 115.0
    
    msg.function_config.automotive_pedal.force_curve_config.pos_min = 10
    msg.function_config.automotive_pedal.force_curve_config.pos_max = 60
    msg.function_config.automotive_pedal.force_curve_config.f_min = 70.0
    msg.function_config.automotive_pedal.force_curve_config.f_max = 120.0
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(0)
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(20)
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(40)
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(60)
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(80)
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(100)
    msg.function_config.automotive_pedal.force_curve_config.force_direction = ffb_protocol.DIRECTION_SUBTRACT
    msg.function_config.automotive_pedal.damper_config.positive_factor = 0.1
    msg.function_config.automotive_pedal.damper_config.negative_factor = 0.1
    msg.function_config.automotive_pedal.pos_idle = 10
    msg.function_config.automotive_pedal.pos_end = 60
    protocol.send_message(msg)
    await asyncio.sleep(3)
    msg = ffb_protocol.Message()
    msg.axis_action.return_function_config = True
    protocol.send_message(msg)

async def main():
    transport, protocol = await serial_asyncio.create_serial_connection(loop, OutputProtocol, 'COM4', baudrate=921600)
    msg_sent = True
    asyncio.create_task(request_configs(protocol))
    async for msg in protocol.get_messages():
        payload_type = msg.WhichOneof("payload")
        if payload_type == 'axis_log_message':
            log_msg = msg.axis_log_message
            print(f'{ffb_protocol.AxisID.Name(log_msg.axis_id)} : {log_msg.msg.rstrip()}')
            if not msg_sent:
                msg = ffb_protocol.Message()
                msg.axis_action.return_axis_config = True
                protocol.send_message(msg)
                msg = ffb_protocol.Message()
                msg.axis_action.return_function_config = True
                protocol.send_message(msg)
                msg_sent = True
        elif payload_type == 'axis_state':
            pass
        else:
            print(f'unhandled message received: {msg}')


loop = asyncio.get_event_loop()
loop.run_until_complete(main())
loop.close()