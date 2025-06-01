import asyncio
import serial_asyncio
from cobs import cobs
import modbus_crc
import ffb_data_types_pb2 as ffb_data

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
                    msg = ffb_data.FFBData().FromString(decoded[:-2])
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
    # msg = ffb_data.FFBData()
    # msg.axis_action.return_axis_config = True
    # protocol.send_message(msg)
    # msg = ffb_data.FFBData()
    # msg.axis_action.return_function_config = True
    # protocol.send_message(msg)
    msg = ffb_data.FFBData()
    msg.function_config.base.function = ffb_data.FUNCTION_BRAKE
    msg.function_config.base.additive_axis_id_1 = ffb_data.AXIS_2
    msg.function_config.base.store = False
    msg.function_config.base.controller_axis = ffb_data.CONTROLLER_AXIS_BRK
    msg.function_config.automotive_pedal.force_curve_config.pos_min = 0
    msg.function_config.automotive_pedal.force_curve_config.pos_max = 60
    msg.function_config.automotive_pedal.force_curve_config.f_min = 70.0
    msg.function_config.automotive_pedal.force_curve_config.f_max = 120.0
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(0)
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(20)
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(40)
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(60)
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(80)
    msg.function_config.automotive_pedal.force_curve_config.f_rel_points.append(100)
    msg.function_config.automotive_pedal.force_curve_config.force_direction = ffb_data.DIRECTION_SUBRTACT
    msg.function_config.automotive_pedal.damper_config.positive_factor = 0.1
    msg.function_config.automotive_pedal.damper_config.negative_factor = 0.1
    msg.function_config.automotive_pedal.pos_idle = 0
    msg.function_config.automotive_pedal.pos_end = 60
    msg.function_config.automotive_pedal.output_mode = ffb_data.OUTPUT_MODE_FORCE
    protocol.send_message(msg)
    await asyncio.sleep(3)
    msg = ffb_data.FFBData()
    msg.axis_action.return_function_config = True
    protocol.send_message(msg)

async def main():
    transport, protocol = await serial_asyncio.create_serial_connection(loop, OutputProtocol, '/dev/ttyACM0', baudrate=921600)
    msg_sent = True
    asyncio.create_task(request_configs(protocol))
    async for msg in protocol.get_messages():
        payload_type = msg.WhichOneof("payload")
        if payload_type == 'log_message':
            print(f'Axis {msg.log_message.axis_id} : {msg.log_message.msg.rstrip()}')
            if not msg_sent:
                msg = ffb_data.FFBData()
                msg.axis_action.return_axis_config = True
                protocol.send_message(msg)
                msg = ffb_data.FFBData()
                msg.axis_action.return_function_config = True
                protocol.send_message(msg)
                msg_sent = True
        else:
            print(f'unhandled message received: {msg}')


loop = asyncio.get_event_loop()
loop.run_until_complete(main())
loop.close()