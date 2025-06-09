import asyncio
import serial_asyncio
from cobs import cobs
import modbus_crc
import diy_ffb_protocol_pb2 as ffb_protocol
import brake_wolf
import accelerator_wolf
import flight_pedals
import brake_alerio
import accelerator_alerio

class OutputProtocol(asyncio.Protocol):
    def __init__(self):
        super().__init__()
        self.current_frame = bytearray()
        self.msg_queue = asyncio.Queue()

    def connection_made(self, transport):
        self.transport = transport
        print('port opened', transport)
        transport.write(b'\x00\x00\x00')

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
    await asyncio.sleep(1)
    # msg = ffb_protocol.Message()
    # msg.axis_action.axis_id = ffb_protocol.AXIS_3
    # msg.axis_action.return_axis_config = True
    # protocol.send_message(msg)
    # await asyncio.sleep(0.2)
    # msg = ffb_protocol.Message()
    # msg.axis_action.axis_id = ffb_protocol.AXIS_3
    # msg.axis_action.return_function_config = True
    # protocol.send_message(msg)
    # await asyncio.sleep(0.2)
    # while True:
    #     await asyncio.sleep(0.1)
    #     msg = ffb_protocol.Message()
    #     msg.ffb_action.function_id = ffb_protocol.FUNCTION_BRAKE
    #     msg.ffb_action.automotive_pedal.trigger_abs = True
    #     protocol.send_message(msg)
    # msg = ffb_protocol.Message()
    # msg.axis_action.axis_id = ffb_protocol.AXIS_2
    # msg.axis_action.restart = True
    # protocol.send_message(msg)
    # await asyncio.sleep(0.5)
    # msg = ffb_protocol.Message()
    # msg.axis_action.axis_id = ffb_protocol.AXIS_3
    # msg.axis_action.restart = True
    # protocol.send_message(msg)
    # await accelerator_wolf.load_config(protocol)
    # await asyncio.sleep(0.2)
    # await brake_wolf.load_config(protocol)
    await flight_pedals.load_config(protocol)
    # await accelerator_alerio.load_config(protocol)
    # await asyncio.sleep(0.2)
    # await brake_alerio.load_config(protocol)

async def main(port):
    transport, protocol = await serial_asyncio.create_serial_connection(loop, OutputProtocol, port, baudrate=3000000)
    msg_sent = True
    asyncio.create_task(request_configs(protocol))
    async for msg in protocol.get_messages():
        payload_type = msg.WhichOneof("payload")
        if payload_type == 'axis_log_message':
            log_msg = msg.axis_log_message
            print(f'{ffb_protocol.AxisID.Name(log_msg.axis_id)} : {log_msg.msg.rstrip()}')
        elif payload_type == 'gateway_log_message':
            log_msg = msg.gateway_log_message
            print(f'{ffb_protocol.GatewayID.Name(log_msg.gateway_id)} : {log_msg.msg.rstrip()}')
        elif payload_type == 'axis_state':
            pass
        elif payload_type == 'gateway_state':
            pass
        else:
            print(f'unhandled message received: {msg}')


if __name__ == "__main__":
    import sys
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main(sys.argv[1]))
    loop.close()