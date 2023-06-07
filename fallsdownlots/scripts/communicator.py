"""
Adapted from the BLEAK UART service example.
"""

import typing
import traceback
import struct
import asyncio
import sys
from itertools import count, takewhile
from typing import Iterator
from cobs import cobs
import time
import os
import numpy as np
import matplotlib.pyplot as plt

os.environ["BLEAK_LOGGING"] = "1"

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
UART_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


# TIP: you can get this function and more from the ``more-itertools`` package.
def sliced(data: bytes, n: int) -> Iterator[bytes]:
    """
    Slices `data` into chunks of size `n`. The last slice may be smaller than `n`.
    """
    return takewhile(len, (data[i : i + n] for i in count(0, n)))

recv_buf = bytearray()

async def uart_terminal(get_packet_to_send, handle_control_param_update, handle_state_update):
    """
        Connects to the robot over BLE UART, handing state update (float lists)
        to `handle_state_update`, control param updates to `handle_control_param_update`,
        and polling `get_packet_to_send` for new serial packets to send over to the robot.
    """

    def match_nus_uuid(device: BLEDevice, adv: AdvertisementData):
        # This assumes that the device includes the UART service UUID in the
        # advertising data. This test may need to be adjusted depending on the
        # actual advertising data supplied by the device.
        if UART_SERVICE_UUID.lower() in adv.service_uuids:
            return True

        return False

    print("Looking for device...")
    device = await BleakScanner.find_device_by_filter(match_nus_uuid)

    if device is None:
        print("no matching device found, you may need to edit match_nus_uuid().")
        sys.exit(1)

    def handle_disconnect(_: BleakClient):
        print("Device was disconnected, goodbye.")
        # cancelling all tasks effectively ends the program
        for task in asyncio.all_tasks():
            task.cancel()

    def handle_rx(_: BleakGATTCharacteristic, data: bytearray):
        for b in data:
            if b == 0:
                decoded_data = None
                try:
                    decoded_data = cobs.decode(recv_buf)
                except cobs.DecodeError as e:
                    pass
                    #print("Could not decode: ", e)

                recv_buf.clear()
                if decoded_data:
                    handled = False

                    # State buffer with 12 floats
                    try:
                        values = struct.unpack("f" * 12, decoded_data)
                        handle_state_update(values)
                        return
                    except struct.error:
                        pass

                    # Param update string with parameter name and then float value. 
                    try: 
                        if len(decoded_data) >= 5 and len(decoded_data) <= 30:
                            num_string_bytes = len(decoded_data) - 5
                            name = decoded_data[:num_string_bytes].decode('ascii')
                            if decoded_data[num_string_bytes] != 0:
                                raise ValueError()
                            value = struct.unpack(f"f", decoded_data[-4:])[0]
                            handle_control_param_update(name, value)
                            return
                    except struct.error:
                        pass
                    except UnicodeDecodeError:
                        pass
    
                    print(f"Discarding message we couldn't handle with {len(decoded_data)} bytes:, {decoded_data}")
                    #vals = [struct.unpack('f', x)[0] for x in sliced(decoded_data, 4) if len(x) == 4]
                    #print([f"{x:.2f}" for x in vals])
            else:
                recv_buf.append(b)

    def handle_input(data: bytes) -> bytearray:
        encoded_data = bytearray(cobs.encode(data))
        encoded_data.append(0)
        return encoded_data

    print("Found device, creating client")
    async with BleakClient(device, disconnected_callback=handle_disconnect) as client:
        print("Start notify call")
        await client.start_notify(UART_TX_CHAR_UUID, handle_rx)

        print("Connected, starting main loop...")

        loop = asyncio.get_running_loop()
        nus = client.services.get_service(UART_SERVICE_UUID)
        rx_char = nus.get_characteristic(UART_RX_CHAR_UUID)

        while True:
            # This waits until you type a line and press ENTER.
            # A real terminal program might put stdin in raw mode so that things
            # like CTRL+C get passed to the remote device.
            await asyncio.sleep(0.01)
            try:
                input_data = await get_packet_to_send()
            except Exception as e:
                print("Got exception in get_packet_to_send: ", e)
                break
            if input_data is None:
                break

            if len(input_data) == 0:
                continue

            try:
                encoded_data = await loop.run_in_executor(None, handle_input, input_data)
            except Exception as e:
                traceback.print_exc()
                continue
        
            if not encoded_data:
                continue

            # Try to parse it into an array of floats. If it matches our
            # expectation, send that down.
                
            # some devices, like devices running MicroPython, expect Windows
            # line endings (uncomment line below if needed)
            # data = data.replace(b"\n", b"\r\n")

            # Writing without response requires that the data can fit in a
            # single BLE packet. We can use the max_write_without_response_size
            # property to split the data into chunks that will fit.

            print("sending cobs data:", encoded_data)
            for s in sliced(encoded_data, rx_char.max_write_without_response_size):
                await client.write_gatt_char(rx_char, s)

class StatePlotter():
    '''
        Expected data:
        pitch, dpitch, dyaw, integrated_velocity_error, integrated_yaw_error, control_l, angle_l, vel_l, control_r, angle_r, vel_r, max_motor_temp

        Plots:
            1) pitch, dpitch, dyaw
            2) l control, anngle, vel
            3) r control, angle, vel
            4) Integrators   5) Temp
    '''
    FIELDS = [
        "pitch", "dpitch", "dyaw", "int_vel_error", "int_yaw_error", "l_control", "l_angle", "l_vel", "r_control", "r_angle", "r_vel", "max temp"
    ]
    FIELDS_BY_SUBPLOT = {
        'State': ["pitch", "dpitch", "dyaw"],
        'LWheel': ['l_control', 'l_angle', 'l_vel'],
        'RWheel': ['r_control', 'r_angle', 'r_vel'],
        'Integrators': ['int_vel_error', 'int_yaw_error'],
        'Temp': ['max temp']
    }
    HISTORY_LEN = 200
    def __init__(self):
        plt.ion()

        self.fig, self.axs = plt.subplot_mosaic([['State', 'State'], ['LWheel', 'LWheel'], ['RWheel', 'RWheel'], ['Integrators', 'Temp']],
                              layout='constrained')
        self.fig.set_size_inches(12, 6)
        
        # Initial plot to get all the lines and legends drawn.
        self.lines_by_field = {}
        self.data_by_field = {}
        self.data_mutex = asyncio.Lock()
        self.t0 = time.time()
        self.ts = np.zeros(self.HISTORY_LEN)
        for subplot_name, ax in self.axs.items():
            ax.set_ylabel(subplot_name)
            assert subplot_name in self.FIELDS_BY_SUBPLOT, f"Bad subplot name {subplot_name}"
            for field_name in self.FIELDS_BY_SUBPLOT[subplot_name]:
                self.data_by_field[field_name] = np.zeros(self.HISTORY_LEN)
                self.lines_by_field[field_name] = ax.plot(self.ts, self.data_by_field[field_name], label=field_name)[0]
            ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.pause(1E-3)

    async def update(self):
        while True:
            async with self.data_mutex:
                for field_name in self.FIELDS:
                    line = self.lines_by_field[field_name]
                    line.set_xdata(self.ts)
                    line.set_ydata(self.data_by_field[field_name])
            for ax in self.axs.values():
                ax.relim()
                ax.autoscale_view(True,True,True)
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            await asyncio.sleep(0.25)

    def handle_state_update(self, values : typing.Iterable[float]):
        # Can't get the lock in this thread as this isn't an async function... ugh asyncio is confusing.
        print("Got state update: ", values)
        assert len(values) == len(self.FIELDS)
        self.ts[:] = np.roll(self.ts, -1)
        self.ts[-1] = time.time() - self.t0
        for field_name, new_value in zip(self.FIELDS, values):
            data = self.data_by_field[field_name]
            data[:] = np.roll(data, -1) # in-place modification
            data[-1] = new_value

if __name__ == "__main__":
    def handle_control_param_update(name : str, value : float):
        print(f"Got control param update: {name}, {value}")

    state_plotter = StatePlotter()
    handle_state_update = state_plotter.handle_state_update
    

    async def get_packet_to_send() -> bytes:
        '''
            Returning `None` aborts the program. Returning an empty bytes send snothing.
        '''
        input = sys.stdin.buffer.readline()
        if not input:
            print("Got nothing from input(), returning None to terminate.")
            return None
        input_data = input.decode().strip(' \r\n')
        print(input_data)
        try:
            chunks = input_data.split(" ")
            if len(chunks) not in [1, 2]:
                raise ValueError
            name = chunks[0]
            if len(chunks) > 1:
                value = float(chunks[1])
            else:
                value = None
        except ValueError:
            print("Bad input -- only input [NAME] <single float>.")
            return bytes()

        data =  bytearray(name, "ascii") + bytes([0])
        if value is not None:
            data += struct.pack("f", value)
        return data

    try:
        async def all_tasks():
            # Schedule three calls *concurrently*:
            await asyncio.gather(
                uart_terminal(get_packet_to_send, handle_control_param_update, handle_state_update),
                state_plotter.update()
            )
        asyncio.run(all_tasks())

    except asyncio.CancelledError:
        # task is cancelled on disconnect, so we ignore this error
        pass