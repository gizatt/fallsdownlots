"""
Adapted from the BLEAK UART service example.
"""

import traceback
import struct
import asyncio
import sys
from itertools import count, takewhile
from typing import Iterator
from cobs import cobs

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
    Slices *data* into chunks of size *n*. The last slice may be smaller than
    *n*.
    """
    return takewhile(len, (data[i : i + n] for i in count(0, n)))


recv_buf = bytearray()

async def uart_terminal():
    """This is a simple "terminal" program that uses the Nordic Semiconductor
    (nRF) UART service. It reads from stdin and sends each line of data to the
    remote device. Any data received from the device is printed to stdout.
    """

    def match_nus_uuid(device: BLEDevice, adv: AdvertisementData):
        # This assumes that the device includes the UART service UUID in the
        # advertising data. This test may need to be adjusted depending on the
        # actual advertising data supplied by the device.
        if UART_SERVICE_UUID.lower() in adv.service_uuids:
            return True

        return False

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
                    print("Could not decode: ", e)
                recv_buf.clear()
                if decoded_data:
                    vals = [struct.unpack('f', x)[0] for x in sliced(decoded_data, 4)]
                    print([f"{x:.2f}" for x in vals])
            else:
                recv_buf.append(b)

    def handle_input(input_data: str) -> bytearray:
        input_data = input_data.decode().strip(' \r\n')
        print(input_data, type(input_data))
        try:
            floats = [float(x) for x in input_data.split(" ")]
        except ValueError:
            print("Bad input -- only input space-separated floats.")
            return None

        # Encode to a COBS buffer
        data = struct.pack("f"*len(floats), *floats)
        encoded_data = bytearray(cobs.encode(data))
        encoded_data.append(0)
        return encoded_data

    async with BleakClient(device, disconnected_callback=handle_disconnect) as client:
        await client.start_notify(UART_TX_CHAR_UUID, handle_rx)

        print("Connected, start typing and press ENTER...")

        loop = asyncio.get_running_loop()
        nus = client.services.get_service(UART_SERVICE_UUID)
        rx_char = nus.get_characteristic(UART_RX_CHAR_UUID)

        while True:
            # This waits until you type a line and press ENTER.
            # A real terminal program might put stdin in raw mode so that things
            # like CTRL+C get passed to the remote device.
            input_data = await loop.run_in_executor(None, sys.stdin.buffer.readline)

            # data will be empty on EOF (e.g. CTRL+D on *nix)
            if not input_data:
                break

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

            print("sending:", encoded_data)
            for s in sliced(encoded_data, rx_char.max_write_without_response_size):
                await client.write_gatt_char(rx_char, s)

            
            
if __name__ == "__main__":
    try:
        asyncio.run(uart_terminal())
    except asyncio.CancelledError:
        # task is cancelled on disconnect, so we ignore this error
        pass