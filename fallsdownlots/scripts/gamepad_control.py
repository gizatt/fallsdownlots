import typing
import sys
import asyncio
from communicator import uart_terminal
import time
import keyboard
import struct

if __name__ == "__main__":
    
    def handle_control_param_update(name : str, value : float):
        print(f"Got control param update: {name}, {value}")

    def handle_state_update(values : typing.Iterable[float]):
        print("Got state update: ", values)

    yaw_target_smoothed = 0.0
    vel_target_smoothed = 0.0
    
    yaw_scaling = 5.0
    vel_scaling = 5.0
    alpha = 0.25
    send_state = False


    def get_packet_to_send() -> bytes:
        '''
            Returning `None` aborts the program. Returning an empty bytes send snothing.
        '''
        global yaw_target_smoothed, vel_target_smoothed, send_state

        yaw_target = yaw_scaling * keyboard.is_pressed('right') - yaw_scaling * keyboard.is_pressed('left') 
        vel_target = vel_scaling * keyboard.is_pressed('up') - vel_scaling * keyboard.is_pressed('down') 

        yaw_target_smoothed = yaw_target_smoothed * alpha + yaw_target * (1. - alpha)
        vel_target_smoothed = vel_target_smoothed * alpha + vel_target * (1. - alpha)
        print(vel_target_smoothed, yaw_target_smoothed)
        if send_state:
            ret = bytes("VEL_T", 'ascii') + bytes([0]) + struct.pack('f', vel_target_smoothed)
        else:
            ret = bytes("DYAW_T", 'ascii') + bytes([0]) + struct.pack('f', yaw_target_smoothed)
        send_state = not send_state

        time.sleep(0.2)
        return ret

    try:
        asyncio.run(uart_terminal(get_packet_to_send, handle_control_param_update, handle_state_update))
    except asyncio.CancelledError:
        # task is cancelled on disconnect, so we ignore this error
        pass

    print("Done, tearing down")
