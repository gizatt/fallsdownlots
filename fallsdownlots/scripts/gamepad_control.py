import typing
import sys
import asyncio
from communicator import uart_terminal, StatePlotter
import time
import keyboard
import struct

if __name__ == "__main__":
    
    def handle_control_param_update(name : str, value : float):
        print(f"Got control param update: {name}, {value}")

    state_plotter = StatePlotter()
    handle_state_update = state_plotter.handle_state_update

    yaw_target_smoothed = 0.0
    vel_target_smoothed = 0.0
    
    yaw_scaling = 5.0
    vel_scaling = 5.0
    alpha = 0.6
    send_state = False

    # Set to False to have state plotting enabled, but it'll possibly crash the robot with too much message traffic.
    requested_state_updates = True


    async def get_packet_to_send() -> bytes:
        '''
            Returning `None` aborts the program. Returning an empty bytes send snothing.
        '''
        global yaw_target_smoothed, vel_target_smoothed, send_state, requested_state_updates

        await asyncio.sleep(0.1)

        if not requested_state_updates:
            requested_state_updates = True
            return bytes("SEND_STATE", "ascii") + bytes([0]) + struct.pack('f', 1.)
    


        if keyboard.is_pressed('shift'):
            scaling = 2.0
        else:
            scaling = 1.0

        yaw_target = scaling * (yaw_scaling * keyboard.is_pressed('right') - yaw_scaling * keyboard.is_pressed('left'))
        vel_target = scaling * (vel_scaling * keyboard.is_pressed('up') - vel_scaling * keyboard.is_pressed('down'))
        
        yaw_target_smoothed = yaw_target_smoothed * alpha + yaw_target * (1. - alpha)
        vel_target_smoothed = vel_target_smoothed * alpha + vel_target * (1. - alpha)
        if send_state:
            ret = bytes("VEL_T", 'ascii') + bytes([0]) + struct.pack('f', vel_target_smoothed)
        else:
            ret = bytes("DYAW_T", 'ascii') + bytes([0]) + struct.pack('f', yaw_target_smoothed)
        send_state = not send_state

        return ret

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

    print("Done, tearing down")
