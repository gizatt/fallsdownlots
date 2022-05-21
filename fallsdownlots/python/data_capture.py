import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
import random
import serial
import time
import traceback
import logging

import serial.tools.list_ports
ports = list(serial.tools.list_ports.comports())
print("Ports: ")
for p in ports:
    print(p)

# initialize serial port
ser = serial.Serial()
ser.port = 'COM4'  # Arduino serial port
ser.baudrate = 115200
ser.timeout = 1  # specify timeout when using readline()
ser.open()
if ser.is_open == True:
    print("\nAll right, serial port now open. Configuration:\n")
    print(ser, "\n")  # print serial parameters

# Create figure for plotting
N = 500
KEYS = ["t", "angle", "dangle", "speed"]
fig = plt.figure()
lines = []
for k in range(len(KEYS) - 1):
    plt.subplot(len(KEYS) - 1, 1, k+1)
    lines.append(plt.plot(np.zeros(N), np.zeros(N))[0])
    plt.ylabel(KEYS[k+1])

xs = np.zeros(N)
ys = np.zeros((N, len(KEYS) - 1))
k = 0
plt.show(block=False)

last_draw_time = time.time()

while 1:
    # Acquire and parse data from serial port
    try:
        line = ser.readline()  # ascii0
        line_as_list = line.split(b',')

        if len(line_as_list) != len(KEYS):
            logging.error("Invalid frame.")
            continue

        t = float(line_as_list[0])
        frame = np.array([float(x) for x in line_as_list[1:]])

        xs[k] = t
        ys[k, :] = frame
        k = (k + 1) % N

        if (time.time() - last_draw_time > 0.2):
            # Draw x and y lists
            for kl, line in enumerate(lines):
                plt.subplot(len(KEYS)-1, 1, kl+1)
                line.set_xdata(xs)
                line.set_ydata(ys[:, kl])
                plt.xlim(np.min(xs), np.max(xs))
                plt.ylim(np.min(ys[:, kl]), np.max(ys[:, kl]))
            fig.canvas.draw()
            fig.canvas.flush_events()

            print("Most recent message time: ", t, k)
            last_draw_time = time.time()

    except KeyboardInterrupt:
        break
    except Exception as e:
        print("Unhandled : ", e)
        print(traceback.format_exc())

input("Press key to close.")
