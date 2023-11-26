import serial
import time

import numpy as np

# s = serial.Serial('/dev/ttyACM0', 9600, timeout = 1)
s = serial.Serial('/dev/ttyACM0', 115200, timeout = 1)
# s = serial.Serial('/dev/ttyACM0', 9600, bytesize=8, timeout = 1)

angle = 50.0
cmd = '{}\n'.format(angle)

res = s.write(cmd.encode())
s.flush()

# chosen_angles = [0, 100, 100, -100, -100]
chosen_angles = [0, 30, -30, 30, -30, 30, -30, 30, -30]

# TODO maybe I just need to push a few lines across first to get the serial connection started? or sleep?
# for i in range(5):
for i in range(9):
# while True:

    # angle = np.random.choice([0, 50, 100, 20, 30])
    angle = chosen_angles[i]
    cmd = 'a{}\n'.format(angle)
    
    res = s.write(cmd.encode())
    s.flush()
    print('Sent: ', cmd)

    r = s.readline()
    print('Got serial line: ', r)
    
    time.sleep(0.1)
    # import pdb;pdb.set_trace()
    # print(res)