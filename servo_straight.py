
# python3
from adafruit_servokit import ServoKit


myKit = ServoKit(channels=16)

myKit.servo[0].angle=int(80)

import pdb;pdb.set_trace()