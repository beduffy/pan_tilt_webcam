import serial

s = serial.Serial('/dev/ttyACM0', 9600, timeout = 1)
# s = serial.Serial('/dev/ttyACM0', 9600, bytesize=8, timeout = 1)

angle = 50.0

# res = s.write('10.0')

cmd = '{}\n'.format(angle)

res = s.write(cmd.encode())
s.flush()

# TODO maybe I just need to push a few liens across first to get the serial connection started? or sleep?
for i in range(5):

# while True:
    r = s.readline()
    print(r)
    res = s.write('100.0\n'.encode())
    s.flush()
    # print(res)