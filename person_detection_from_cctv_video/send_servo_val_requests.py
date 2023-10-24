import requests
import grequests
import json


import sys

print(sys.argv)

# TODO move outside of this folder
# curl 192.168.178.125:8080/control_servo -H "Content-Type: application/json" -d '{"servo_value": "0"}'
def post_servo_value(servo_value, servo_idx=0):
    # IP = '192.168.178.125'
    IP = '192.168.178.36'

    url = 'http://{}:8080/control_servo'.format(IP)
    data_to_send = {'servo_value': '{}'.format(servo_value), 
                    'servo_index': servo_idx}

    # print(url)
    # x = requests.post(url, data = json.dumps(myobj), headers={"Content-Type": "application/json"})
    rs = (grequests.post(url, data = json.dumps(data_to_send), headers={"Content-Type": "application/json"}), )
    grequests.map(rs)
    # print('Sent {:.3f} angle to flask'.format(servo_value))
    # print(x.text)

if __name__ == '__main__':
    # post_servo_value(sys.argv[1])
    post_servo_value(sys.argv[1], sys.argv[2])