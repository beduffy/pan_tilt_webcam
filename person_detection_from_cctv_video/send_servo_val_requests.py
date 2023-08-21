import requests
import json

def post_servo_value(servo_value):
    # curl 192.168.178.125:8080/control_servo -H "Content-Type: application/json" -d '{"servo_value": "0"}'
    IP = '192.168.178.125'

    url = 'http://{}:8080/control_servo'.format(IP)
    myobj = {'servo_value': '{}'.format(servo_value)}

    # print(url)
    # x = requests.post(url, json = myobj, headers={"Content-Type": "application/json"})
    x = requests.post(url, data = json.dumps(myobj), headers={"Content-Type": "application/json"})
    print('Sent {} angle to flask'.format(servo_value))
    # print(x.text)

if __name__ == '__main__':
    post_servo_value(0)