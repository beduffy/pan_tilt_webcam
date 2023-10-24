from flask import Flask, request, jsonify
from adafruit_servokit import ServoKit


myKit = ServoKit(channels=16)

app = Flask(__name__)

@app.route("/")
def hello_world():
    return "Hello/n"

@app.route("/control_servo", methods=["POST"])
def control_servo():
    data = request.get_json()
    print('incoming data:', data)
    servo_value = data["servo_value"]
    servo_index = int(data.get('servo_index', 0))
    print('servo_value: ', servo_value, ' servo_index', servo_index)
    myKit.servo[servo_index].angle=float(servo_value)
    return "servo controlled"

#app.run(port=8080)
app.run(port=8080, host="0.0.0.0")

# TODO put it on the nano and then update nicely. and/Or use vscode on nano