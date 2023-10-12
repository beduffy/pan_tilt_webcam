from flask import Flask, request, jsonify
from adafruit_servokit import ServoKit


myKit = ServoKit(channels=16)

app = Flask(__name__)

@app.route("/")
def hello_world():
    return "Hello, Irene/n"

@app.route("/control_servo", methods=["POST"])
def control_servo():
    data = request.get_json()
    print(data)
    servo_value = data["servo_value"]
    print(servo_value)
    myKit.servo[0].angle=int(servo_value)
    return "servo controlled"

#app.run(port=8080)
app.run(port=8080, host="0.0.0.0")