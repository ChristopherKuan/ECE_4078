from feedforward_pid import PID
from picamera2 import Picamera2
from flask import Flask, Response, request
from gpiozero import Robot, Motor, DigitalInputDevice
import io
import time
import threading

app = Flask(__name__)

class Encoder(object):
    def __init__(self, pin):
        self._value = 0
        self.encoder = DigitalInputDevice(pin)
        self.encoder.when_activated = self._increment
        self.encoder.when_deactivated = self._increment
    
    def reset(self):
        self._value = 0
    
    def _increment(self):
        self._value += 1
        
    @property
    def value(self):
        return self._value

# Main function to control the robot wheels
def move_robot():
    global use_pid, left_speed, right_speed
    flag_new_pid_cycle = True
    while True:
        if not use_pid:
            pibot.value = (left_speed, right_speed)          
        else:
            if motion == 'stop':
                pibot.value = (0, 0)
                left_encoder.reset()
                right_encoder.reset()
                flag_new_pid_cycle = True
            else:
                left_speed, right_speed = abs(left_speed), abs(right_speed)

                if flag_new_pid_cycle:
                    # Initialize PID controllers
                    pid_avg_speed = PID(kp, ki, kd, setpoint=(left_speed + right_speed) / 2, output_limits=(0, 1), starting_output=(left_speed + right_speed) / 2)
                    pid_diff_speed = PID(kp_diff, ki_diff, kd_diff, setpoint=0, output_limits=(-1, 1), starting_output=0)
                    flag_new_pid_cycle = False

                # Calculate the average speed of both wheels
                avg_speed = (left_encoder.value + right_encoder.value) / 2

                # Calculate the speed differential
                speed_diff = left_encoder.value - right_encoder.value

                # Update the PID controllers
                avg_speed_control = pid_avg_speed(avg_speed)
                diff_speed_control = pid_diff_speed(speed_diff)

                # Adjust speeds based on PID control
                left_control_speed = avg_speed_control + diff_speed_control
                right_control_speed = avg_speed_control - diff_speed_control

                if motion == 'forward':
                    pibot.value = (left_control_speed, right_control_speed)
                elif motion == 'turning':
                    pibot.value = (left_control_speed, right_control_speed)
                else:
                    pibot.value = (-left_control_speed, -right_control_speed)
        time.sleep(0.005)

# Receive confirmation whether to use PID or not to control the wheels
@app.route('/pid')
def set_pid():
    global use_pid, kp, ki, kd, kp_diff, ki_diff, kd_diff
    use_pid = int(request.args.get('use_pid'))
    if use_pid:
        kp, ki, kd = float(request.args.get('kp')), float(request.args.get('ki')), float(request.args.get('kd'))
        kp_diff, ki_diff, kd_diff = float(request.args.get('kp_diff')), float(request.args.get('ki_diff')), float(request.args.get('kd_diff'))
        return "Using PID"
    else:
        return "Not using PID"

# Receive a request to capture and send a snapshot of the picamera
@app.route('/image')
def capture_image():
    stream = io.BytesIO()
    picam2.capture_file(stream, format='jpeg')
    stream.seek(0)
    return Response(stream, mimetype='image/jpeg')

# Receive command to move the pibot
@app.route('/move')
def move():
    global left_speed, right_speed, motion
    left_speed, right_speed = float(request.args.get('left_speed')), float(request.args.get('right_speed'))

    if left_speed == 0 and right_speed == 0:
        motion = 'stop'
    elif left_speed != right_speed:
        motion = 'turning'
    else:
        motion = 'forward'
    return motion

# Constants
in1 = 17
in2 = 27
ena = 18
in3 = 23
in4 = 24
enb = 25
enc_a = 26
enc_b = 16

# Initialize robot and encoders
pibot = Robot(right=Motor(forward=in1, backward=in2, enable=ena), left=Motor(forward=in3, backward=in4, enable=enb))

# Initialize PID controllers for wheels
left_encoder = Encoder(enc_a)
right_encoder = Encoder(enc_b)
use_pid = 0
kp, ki, kd = 0, 0, 0
kp_diff, ki_diff, kd_diff = 0, 0, 0
left_speed, right_speed = 0, 0
motion = ''

# Initialize the PiCamera
picam2 = Picamera2()
config = picam2.create_preview_configuration(lores={"size": (640,480)})
picam2.configure(config)
picam2.start()
time.sleep(2)

# Initialize flask
def run_flask():
    app.run(host='0.0.0.0', port=5000)
flask_thread = threading.Thread(target=run_flask)
flask_thread.daemon = True
flask_thread.start()

try:
    while True:
        move_robot()
except KeyboardInterrupt:
    pibot.stop()
    picam2.stop()
    print("Program interrupted by user.")
