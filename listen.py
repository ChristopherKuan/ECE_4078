from simple_pid import PID
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
        

# main function to control the robot wheels
def move_robot():
    global use_pid, left_speed, right_speed, milestone
    flag_new_pid_cycle = True
    while True:
        ### if not using pid, just move the wheels as commanded
        if not use_pid:
            pibot.value = (left_speed, right_speed)          
        
        ### with pid, left wheel is set as reference, and right wheel will try to match the encoder counter of left wheel
        ### pid only runs when robot moves forward or backward. Turning does not use pid
        else:
            if (motion == 'stop') or (motion == 'turning') or (motion == ''):
                if motion == 'stop':
                    pibot.value = (0, 0) 
                    left_encoder.reset()
                    right_encoder.reset()
                    flag_new_pid_cycle = True
                    time.sleep(0.1)
                    #print("Stopped state\n")
                elif motion == 'turning':
                    if left_speed > right_speed:
                        dir = "right"
                        if flag_new_pid_cycle:
                            pid_left = PID(0.1, 0.01, 0.0004, setpoint=right_encoder.value, output_limits=(0,1), starting_output=0)
                            flag_new_pid_cycle = False
                    else:
                        dir = "left"
                        if flag_new_pid_cycle:
                            pid_right = PID(0.1,0.01, 0.0004, setpoint=left_encoder.value, output_limits=(0,1), starting_output=0)
                            flag_new_pid_cycle = False
                    if dir == "left":
                        pid_right.setpoint = left_encoder.value
                        right_speed = pid_right(right_encoder.value)
                        print(left_speed, right_speed)
                        pibot.value = (left_speed, right_speed)
                        if left_encoder.value >= 30:
                            pibot.value = (left_speed*0.5, right_speed*0.5)
                        if left_encoder.value >= 39:
                            pibot.value = (-0.1, -0.1)
                            
                    else:
                        pid_left.setpoint = right_encoder.value
                        left_speed = pid_left(left_encoder.value)
                        print(left_speed, right_speed)
                        pibot.value = (left_speed, right_speed)
                        if left_encoder.value >= 30:
                            pibot.value = (left_speed*0.5, right_speed*0.5)
                        if right_encoder.value >= 39:
                            pibot.value = (-0.1, -0.1)
                            
                    print("Left Turning:",left_encoder.value)
                    print("Right Turning:", right_encoder.value)


                else:
                    pibot.value = (0, 0) 
                    left_encoder.reset()
                    right_encoder.reset()
                    flag_new_pid_cycle = True
                        
                
                        # if left_speed < right_speed: # turn left
                        #     while right_encoder.value < 20 or left_encoder.value < 16:
                        #         while right_encoder.value < 4:
                        #             pibot.value = (0, right_speed)
                        #             print(left_encoder.value, right_encoder.value)

                        #         if right_encoder.value >= 20 and left_encoder.value < 16:
                        #             pibot.value = (left_speed, 0)
                        #         elif right_encoder.value < 20 and left_encoder.value >= 16:
                        #             pibot.value = (0, right_speed)
                        #         elif right_encoder.value - left_encoder.value != 4:
                        #             pibot.value = (right_speed, 0)
                        #         else:
                        #             pibot.value = (left_speed, right_speed)
                        #         print(left_encoder.value, right_encoder.value)
                        # else:    # turn right
                        #     while left_encoder.value < 20 or right_encoder.value < 16:
                        #         while left_encoder.value < 4:
                        #                 pibot.value = (left_speed, 0)
                        #                 print(left_encoder.value, right_encoder.value)

                        #         if left_encoder.value >= 20 and right_encoder.value < 16:
                        #             pibot.value = (0, right_speed)
                        #         elif left_encoder.value < 20 and right_encoder.value >= 16:
                        #             pibot.value = (left_speed, 0)
                        #         elif left_encoder.value - right_encoder.value != 4:
                        #             pibot.value = (left_speed, 0)
                        #         else:
                        #             pibot.value = (left_speed, right_speed)
                        #         print(left_encoder.value, right_encoder.value)                 
            else:
                # left_speed, right_speed = abs(left_speed), abs(right_speed)
                # if flag_new_pid_cycle:
                #     #pid_right = PID(kp, ki, kd, setpoint=left_encoder.value, output_limits=(0,1), starting_output=0)
                #     pid_left = PID(kp, ki, kd, setpoint=right_encoder.value, output_limits=(0,1), starting_output=0)
                #     flag_new_pid_cycle = False
                # #pid_right.setpoint = left_encoder.value
                # pid_left.setpoint = right_encoder.value
                # #right_speed = pid_right(right_encoder.value)
                # left_speed = pid_left(left_encoder.value)
                if flag_new_pid_cycle:
                    pid_left = PID(kp, ki, kd, setpoint=left_encoder.value, output_limits=(0, 1), starting_output=left_speed)
                    pid_right = PID(kp, ki, kd, setpoint=right_encoder.value, output_limits=(0, 1), starting_output=right_speed)
                    flag_new_pid_cycle = False
    
                # Set independent setpoint for left wheel (desired speed)
                pid_left.setpoint = left_encoder.value + left_speed
    
                # Right wheel tries to match the left wheel's encoder value
                pid_right.setpoint = left_encoder.value+ left_speed
    
                left_speed = pid_left(left_encoder.value)
                right_speed = pid_right(right_encoder.value)

                if motion == 'forward': 
                    if left_encoder.value >= 80 and right_encoder.value >= 80 :
                          pibot.value = (-0.02, -0.02)
                    elif left_encoder.value >= 60 and right_encoder.value >= 60:
                        pibot.value = (left_speed * 0.5, right_speed * 0.5)    
                    else:
                        pibot.value = (left_speed, right_speed)
                elif motion == "backward": 
                    if left_encoder.value >= 80 and right_encoder.value >= 80:
                          pibot.value = (0.02, 0.02)
                    elif left_encoder.value >= 60 and right_encoder.value >= 60:
                        pibot.value = (left_speed * 0.5, right_speed * 0.5)
                    else:
                        pibot.value = (left_speed, right_speed)
                print("Left:",left_encoder.value)
                print("Right:", right_encoder.value)
        time.sleep(0.002)
    
    
# Receive confirmation whether to use pid or not to control the wheels (forward & backward)
@app.route('/pid')
def set_pid():
    global use_pid, kp, ki, kd ,kf
    use_pid = int(request.args.get('use_pid'))
    if use_pid:
        kp, ki, kd = float(request.args.get('kp')), float(request.args.get('ki')), float(request.args.get('kd'))
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
    global left_speed, right_speed, motion, milestone
    left_speed, right_speed = float(request.args.get('left_speed')), float(request.args.get('right_speed'))#, int(request.args.get('milestone'))
    if (left_speed == 0 and right_speed == 0):
        motion = 'stop'
    elif (left_speed != right_speed ):
        motion = 'turning'
    elif (left_speed > 0 and right_speed > 0):
        motion = 'forward'
    elif (left_speed < 0 and right_speed < 0):
        motion = 'backward'
    return motion
    
    # if 'time' in request.args:


# Constants
in1 = 17 # may have to change this
in2 = 27 # may have to change this
ena = 18
in3 = 23 # may have to change this
in4 = 24 # may have to change this
enb = 25
enc_a = 26
enc_b = 16

# Initialize robot and encoders
pibot = Robot(right=Motor(forward=in1, backward=in2, enable=ena), left=Motor(forward=in3, backward=in4, enable=enb))

# Initialize PID controllers for wheels
left_encoder = Encoder(enc_a)
right_encoder = Encoder(enc_b)
use_pid = 0
kp = 0
ki = 0
kd = 0
left_speed, right_speed = 0, 0
motion = ''
milestone = 0

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
