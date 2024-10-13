from simple_pid import PID
from picamera2 import Picamera2
from flask import Flask, Response, request, jsonify
from gpiozero import Robot, Motor, DigitalInputDevice
import pigpio
import io
import time
import threading
app = Flask(__name__)

# Servo GPIO pin
servo_pin = 12

# Initialize pigpio for servo control
pwm = pigpio.pi()
if not pwm.connected:
    print("Could not connect to pigpio daemon!")
    exit()
pwm.set_mode(servo_pin, pigpio.OUTPUT)
pwm.set_PWM_frequency(servo_pin, 50)

# Function to move the servo to specific angles
def move_servo(angle):
    # Map -90 to 90 degrees to pulsewidth 500 to 2500
    pulsewidth = int(1500 + (angle * 1000 / 90))  
    print(f"Moving servo to {angle} degrees with pulsewidth {pulsewidth}")
    pwm.set_servo_pulsewidth(servo_pin, pulsewidth)
    time.sleep(1)  # Adjust this based on your servo's speed

# Flask route to control the servo via API
@app.route('/move_servo', methods=['POST'])
def control_servo():
    try:
        angle = int(request.form['angle'])
        if -90 <= angle <= 90:
            move_servo(angle)
            return f"Servo moved to {angle} degrees", 200
        else:
            return "Invalid angle. Allowed range is -90 to 90 degrees.", 400
    except Exception as e:
        return f"Error: {e}", 400


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
    global use_pid, left_speed, right_speed, motion, milestone, ticks, left_encoder_value, right_encoder_value
    flag_new_pid_cycle = True
    while True:
        if milestone != 4:
            # print("WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWRRRRRRRRRRRRRRROOOOOOOOOONNNNNNNNNNGGGGGGGGGGGGG")
            ### if not using pid, just move the wheels as commanded
            if not use_pid:
                pibot.value = (left_speed, right_speed)          
            
            ### with pid, left wheel is set as reference, and right wheel will try to match the encoder counter of left wheel
            ### pid only runs when robot moves forward or backward. Turning does not use pid
            else:
                if (motion == 'stop') or (motion == 'turning'):
                    pibot.value = (left_speed, right_speed) 
                    left_encoder.reset()
                    right_encoder.reset()
                    flag_new_pid_cycle = True          
                else:
                    left_speed, right_speed = abs(left_speed), abs(right_speed)
                    if flag_new_pid_cycle:
                        pid_left = PID(kp, ki, kd, setpoint=right_encoder.value, output_limits=(0,1), starting_output=left_speed)
                        flag_new_pid_cycle = False
                    pid_left.setpoint = right_encoder.value
                    left_speed = pid_left(left_encoder.value)
                    if motion == 'forward':
                            if left_encoder.value <= 10 and right_encoder.value <= 10 :
                                pibot.value = (left_speed * 0.9, right_speed * 0.6)         
                            else:
                                pibot.value = (left_speed, right_speed)
                        #pibot.value = (left_speed, right_speed)
                    else: 
                            if left_encoder.value <= 10 and right_encoder.value <= 10 :
                                pibot.value = (-left_speed * 0.9, -right_speed * 0.6)         
                            else:
                                pibot.value = (-left_speed, -right_speed)
                        # pibot.value = (-left_speed, -right_speed)
                    # if flag_new_pid_cycle:
                    #     pid_right = PID(kp, ki, kd, setpoint=left_encoder.value, output_limits=(0,1), starting_output=right_speed)
                    #     flag_new_pid_cycle = False
                    # pid_right.setpoint = left_encoder.value
                    # right_speed = pid_right(right_encoder.value)
                    # if motion == 'forward': pibot.value = (left_speed, right_speed)
                    # else: pibot.value = (-left_speed, -right_speed)
                    print('Value', left_encoder.value, right_encoder.value)
                    print('Speed', left_speed, right_speed)

        else: ### EDIT THIS ONE
            ### if not using pid, just move the wheels as commanded
            # print("CCCCCCCCCCCCCCCCCoooooooooooooooooooooooooooorrect")
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
                        
                    elif motion == 'turning':
                        if left_speed > right_speed:
                            dir = "right"
                        else:
                            dir = "left"
                            
                        if dir == 'left':
                            if left_encoder.value >= ticks:
                                pibot.value = (-0.02, -0.02)
                                left_encoder_value = left_encoder.value
                                right_encoder_value = right_encoder.value
                                motion = "stop"
                            else:
                                # if flag_new_pid_cycle:
                                #     pid_right = PID(0.1,0.01, 0.0004, setpoint=left_encoder.value, output_limits=(0,1), starting_output=0)
                                #     flag_new_pid_cycle = False
                                # pid_right.setpoint = left_encoder.value
                                # right_speed = pid_right(right_encoder.value)
                                pibot.value = (left_speed, right_speed)
                        else:
                            if right_encoder.value >= ticks:
                                pibot.value = (-0.02, -0.02)
                                left_encoder_value = left_encoder.value
                                right_encoder_value = right_encoder.value
                                motion = "stop"
                            else:
                                # if flag_new_pid_cycle:
                                #     pid_left = PID(0.1, 0.01, 0.0004, setpoint=right_encoder.value, output_limits=(0,1), starting_output=0)
                                #     flag_new_pid_cycle = False
                                # pid_left.setpoint = right_encoder.value
                                # left_speed = pid_left(left_encoder.value)
                                pibot.value = (left_speed, right_speed)

                    else:
                        pibot.value = (0, 0)
                        left_encoder.reset()
                        right_encoder.reset()
                        flag_new_pid_cycle = True

                else:
                    left_speed, right_speed = abs(left_speed), abs(right_speed)
                    if flag_new_pid_cycle:
                        pid_right = PID(kp, ki, kd, setpoint=left_encoder.value, output_limits=(0,1), starting_output=right_speed)
                        flag_new_pid_cycle = False
                    pid_right.setpoint = left_encoder.value
                    right_speed = pid_right(right_encoder.value)
                    if motion == 'forward':
                            if left_encoder.value >= ticks:
                                pibot.value = (-0.02, -0.02)
                                left_encoder_value = left_encoder.value
                                right_encoder_value = right_encoder.value
                                motion = "stop" 
                            elif left_encoder.value <= 10 and right_encoder.value <= 10 :
                                pibot.value = (left_speed * 0.9, right_speed * 0.6)
                            else:
                                pibot.value = (left_speed, right_speed)
                        #pibot.value = (left_speed, right_speed)
                    else: 
                            if left_encoder.value <= 10 and right_encoder.value <= 10 :
                                pibot.value = (-left_speed * 0.9, -right_speed * 0.6)         
                            else:
                                pibot.value = (-left_speed, -right_speed)
                    print('Value', left_encoder.value, right_encoder.value)
                    print('Speed', left_speed, right_speed)
        time.sleep(0.002)

@app.route('/get_encoders', methods=['GET'])
def return_encoders():
    global left_encoder_value, right_encoder_value
    try:
        # Return the encoder values as a JSON response
        data = {
            'left_encoder': left_encoder_value,
            'right_encoder': right_encoder_value
        }
        return jsonify(data), 200
    except Exception as e:
        return f"Error: {e}", 400 
    
    
# Receive confirmation whether to use pid or not to control the wheels (forward & backward)
@app.route('/pid')
def set_pid():
    global use_pid, kp, ki, kd
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
    global left_speed, right_speed, motion, milestone, ticks
    left_speed, right_speed, milestone, ticks = float(request.args.get('left_speed')), float(request.args.get('right_speed')), int(request.args.get('milestone')), int(request.args.get('ticks'))
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
# own added
milestone = 0
ticks = 0


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
        return_encoders()
except KeyboardInterrupt:
    pibot.stop()
    picam2.stop()
    print("Program interrupted by user.")
finally:
    # Turn off the servo when the program ends
    pwm.set_servo_pulsewidth(servo_pin, 0)  # Stop sending signals to the servo
    pwm.stop()  # Stop pigpio
