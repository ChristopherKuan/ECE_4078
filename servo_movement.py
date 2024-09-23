from flask import Flask
import pigpio
import time
import threading

app = Flask(__name__)

# Servo GPIO pin
servo_pin = 23

# Initialize pigpio
pwm = pigpio.pi()
pwm.set_mode(servo_pin, pigpio.OUTPUT)
pwm.set_PWM_frequency(servo_pin, 50)

# Function to move the servo to specific angles
def move_servo(angle):
    if angle == 0:
        print("0 deg")
        pwm.set_servo_pulsewidth(servo_pin, 500)
    elif angle == 90:
        print("90 deg")
        pwm.set_servo_pulsewidth(servo_pin, 1500)
    elif angle == 180:
        print("180 deg")
        pwm.set_servo_pulsewidth(servo_pin, 2500)
    else:
        raise ValueError("Angle must be 0, 90, or 180 degrees.")
    time.sleep(3)  # Wait for the servo to reach the position

# Function for automatic movement
def automatic_movement():
    while True:
        move_servo(0)
        #move_servo(90)
        #move_servo(180)

@app.route('/')
def home():
    return "Servo is moving automatically!"

def run_flask():
    app.run(host='0.0.0.0', port=5000)

# Start the Flask server in a separate thread
flask_thread = threading.Thread(target=run_flask)
flask_thread.daemon = True
flask_thread.start()

# Start the automatic movement in a separate thread
auto_movement_thread = threading.Thread(target=automatic_movement)
auto_movement_thread.daemon = True
auto_movement_thread.start()

try:
    while True:
        time.sleep(1)  # Keep the main thread alive
except KeyboardInterrupt:
    print("Program interrupted by user.")
    # Turn off the servo
    pwm.set_servo_pulsewidth(servo_pin, 0)  # Stop sending signals to the servo
    pwm.stop()  # Stop pigpio
