from flask import Flask, request
from gpiozero import Servo
import time
import threading

app = Flask(__name__)

# Initialize the servo
servo = Servo(22)  # Use the GPIO pin connected to the servo

# Function to map angle to servo value
def angle_to_servo_value(angle):
    if angle < -90 or angle > 90:
        raise ValueError("Angle must be between -90 and 90 degrees.")
    return angle / 90  # Map -90 to 1 and 90 to -1

# Function to move the servo to a specific angle
def move_servo(angle):
    servo.value = angle_to_servo_value(angle)

@app.route('/move_servo')
def control_servo():
    angle = request.args.get('angle', default=0, type=int)
    try:
        move_servo(angle)
        return f'Servo moved to {angle} degrees.'
    except ValueError as e:
        return str(e), 400  # Return error if angle is out of range

def run_flask():
    app.run(host='0.0.0.0', port=5000)

# Start the Flask server in a separate thread
flask_thread = threading.Thread(target=run_flask)
flask_thread.daemon = True
flask_thread.start()

try:
    while True:
        time.sleep(1)  # Keep the main thread alive
except KeyboardInterrupt:
    print("Program interrupted by user.")
