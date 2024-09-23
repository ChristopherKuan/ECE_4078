from flask import Flask
from gpiozero import Servo
import time
import threading

app = Flask(__name__)

# Initialize the servo
servo = Servo(22)  # Use the GPIO pin connected to the servo

# Current angle of the servo
current_angle = 0

# Function to map angle to servo value
def angle_to_servo_value(angle):
    if angle < -90 or angle > 90:
        raise ValueError("Angle must be between -90 and 90 degrees.")
    return angle / 90  # Map -90 to 1 and 90 to -1

# Function to move the servo to a specific angle
def move_servo(angle):
    global current_angle
    servo.value = angle_to_servo_value(angle)
    current_angle = angle

# Function for automatic movement
def automatic_movement():
    global current_angle
    while True:
        # Increment the angle by 10 degrees, ensuring it stays within bounds
        new_angle = current_angle + 10
        if new_angle > 90:
            new_angle = -90  # Reset to -90 if it exceeds 90
        move_servo(new_angle)
        time.sleep(2)  # Wait for 2 seconds before moving again

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
