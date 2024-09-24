from flask import Flask, request
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
    pulsewidth = int(1500 + (angle * 1000 / 90))  # Map -90 to 90 to pulsewidth 500 to 2500
    print(f"Moving servo to {angle} degrees with pulsewidth {pulsewidth}")
    pwm.set_servo_pulsewidth(servo_pin, pulsewidth)
    time.sleep(1)  # Adjust this based on your servo's speed

# Flask route to control servo via API
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

@app.route('/')
def home():
    return "Servo control API is running!"

def run_flask():
    app.run(host='0.0.0.0', port=5000)

# Start the Flask server in a separate thread
flask_thread = threading.Thread(target=run_flask)
flask_thread.daemon = True
flask_thread.start()

# Start keyboard control in the main thread
try:
    keyboard_control()
except KeyboardInterrupt:
    print("Program interrupted by user.")
finally:
    # Turn off the servo
    pwm.set_servo_pulsewidth(servo_pin, 0)  # Stop sending signals to the servo
    pwm.stop()  # Stop pigpio
