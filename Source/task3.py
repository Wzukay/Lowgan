import cv2
import numpy as np
from gpiozero import Servo, OutputDevice, PWMOutputDevice
import time
from time import sleep

# ======= DC MOTOR SETUP =======
# Define GPIO pins for DC motor control
IN1 = OutputDevice(14)          # Motor input 1
IN2 = OutputDevice(15)          # Motor input 2
ENA = PWMOutputDevice(17)       # PWM for motor speed control (ENA pin)

# Step sequence for motor (simplified here)
step_sequence = [
    [1, 0],
    [1, 0],
    [1, 0],
    [1, 0],
    [1, 0],
    [1, 0],
    [1, 0],
    [1, 0]
]

def set_step(w1, w2):
    """Set motor GPIO pins to specified logic levels."""
    IN1.value = w1
    IN2.value = w2

def step_motor(steps, direction=1, delay=0.01, speed=1.0):
    """
    Run the motor for a number of steps.

    Args:
        steps (int): Number of steps to run
        direction (int): 1 for forward, -1 for backward
        delay (float): Delay between steps in seconds
        speed (float): PWM duty cycle (0 to 1)
    """
    ENA.value = speed  # Set motor speed via PWM
    for _ in range(steps):
        seq = step_sequence if direction > 0 else reversed(step_sequence)
        for step in seq:
            set_step(*step)
            sleep(delay)
    ENA.value = 0  # Stop the motor after stepping

# ======= SERVO SETUP =======
# Initialize servo motor on GPIO pin 18 with PWM pulse width range
servo = Servo(18, min_pulse_width=500/1_000_000, max_pulse_width=2000/1_000_000)

# ======= GLOBAL STATE VARIABLE =======
state = 0  # 0=no detection, 1=detected with camera (controls servo and motor), 2=placeholder

def ChangeState(n):
    """Change the global state variable."""
    global state
    state = n

# ======= COLOR DETECTION & CONTROL FUNCTIONS =======

def State0(frame):
    """
    State 0: Detect red and green colors in the frame but do NOT control servo or motor.
    Just display detection results and set state to 1 when detection occurs.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    green_size = 0
    red_size = 0

    # HSV ranges for red (two ranges due to hue wrap)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    # HSV range for green
    lower_green = np.array([25, 80, 80])
    upper_green = np.array([45, 255, 255])

    # Create red mask combining two ranges
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)

    # Green mask
    green_mask = cv2.inRange(hsv, lower_green, upper_green)

    # Detect red contours
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(red_contours) != 0:
        red_contours = sorted(red_contours, key=cv2.contourArea, reverse=True)
        largest = red_contours[0]
        if cv2.contourArea(largest) > 1000:
            x, y, w, h = cv2.boundingRect(largest)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
            red_size = cv2.contourArea(largest)
            print('Red detected:', red_size)
            cv2.putText(frame, 'Red Detected', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

    # Detect green contours
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(green_contours) != 0:
        green_contours = sorted(green_contours, key=cv2.contourArea, reverse=True)
        largest = green_contours[0]
        if cv2.contourArea(largest) > 1000:
            x, y, w, h = cv2.boundingRect(largest)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            green_size = cv2.contourArea(largest)
            print('Green detected:', green_size)
            cv2.putText(frame, 'Green Detected', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    # If any color detected, change state to 1 (active control)
    if red_size > 1000 or green_size > 1000:
        ChangeState(1)

def State1(frame):
    """
    State 1: Detect red and green, control servo and DC motor accordingly.
    Servo turns left/right based on color, motor runs forward/backward accordingly.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    green_size = 0
    red_size = 0

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    lower_green = np.array([25, 80, 80])
    upper_green = np.array([45, 255, 255])

    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)

    green_mask = cv2.inRange(hsv, lower_green, upper_green)

    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(red_contours) != 0:
        red_contours = sorted(red_contours, key=cv2.contourArea, reverse=True)
        largest = red_contours[0]
        if cv2.contourArea(largest) > 1000:
            x, y, w, h = cv2.boundingRect(largest)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
            red_size = cv2.contourArea(largest)
            cv2.putText(frame, 'Red Detected', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(green_contours) != 0:
        green_contours = sorted(green_contours, key=cv2.contourArea, reverse=True)
        largest = green_contours[0]
        if cv2.contourArea(largest) > 1000:
            x, y, w, h = cv2.boundingRect(largest)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            green_size = cv2.contourArea(largest)
            cv2.putText(frame, 'Green Detected', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    # Control servo and motor based on which color is larger
    if red_size > green_size and red_size > 1000:
        cv2.putText(frame, 'Go Right', (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 2)
        print("Servo turn RIGHT")
        servo.value = 1  # Servo full right
        time.sleep(0.5)
        servo.value = 0  # Center servo
        
        print("Motor running FORWARD (red detected)")
        step_motor(steps=100, direction=1, speed=0.8)  # Motor forward

    elif green_size > red_size and green_size > 1000:
        cv2.putText(frame, 'Go Left', (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), 2)
        print("Servo turn LEFT")
        servo.value = -1  # Servo full left
        time.sleep(0.5)
        servo.value = 0  # Center servo
        
        print("Motor running BACKWARD (green detected)")
        step_motor(steps=100, direction=-1, speed=0.8)  # Motor backward

    else:
        # If no clear dominant color, keep servo centered and stop motor
        servo.value = 0

def State2(frame):
    """Placeholder for state 2 - no servo or motor control."""
    print("State 2 - no control implemented")

def Loop():
    """Capture frame, process by current state, and display."""
    ret, frame = cap.read()
    if not ret:
        ChangeState(-1)  # Signal exit if frame not captured

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        ChangeState(-1)

    # Call function based on current state
    if state == 0:
        State0(frame)
    elif state == 1:
        State1(frame)
    elif state == 2:
        State2(frame)

    # Show current state on frame
    cv2.putText(frame, 'State=' + str(state), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 2)

    # Display frame window
    cv2.imshow("PY", frame)

    # Additional key controls for cycling states
    key = cv2.waitKey(1) & 0xFF
    if key == ord('f'):
        # Cycle states 1 -> 2 -> 0
        if state == 1:
            ChangeState(2)
        elif state == 2:
            ChangeState(0)

    if key == ord('e'):
        # Cycle states 0 -> 1 -> 2 -> 0
        if state == 0:
            ChangeState(1)
        elif state == 1:
            ChangeState(2)
        elif state == 2:
            ChangeState(0)

# ======= INITIALIZE CAMERA =======
cap = cv2.VideoCapture(1)  # Use camera index 1, change if needed
if not cap.isOpened():
    print("Cannot open camera")
    exit()

# ======= MAIN PROGRAM LOOP =======
try:
    while True:
        Loop()
        if state == -1:
            break  # Exit requested

except KeyboardInterrupt:
    print("Program stopped by user")

finally:
    # Cleanup
    servo.detach()        # Stop servo PWM
    cap.release()         # Release camera
    cv2.destroyAllWindows()  # Close OpenCV windows
    print("Cleaned up and exited")
