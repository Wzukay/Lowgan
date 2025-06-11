import time
import pigpio
from gpiozero import PWMOutputDevice, DigitalOutputDevice, DigitalInputDevice
import cv2
import numpy as np
from picamera2 import Picamera2
from time import sleep

# Initialize pigpio
pi = pigpio.pi()

# --- Color sensor pin setup (TCS3200) ---
S0 = DigitalOutputDevice(23)
S1 = DigitalOutputDevice(24)
S0.on()
S1.off()
S2 = DigitalOutputDevice(27)
S3 = DigitalOutputDevice(22)
OUT = DigitalInputDevice(17)

pi.set_mode(26, pigpio.OUTPUT)
pi.set_mode(14, pigpio.OUTPUT)

# --- Angular servo control via pigpio ---
SERVO_GPIO = 2
CENTER_PW = 1400  # Âµs for center
LEFT_PW = 1800  # Âµs for left turn
RIGHT_PW = 1200  # Âµs for right turn

# --- State tracking
lastColor = None
has_turned = False
left = -1

# ======= GLOBAL STATE VARIABLE =======
state = 0  # 0=no detection, 1=detected with camera (controls servo and motor), 2=placeholder


def ChangeState(n):
    """Change the global state variable."""
    global state
    state = n


# --- TCS3200 color reading ---
def read_color_frequency(s2_val, s3_val):
    S2.value = s2_val
    S3.value = s3_val

    count = 0
    start_time = time.time()
    timeout = 0.5

    while count < 10 and (time.time() - start_time < timeout):
        OUT.wait_for_inactive()
        OUT.wait_for_active()
        count += 1

    duration = time.time() - start_time
    return 10 / duration if duration > 0 else 0


def get_color():
    red = read_color_frequency(0, 0)
    blue = read_color_frequency(0, 1)
    green = read_color_frequency(1, 1)

    print(f"R: {red:.1f}, G: {green:.1f}, B: {blue:.1f}")

    if red > green and red > blue:
        return "orange"
    elif blue > red and blue > green and blue < 4000:
        return "blue"
    else:
        return "white"


# --- Servo angle control ---
def set_servo_pulse(pulse_width):
    pi.set_servo_pulsewidth(SERVO_GPIO, pulse_width)


def steer_center():
    set_servo_pulse(CENTER_PW)
    print("Center")


def steer_left():
    set_servo_pulse(LEFT_PW)
    print("Left")


def steer_right():
    set_servo_pulse(RIGHT_PW)
    print("Righjt")


def set_motor_direction(forward=True):
    pi.write(14, 0 if forward else 1)


def ensure_motor_running():
    pi.set_PWM_dutycycle(26, 180)  # Full speed (255 = 100% duty cycle)
    set_motor_direction(True)


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
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            red_size = cv2.contourArea(largest)
            print('Red detected:', red_size)
            cv2.putText(frame, 'Red Detected', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # Detect green contours
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(green_contours) != 0:
        green_contours = sorted(green_contours, key=cv2.contourArea, reverse=True)
        largest = green_contours[0]
        if cv2.contourArea(largest) > 1000:
            x, y, w, h = cv2.boundingRect(largest)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            green_size = cv2.contourArea(largest)
            print('Green detected:', green_size)
            cv2.putText(frame, 'Green Detected', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

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
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            red_size = cv2.contourArea(largest)
            cv2.putText(frame, 'Red Detected', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(green_contours) != 0:
        green_contours = sorted(green_contours, key=cv2.contourArea, reverse=True)
        largest = green_contours[0]
        if cv2.contourArea(largest) > 1000:
            x, y, w, h = cv2.boundingRect(largest)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            green_size = cv2.contourArea(largest)
            cv2.putText(frame, 'Green Detected', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Control servo and motor based on which color is larger
    if red_size > green_size and red_size > 1000:
        cv2.putText(frame, 'Go Right', (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        print("Servo turn RIGHT")
        steer_right()
        print("Motor running FORWARD (red detected)")
         # Motor forward

    elif green_size > red_size and green_size > 1000:
        cv2.putText(frame, 'Go Left', (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        print("Servo turn LEFT")
        steer_left()  # Center servo

        print("Motor running BACKWARD (green detected)")

    else:
        # If no clear dominant color, keep servo centered and stop motor
        steer_center()

def State2(frame):
    """Placeholder for state 2 - no servo or motor control."""
    print("State 2 - no control implemented")


def Loop():
    """Capture frame, process by current state, and display."""
    try:
        # Capture frame using picamera2
        frame = picam2.capture_array()

        # Convert from RGB to BGR for OpenCV compatibility
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    except Exception as e:
        print(f"Error capturing frame: {e}")
        ChangeState(-1)  # Signal exit if frame not captured
        return

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
    cv2.putText(frame, 'State=' + str(state), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

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
print("Initializing picamera2...")
picam2 = Picamera2()

# Configure camera for preview (you can adjust resolution as needed)
config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(config)

try:
    # Start the camera
    picam2.start()
    print("Camera started successfully")

    # Give camera time to warm up
    time.sleep(2)

except Exception as e:
    print(f"Cannot initialize camera: {e}")
    exit()


# --- Main loop ---
try:
    ensure_motor_running()

    while True:
        color = get_color()
        print(f"Detected: {color}")
        Loop()
        if state == -1:
            break  # Exit requested

        if not has_turned:
            if color == "blue":
                lastColor = "blue"
                print("Turning left")
                steer_right()
                left = 1
                has_turned = True
            elif color == "orange":
                lastColor = "orange"
                print("Turning right")
                steer_left()
                left = 0
                has_turned = True
            elif color == "white":
                print("White detected â€” going straight")
                lastColor = ""
                steer_center()
                left = -1
        elif has_turned:
            if lastColor == "blue" and color == "orange":
                has_turned = False
                steer_center()
            elif lastColor == "orange" and color == "blue":
                has_turned = False
                steer_center()

        # Always move forward
        ensure_motor_running()

        time.sleep(0.4)

except KeyboardInterrupt:
    print("Stopping...")
finally:
    try:
        pi.set_servo_pulsewidth(26, 0)
        pi.set_servo_pulsewidth(SERVO_GPIO, 0)  # Stop servo signal
        pi.stop()
        servo.detach()
        picam2.stop()
        cv2.destroyAllWindows()
    except:
        print("Error during cleanup")
