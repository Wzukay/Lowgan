import time
import pigpio
from gpiozero import PWMOutputDevice, DigitalOutputDevice, DigitalInputDevice

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
CENTER_PW = 1400  # µs for center
LEFT_PW = 1800  # µs for left turn
RIGHT_PW = 1200    # µs for right turn

# --- State tracking
lastColor = None
has_turned = False
left = -1

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

    if red>green and red>blue:
        return "orange"
    elif blue>red and blue>green and blue<4000:
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

def steer(ang):
    set_servo_pulse(ang)
    print(ang)
    
def set_motor_direction(forward=True):
    pi.write(14, 0 if forward else 1)

def ensure_motor_running():
    pi.set_PWM_dutycycle(26, 180)  # Full speed (255 = 100% duty cycle)
    set_motor_direction(True)
    
# --- Main loop ---
try:
    ensure_motor_running()
    
    while True:
        color = get_color()
        print(f"Detected: {color}")

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
    pi.set_servo_pulsewidth(26, 0)
    pi.set_servo_pulsewidth(SERVO_GPIO, 0)  # Stop servo signal
    pi.stop()
