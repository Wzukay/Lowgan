import time
import lgpio
from gpiozero import AngularServo, OutputDevice, PWMOutputDevice
from time import sleep

# === PIN AND CONSTANT CONFIGURATION ===

# Pins for the TCS3200 color sensor
PIN_S0 = 23
PIN_S1 = 24
PIN_S2 = 27
PIN_S3 = 22
PIN_OUT = 17  # Frequency output pin from the sensor

fa = 165
fadr = 180
fast = 60

lastDirection = -1
lastValue = -1
lastColor = ""

# Pin for controlling the servo motor
servo = AngularServo(12, min_angle = 0, max_angle =180, min_pulse_width=500/1_000_000, max_pulse_width=2000/1_000_000)

# Pins for controlling the DC motor
IN1 = OutputDevice(14)                            
IN2 = OutputDevice(15)
ENA = PWMOutputDevice(26)  # ENA for Motor 1 PWM speed control

# Color detection tolerance threshold
COLOR_TOLERANCE = 200

# === GPIO INITIALIZATION ===

# Open connection to GPIO chip 0 for color sensor
h = lgpio.gpiochip_open(0)

# Configure sensor pins as outputs or input
lgpio.gpio_claim_output(h, PIN_S0)
lgpio.gpio_claim_output(h, PIN_S1)
lgpio.gpio_claim_output(h, PIN_S2)
lgpio.gpio_claim_output(h, PIN_S3)
lgpio.gpio_claim_input(h, PIN_OUT)

# Set sensor output frequency scaling to 100% (maximum frequency)
lgpio.gpio_write(h, PIN_S0, 1)
lgpio.gpio_write(h, PIN_S1, 1)

# === HELPER FUNCTIONS FOR COLOR SENSOR ===

def set_color_filter(s2, s3):
    """Set color filter on sensor using pins S2 and S3."""
    lgpio.gpio_write(h, PIN_S2, s2)
    lgpio.gpio_write(h, PIN_S3, s3)
    
def SetAngle(angle):
    servo.angle = angle
    print(angle)

def read_frequency(duration=0.1):
    """Measure frequency output from color sensor over given duration."""
    t_start = time.time()
    count = 0
    last = lgpio.gpio_read(h, PIN_OUT)
    
    while time.time() - t_start < duration:
        current = lgpio.gpio_read(h, PIN_OUT)
        if current != last:
            count += 1
            last = current
    
    freq = count / (2 * duration)  # Calculate frequency in Hz
    return freq

def read_rgb():
    """Read red, green, blue frequency values from sensor."""
    set_color_filter(0, 0)  # Red filter
    red = read_frequency()
    set_color_filter(1, 1)  # Green filter
    green = read_frequency()
    set_color_filter(0, 1)  # Blue filter
    blue = read_frequency()
    return red, green, blue

def detect_color(red, green, blue):
    """Determine color from RGB frequencies with tolerance."""
    if red > blue + COLOR_TOLERANCE:
        return "ORANGE"
    elif blue > red + COLOR_TOLERANCE and blue < 40000:
        return "BLUE"
    else:
        return "WHITE"

def stop_servo():
    """Detach servo to stop PWM signals."""
    print("[DEBUG] Stop servo")
    servo.detach()

# === DC MOTOR CONTROL FUNCTIONS ===

def set_step(w1, w2):
    """Set motor driver input pins."""
    IN1.value = w1
    IN2.value = w2

# === DC MOTOR CONTROL FUNCTIONS - FIXED ===

def ensure_motor_running():
    """Asigura-te ca motorul merge continuu - DOAR daca nu merge deja"""
    if ENA.value != .8:  # Verifica daca PWM-ul nu e setat corect
        ENA.value = .8  # Seteaza viteza
    
    if IN1.value != 0 or IN2.value != 1:
        set_step(0, 1)    # Seteaza direc?ia

# === MAIN EXECUTION LOOP - OPTIMIZED ===

try:
    print("Starting color sensor reading, servo, and DC motor control...")
    
    # Porneste motorul continuu la Inceput
    ENA.value = .8
    set_step(0, 1)    
    
    loop_counter = 0
    
    while True:
        # Read color sensor frequencies
        red, green, blue = read_rgb()
        print(f"Red: {red:.1f} Hz | Green: {green:.1f} Hz | Blue: {blue:.1f} Hz")
        
        # Detect color
        color = detect_color(red, green, blue)
        print(f"Detected {color}")

        # Control servo based on detected color
        if color == "ORANGE":
            if lastColor == "BLUE":
                 lastColor = "BLUE/ORANGE"
            elif lastColor == "":
                lastColor = "ORANGE"
                
        
            SetAngle(fadr)  # Start moving to final angle
            lastDirection = 1
            
        elif color == "BLUE":
            print("Turn servo LEFT")
            if lastColor == "ORANGE":
                lastColor = "ORANGE/BLUE"
            elif lastColor == "":
                lastColor = "BLUE"
            
            SetAngle(fast)  # Start moving to side angle
            lastDirection = 0

        elif color == "WHITE":
            print("Color WHITE detected")
            if lastColor == "ORANGE/BLUE" or lastColor == "BLUE/ORANGE":
                print("changed in color after both")
                
            if lastDirection == 0:
                SetAngle(150)
            elif lastDirection == 1:
                SetAngle(90)
            elif lastDirection == -1:
                SetAngle(180)
            lastDirection = -1
        
        # Verifica motorul doar la fiecare 20 de iteratii pentru a evita interferentele
        loop_counter += 1
        if loop_counter % 15 == 0:
            ensure_motor_running()
            
        time.sleep(0.05)
        
except KeyboardInterrupt:
    print("Manual stop by user.")

finally:
    # Cleanup: stop motor, servo and release GPIO
    ENA.value = 0  # Opreste motorul
    set_step(0, 0)  # Opreste semnalele de directie
    stop_servo()
    lgpio.gpiochip_close(h)
    print("Motor stopped, PWM stopped and GPIO released.")
