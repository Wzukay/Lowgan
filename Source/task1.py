import time
import lgpio
from gpiozero import Servo, OutputDevice, PWMOutputDevice
from time import sleep

# === PIN AND CONSTANT CONFIGURATION ===

# Pins for the TCS3200 color sensor
PIN_S0 = 23
PIN_S1 = 24
PIN_S2 = 27
PIN_S3 = 22
PIN_OUT = 17  # Frequency output pin from the sensor

# Pin for controlling the servo motor
servo = Servo(18, min_pulse_width=500/1_000_000, max_pulse_width=2000/1_000_000)

# Pins for controlling the DC motor
IN1 = OutputDevice(14)
IN2 = OutputDevice(15)
ENA = PWMOutputDevice(17)  # ENA for Motor 1 PWM speed control

# Color detection tolerance threshold
COLOR_TOLERANCE = 200

# Step sequence for DC motor control (simple on/off pattern here)
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

def step_motor(steps, direction=1, delay=0.01, speed=1.0):
    """Run the DC motor for a number of steps with given direction and speed."""
    ENA.value = speed  # Set motor speed using PWM (0 to 1)
    for _ in range(steps):
        seq = step_sequence if direction > 0 else reversed(step_sequence)
        for step in seq:
            set_step(*step)
            sleep(delay)
    ENA.value = 0  # Stop motor after steps completed

# === MAIN EXECUTION LOOP ===

try:
    print("Starting color sensor reading, servo, and DC motor control...")

    while True:
        # Read color sensor frequencies
        red, green, blue = read_rgb()
        print(f"Red: {red:.1f} Hz | Green: {green:.1f} Hz | Blue: {blue:.1f} Hz")
        
        # Detect color
        color = detect_color(red, green, blue)
        print(f"Detected {color}")

        # Control servo based on detected color
        if color == "ORANGE":
            print("Turn servo RIGHT")
            servo.value = 1       # Right position
            time.sleep(1)
            servo.value = -0.12   # Near center

            # Run DC motor forward when ORANGE detected
            print("Run DC motor forward")
            step_motor(steps=100, direction=1, speed=0.8)

        elif color == "BLUE":
            print("Turn servo LEFT")
            servo.value = -1      # Left position
            time.sleep(1)
            servo.value = 0.5     # Near center

            # Run DC motor backward when BLUE detected
            print("Run DC motor backward")
            step_motor(steps=100, direction=-1, speed=0.8)

        else:
            print("Color WHITE detected (no servo or motor action)")

except KeyboardInterrupt:
    print("Manual stop by user.")

finally:
    # Cleanup: stop servo and release GPIO
    stop_servo()
    lgpio.gpiochip_close(h)
    print("PWM stopped and GPIO released.")
