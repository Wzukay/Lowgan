# 🤖 Lowgan - AI-Powered Autonomous Vehicle Navigation System
Welcome to the official repository of Lowgan, an advanced robotics solution developed for the WRO 2025 season. This project showcases the fusion of embedded intelligence, computer vision, and sensor integration in a custom-built robotic vehicle designed to autonomously navigate a defined game mat environment. Developed by a passionate team of robotics enthusiasts, Lowgan is capable of tackling real-world driving challenges at a small scale.

This repository contains all necessary resources to build, understand, and replicate the project: from hardware schematics and STL models, to Python-based control systems and AI modules.

## 🧠 Project Description
Lowgan is a small, AI-powered autonomous vehicle based on the Raspberry Pi 5 platform. The vehicle is designed to perform three sequential, progressively complex driving tasks, autonomously and without human intervention, using onboard computation and sensor feedback:

1. **Stage 1 - Circular Driving Around the Center**:
    The vehicle identifies the central landmark of the mat using color and visual cues.
    It maintains a constant radius and direction while circling, ensuring smooth arc motion.
    Control logic dynamically adjusts the steering servo and rear motor to stay on path.

2. **Stage 2 - Obstacle Avoidance**:
    The vehicle detects cube-shaped obstacles on the mat using its front-mounted camera.
    It calculates safe trajectories around these obstacles using computer vision and predefined rules.
    Precision motor control prevents contact, and sensor fusion ensures safe real-time reactions.

3. **Stage 3 - Lateral Parking**:
    The vehicle uses spatial awareness and side clearances to identify a parking zone.
    It then performs a lateral (parallel) parking maneuver similar to real urban driving.
    Vision input and color sensor feedback are used to measure space and initiate parking steps.

The software driving the vehicle includes several Python-based modules for motor control, image processing, and sensor data fusion.

## 🔩 Hardware Components
Lowgan combines precision hardware with flexible embedded software. The complete electromechanical setup includes:
    - **Raspberry Pi 5**: 	Main processor running Raspberry PI OS and all AI/logic systems
    - **L298N Motor Driver**: Controls the rear 6V DC motor using PWM for speed and direction control
    - **SG90 Servo Motor**: Attached to the front wheels, controls the steering angle using PWM
    - **Camera Module**: Provides real-time video for computer vision-based navigation
    - **TCS34725 Color Sensor**: Detects color transitions on the mat to signal task changes and assists with navigation
    - **Power Supply**: A regulated battery pack powers the entire vehicle
    - **3D Chassis**: 3D printed using custom-designed STL files available in the `/3D Models` folder

## 🧰 Software Modules
The codebase is structured into several modules, each responsible for a key aspect of the vehicle’s behavior:

## 🔁 System Integration
The Raspberry Pi 5 runs Raspberry Pi OS, and all scripts are written in Python 3.
The modules interact as follows:
    Camera Input + Color Sensor Input -- > Sensor Fusion --> Vision Processing --> Motion Planning & Decisions per Task --> Servo Motor + DC Motor Controller


## 💻 Code Overview
The software driving the Lowgan autonomous vehicle is organized into modular Python scripts to ensure maintainability, scalability, and clarity. The codebase is designed to interface directly with the vehicle’s hardware components, process sensory data, and implement AI-based navigation logic for each stage of the challenge.

    task1.py : 
        This Python script interfaces with a TCS3200 color sensor connected to a Raspberry Pi (using the lgpio library) to detect colors and control a servo motor based on the detected color.

        Key Components:
            - TCS3200 Color Sensor Pins:
                PIN_S0 to PIN_S3 configure the sensor’s output frequency scaling and color filter selection.
                PIN_OUT reads the frequency output corresponding to the intensity of the detected color.

            - Servo Motor:
                Connected to GPIO18 and controlled via gpiozero.Servo.
                Moves to different positions (left, center, right) based on the color detected.

            - How It Works:
                GPIO Setup:
                    Opens GPIO chip 0 and sets sensor pins as inputs or outputs.
                    Configures the sensor output frequency scaling to 100% (maximum frequency).
                Color Reading:
                    The sensor output frequency varies with the intensity of the detected color filtered by setting PIN_S2 and PIN_S3 to select red, green, or blue filters.
                    The script cycles through red, green, and blue filters, measuring the output frequency for a short duration.
                    Frequencies correspond roughly to the amount of each primary color detected.
                Color Detection Logic:
                    Compares the measured red, green, and blue frequencies.
                    Uses a tolerance value to decide if the detected color is "ORANGE", "BLUE", or defaults to "WHITE".
                    For example, if red frequency is significantly higher than blue, it classifies as ORANGE; if blue is higher than red but below a certain threshold, it's BLUE.
                Servo Control:
                    Based on the detected color:
                        ORANGE: Servo rotates to the right position briefly then returns near center.
                        BLUE: Servo rotates to the left position briefly then returns near center.
                        WHITE: No servo movement.
                DC Motor Control: Simultaneously, the DC motor runs in different directions based on color detection:
                    Based on the detected color:
                        ORANGE: Motor runs forward.
                        BLUE: Motor runs backward.
                        WHITE: Motor remains stopped.
                Program Loop and Cleanup:
                    Runs continuously, printing sensor readings and servo actions.
                    Stops cleanly on a keyboard interrupt (Ctrl+C), detaches the servo, and closes GPIO resources.

        This program is useful for projects requiring simple color recognition and corresponding mechanical response, such as sorting objects by color or interactive color-based robotics.

    task2.py :
        This Python script uses OpenCV to detect red and green objects in a live video feed from a camera and controls a servo motor connected to a Raspberry Pi based on the detected colors.

            - The script captures frames from a camera and converts them to HSV color space to detect red and green colors using color thresholding.
            - It identifies the largest contour of each color and draws bounding rectangles around detected objects.
            - Depending on which color has a larger detected area, the script changes its internal state variable and displays corresponding text on the video feed.
            - When in state 1, the script actively controls a servo motor:
                If red is dominant, the servo turns fully right briefly.
                If green is dominant, the servo turns fully left briefly.
                If no dominant color is detected, the servo remains centered.
            - The program supports switching between different states (0, 1, 2) using keyboard keys (e and f), though state 2 currently has no servo control implemented.
            - The DC motor is controlled via GPIO pins with PWM to set speed and direction through a stepping sequence.
            - The servo motor is controlled using PWM on a specified GPIO pin with defined pulse width ranges.
            - The program runs continuously, displaying real-time video with overlays indicating detected colors and current state.
            - The program supports graceful termination with the "q" key or Ctrl+C, ensuring proper cleanup by stopping the servo PWM, stopping the motor, releasing the camera, and  closing OpenCV windows.

        This setup could be useful for simple color-based object tracking and robotic control applications, such as guiding a servo to respond to color cues detected in real time.

    task3.py : 
        This Python program captures video from a camera and processes it in real time to detect red and green colors, using these detections to operate a servo motor and a DC motor connected to a Raspberry Pi.

        Main Features:
            - Color Recognition:
                Utilizing OpenCV, the program grabs video frames and converts them to the HSV color model. It isolates red and green objects through color filtering and contour analysis. Detected objects are highlighted with rectangles and labels shown on the video stream.

            - Operational Modes (States):
                The script controls behavior based on a global state variable with three distinct modes:
                    State 0: Scans for red and green but only shows detection visuals without moving motors. Detection triggers a switch to state 1.
                    State 1: Actively manages motors. According to the dominant color:
                        For red, the servo swings fully to the right momentarily, then returns to center, and the DC motor spins forward.
                        For green, the servo moves fully left briefly, then centers, and the DC motor runs backward.
                        If neither color is dominant, the servo centers and the DC motor halts.
                    State 2: Reserved mode with no control over motors or servo.

            - Motor Management:
                The DC motor direction is controlled via two GPIO output pins (IN1, IN2), while a PWM pin (ENA) modulates speed. The motor runs forward or backward by cycling through a predefined step pattern, with adjustable speed and timing.
                The servo motor, connected to GPIO pin 18, is driven by PWM signals, allowing positions from full left, center, to full right.

            - User Controls:
                The keys e and f let the user cycle through the states to test or operate different functionalities.
                The program can be exited cleanly by pressing q or interrupting with Ctrl+C, which stops PWM signals, releases the camera, and closes all OpenCV display windows.

            - Use Cases:
                This code serves as a basic framework for color-responsive robotics, enabling real-time color tracking with corresponding mechanical responses, suitable for educational projects or prototype systems involving color-based navigation or interaction.

## 🧱 Hardware Wiring
L298N Motor Driver:
    IN1, IN2 to Raspberry Pi GPIO pins -> Controls motor direction
    ENA connected through PWM -> Controls motor speed
    6V DC motor connected to OUT1 and OUT2

Servo Motor:
    PWM control wire connected to Raspberry Pi PWM pin -> Front-wheel steering control
    Powered from 5V rail

Color Sensor:
    Connect via I2C (SDA, SCL) -> Color detection to switch tasks

Camera Module:
    Use the CSI port on Raspberry Pi -> Direct ribbon connection to Pi

##  📦 Folder Structure
    📁 Lowgan/
    ├── 📁 3D Models/           # STL files for chassis and mounting
    ├── 📁 Source/
    │   ├── 📁 task1.py
    │   ├── 📁 task2.py
    │   └── 📁 task3.py
    ├── 📁 Team Photos/         # Documentation photos
    ├── 📁 Video/               # Recording of the vehicle in action
    ├── 📁 Wiring/              # Diagrams and schematics
    └── 📄 README.md            # Project description and documentation

## 🧠 Learning Impact
This project is a practical implementation of autonomous robotics concepts including:
    - PID control and PWM
    - Computer vision using OpenCV
    - Embedded I2C sensor integration
    - Parallel parking and navigation logic
    - Task switching based on environmental cues

It bridges theory and practice in AI-driven mechatronics and provides a hands-on learning experience in robotics and embedded systems.
