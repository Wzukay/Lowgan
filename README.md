## 🤖 Lowgan - AI-Powered Autonomous Vehicle Navigation System
Welcome to the official repository of Lowgan, an advanced autonomous vehicle project developed for the WRO 2025 season. Lowgan integrates embedded intelligence, computer vision, and sensor fusion in a custom robotic car built on Raspberry Pi 5, designed to autonomously navigate a defined game mat. This system addresses realistic small-scale driving challenges through AI and real-time sensing.

This repository includes all resources needed to build, understand, and replicate Lowgan: hardware schematics, 3D models, and Python-based control and AI modules.

## 🧠 Project Overview
Lowgan is a compact AI-driven autonomous vehicle running on Raspberry Pi 5. It executes three progressive driving tasks autonomously, relying on onboard computing and sensors:

    - Stage 1: Circular Driving Around the Center
        Detects the mat's center using color and visual cues.
        Maintains a smooth circular path at a fixed radius.
        Dynamically adjusts steering and motor speed for stability.

    - Stage 2: Obstacle Avoidance
        Detects cube-shaped obstacles using front camera vision.
        Calculates safe trajectories and avoids collisions in real time.
        Uses sensor fusion and precise motor control for responsive maneuvering.

    - Stage 3: Lateral Parking
        Identifies parking zones using side clearances detected by vision and color sensors.
        Executes lateral (parallel) parking similar to urban driving.
        Combines vision input and sensor data to measure space and control parking steps.

The vehicle’s software consists of modular Python scripts that manage motor control, image processing, and sensor fusion for autonomous navigation.

## 🔩 Hardware Components
Lowgan integrates precise hardware with flexible software:

    - Raspberry Pi 5: Main processor running the OS and AI modules.
    - L298N Motor Driver: Controls a 6V rear DC motor with PWM for speed and direction.
    - SG90 Servo Motor: Steers the front wheels using PWM.
    - Camera Module: Provides live video for computer vision.
    - TCS34725 Color Sensor: Detects mat colors to trigger task transitions and aid navigation.
    - Battery Pack: Regulated power supply for all components.
    - 3D-Printed Chassis: Custom designed and printed from STL files (in /3D Models).

## 🔬 Engineering and Design Considerations
Steering and Vehicle Dynamics:

    - Steering Geometry: Based on Ackermann principles for realistic turning and minimized tire slip.
    - Servo Calibration: Ensures precise front-wheel angle control.
    - Smooth Transition Dynamics: Vital for circular driving and lateral parking maneuvers.
Center of Mass (CoM) Optimization:

    - Components arranged to lower the CoM, improving stability and reducing tipping risk.
    - Symmetrical weight distribution for predictable handling during obstacle avoidance and parking.
Physics and Motor Control:

    - Drag and Rolling Resistance: Modeled to refine power usage and maintain consistent speed.
    - Inertia and Momentum: Controlled acceleration and deceleration for accurate steering corrections and stopping.

## 🛠️ 3D Modeling Workflow
Lowgan’s mechanical parts were designed using two tools:

    - OpenSCAD: Script-based parametric modeling for core chassis parts, enabling version control and precise STL exports.
    - Tinkercad: Browser-based tool for quick prototyping and supplementary parts like sensor mounts and brackets.

Models are exported as STL files and stored in /3D Models. Modular design allows easy replacement and customization. Parts are printed in PLA filament, balancing weight and strength.

## 🚗 Mobility Management
Components:

    - DC Motor: Drives rear wheels, powered by Li-ion battery, controlled by PWM signals.
    - L298N Motor Driver: Interfaces motor with Raspberry Pi, managing speed and direction.
    - Raspberry Pi 5: Processes commands and sensor data to control motion.

Control Logic:

    - Motor speed controlled dynamically via PWM.
    - Direction managed by GPIO pins, enabling forward, reverse, and turns.
    - Supports scripted autonomous routines or remote manual control.

Mobility Features:

    - Differential drive system for sharp turns.
    - Speed limiting to prevent overload.
    - Failsafe to stop motors on signal loss or faults.

## 🧱 Obstacle Detection and Avoidance
Lowgan uses classical computer vision (OpenCV) for real-time obstacle detection:

    Camera: Front-mounted Raspberry Pi camera module captures video feed.
    Vision Processing:
        - Edge detection (Canny) to find contours.
        - Color thresholding (HSV filtering) to detect obstacles by color.
        - Contour analysis to estimate object size and location.
    Reactive Logic:
        - Stop immediately if obstacle detected.
        - Choose clear path direction by evaluating obstacle density.
        - Reverse and try alternate routes if blocked.

No heavy AI models are required; the system relies on lightweight, real-time CV.

## ⚡ Power and Sensor Integration
- Powered by a Li-ion battery with regulated voltage.
- Raspberry Pi manages computing and sensor operation.
- Motor driver supplies power to the DC motor.
- Color sensor (TCS34725) and servo motor powered directly from Pi.
- Battery management ensures stable and safe power delivery.

## 🔁 System Integration
Runs Raspberry Pi OS on Raspberry Pi 5.
Python scripts handle:

    - Camera and color sensor input.
    - Sensor fusion and vision processing.
    - Motion planning and motor control.

Modular architecture ensures clear data flow and easy maintenance.

## 💻 Software Overview
Lowgan software is modular and Python-based:

    - task1.py: Color-based circular navigation using TCS3200 sensor.
    - task2.py: Camera-based obstacle detection and avoidance.
    - task3.py: Combines obstacle avoidance with lateral parking using vision and color data.

task1.py Key Points:

    - Uses TCS3200 color sensor and servo motor to detect colors (orange, blue, white).
    - Controls servo steering based on color detection.
    - Runs DC motor forward or backward according to color.
    - Implements continuous loop with clean exit on interrupt.

task2.py Key Points:

    - Processes live camera feed via OpenCV to detect red and green objects.
    - Controls servo based on dominant color detected.
    - Motor speed and direction controlled via PWM.
    - Supports multiple operational states controlled by keyboard input.
    - Runs continuously with graceful shutdown.

task3.py Key Points:

    - Real-time color tracking of red and green objects.
    - Implements three operational states:
        State 0: Visual detection only.
        State 1: Active motor and servo control based on detected colors.
        State 2: Idle/no control.
    - User input switches between states.
    - Controls DC motor direction and servo position with PWM.
    - Clean exit and resource release supported.

## 🧱 Hardware Wiring Summary
L298N Motor Driver:

    - IN1, IN2: Raspberry Pi GPIO for motor direction.
    - ENA: PWM for speed control.
    - Connects to 6V DC motor.

Servo Motor:

    - PWM control to Raspberry Pi PWM pin.
    - Powered from 5V rail.

Color Sensor (TCS34725):

    - I2C connection (SDA, SCL).

Camera Module:

    - Connected via Raspberry Pi CSI port.

## 📦 Project Folder Structure
    Lowgan/
    ├── 3D Models/           # STL files for chassis and mounts
    ├── Source/
    │   ├── task1.py
    │   ├── task2.py
    │   └── task3.py
    ├── Team Photos/         # Project documentation photos
    ├── Video/               # Vehicle demonstration recordings
    ├── Wiring/              # Electrical diagrams and schematics
    └── README.md            # Project documentation

## 🛠️ Engineering Highlights
    - System Design: Embedded Raspberry Pi controller running vision processing and motor control.
    - Power System: Efficient LiPo battery with proper voltage regulation.
    - Motor Control: PWM-based speed and direction via L298N driver.
    - Obstacle Detection: Camera-based with lightweight OpenCV processing.
    - Mechanical: Lightweight 3D-printed chassis with balanced weight distribution.
    - Electrical: Compact wiring with power isolation for reliability.
    - Software: Modular Python scripts for maintainability and tuning.
    - Testing: Independent and integrated testing ensures stable performance.
    - Calibration: Simple routines adapt to light and surface variations.

## 🧠 Learning Outcomes
Lowgan is a hands-on robotics platform demonstrating:

    - PID and PWM motor control.
    - Real-time computer vision with OpenCV.
    - Embedded I2C sensor integration.
    - Autonomous navigation with task switching.
    - Practical AI-mechatronics applications.

Built with ❤️ by Team Lowgan — WRO 2025