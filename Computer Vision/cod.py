import cv2
import numpy as np

cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

state = 0
# 0 = no detection
# 1 = detected with cam, not with sensor
# 2 = detected with cam and sensor

def ChangeState(n):
    global state
    state = n

def State0(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    green_size = 0
    red_size = 0

    # --- RED COLOR RANGE (2 ranges for wrapping hue) ---
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    # --- GREEN COLOR RANGE ---
    lower_green = np.array([25, 80, 80])
    upper_green = np.array([45, 255, 255])

    # Create masks
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)

    green_mask = cv2.inRange(hsv, lower_green, upper_green)

    # Find contours for red
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(red_contours) == 0:
            red_size = 0
    else:
        red_contours = sorted(red_contours, key=cv2.contourArea, reverse=True)

        largest_contour = red_contours[0]
        if cv2.contourArea(largest_contour) > 1000:
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            red_size = cv2.contourArea(largest_contour)
            print('Red ' + str(red_size))
            if red_size > green_size:
                ChangeState(1)
                cv2.putText(frame, 'Red is Bigger', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                ChangeState(1)
                cv2.putText(frame, 'Red', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # Find contours for green
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(green_contours) == 0:
        green_size = 0
    else:
        green_contours = sorted(green_contours, key=cv2.contourArea, reverse=True)

        largest_contour = green_contours[0]

        if cv2.contourArea(largest_contour) > 1000:
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            green_size = cv2.contourArea(largest_contour)
            print('Green ' + str(green_size))
            if green_size > red_size:
                ChangeState(1)
                cv2.putText(frame, 'Green is Bigger', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                ChangeState(1)
                cv2.putText(frame, 'Green', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

def State1(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    green_size = 0
    red_size = 0

    # --- RED COLOR RANGE (2 ranges for wrapping hue) ---
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    # --- GREEN COLOR RANGE ---
    lower_green = np.array([25, 80, 80])
    upper_green = np.array([45, 255, 255])

    # Create masks
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)

    green_mask = cv2.inRange(hsv, lower_green, upper_green)

    # Find contours for red
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(red_contours) == 0:
            red_size = 0
    else:
        red_contours = sorted(red_contours, key=cv2.contourArea, reverse=True)

        largest_contour = red_contours[0]
        if cv2.contourArea(largest_contour) > 1000:
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            red_size = cv2.contourArea(largest_contour)
            print('Red ' + str(red_size))
            if red_size > green_size:
                cv2.putText(frame, 'Red is Bigger', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                cv2.putText(frame, 'Red', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # Find contours for green
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(green_contours) == 0:
        green_size = 0
    else:
        green_contours = sorted(green_contours, key=cv2.contourArea, reverse=True)

        largest_contour = green_contours[0]

        if cv2.contourArea(largest_contour) > 1000:
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            green_size = cv2.contourArea(largest_contour)
            print('Green ' + str(green_size))
            if green_size > red_size:
                cv2.putText(frame, 'Green is Bigger', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(frame, 'Green', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    if red_size > green_size:
        # da i maxim dreapta
        cv2.putText(frame, 'Go right', (0, 200), cv2.FONT_HERSHEY_SIMPLEX,
            1.0, (0, 0, 255), 1)
    elif green_size > red_size:
        # da i maxim stange
        cv2.putText(frame, 'Go left', (0, 200), cv2.FONT_HERSHEY_SIMPLEX,
            1.0, (0, 0, 255), 1)

def State2(frame):
    print('2')

def Loop():
    ret, frame = cap.read()
    if not ret:
        ChangeState(-1)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        ChangeState(-1)

    newState = 0

    if state == 0:
        State0(frame)
    if state == 1:
        State1(frame)
    if state == 2:
        State2(frame)

    cv2.putText(frame, 'State=' + str(state), (0, 100), cv2.FONT_HERSHEY_SIMPLEX,
            1.0, (0, 0, 255), 1)

    # Show the output
    cv2.imshow("PY", frame)

    if cv2.waitKey(1) & 0xFF == ord('f'):
        if state == 1:
            ChangeState(2)
        
        if state == 2:
            ChangeState(0)

    if cv2.waitKey(1) & 0xFF == ord('e'):
        if state == 0: ChangeState(1)
        elif state == 1: ChangeState(2)
        elif state == 2: ChangeState(0)

while True:
    Loop()    
    if state == -1:
        break

cap.release()
cv2.destroyAllWindows()
