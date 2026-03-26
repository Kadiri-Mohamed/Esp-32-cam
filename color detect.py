import cv2
import numpy as np
import time

# ==============================
# COLOR RANGES (HSV)
# ==============================
COLOR_RANGES = {
    "red1": ([0, 120, 70], [10, 255, 255]),
    "red2": ([170, 120, 70], [180, 255, 255]),  # important for red
    "green": ([36, 100, 100], [86, 255, 255]),
    "yellow": ([15, 100, 100], [35, 255, 255]),
}

# ==============================
# DETECT FUNCTION
# ==============================
def detect_color(frame, color_name):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower = np.array(COLOR_RANGES[color_name][0])
    upper = np.array(COLOR_RANGES[color_name][1])

    mask = cv2.inRange(hsv, lower, upper)

    # Clean noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        c = max(contours, key=cv2.contourArea)

        if cv2.contourArea(c) > 1500:
            x, y, w, h = cv2.boundingRect(c)

            # Draw rectangle
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

            # Center
            center = (x + w//2, y + h//2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            cv2.putText(frame, color_name, (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

            return True, center

    return False, None


# ==============================
# MAIN
# ==============================
ESP32_IP = "192.168.137.57"   # 🔴 CHANGE THIS
URL = f"http://{ESP32_IP}:81/stream"

cap = cv2.VideoCapture(URL)

if not cap.isOpened():
    print("❌ Cannot open camera")
    exit()

print("✅ Camera connected")

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Frame error")
        continue

    detected_action = None

    # ------------------------------
    # RED (combine two masks)
    # ------------------------------
    found1, center1 = detect_color(frame, "red1")
    found2, center2 = detect_color(frame, "red2")

    if found1 or found2:
        print("🔴 RED → TURN RIGHT")
        detected_action = "RIGHT"

    # ------------------------------
    # GREEN
    # ------------------------------
    else:
        found, center = detect_color(frame, "green")
        if found:
            print("🟢 GREEN → FORWARD")
            detected_action = "FORWARD"

    # ------------------------------
    # YELLOW
    # ------------------------------
    if detected_action is None:
        found, center = detect_color(frame, "yellow")
        if found:
            print("🟡 YELLOW → LEFT")
            detected_action = "LEFT"

    # Show result
    cv2.imshow("ESP32-CAM Detection", frame)

    key = cv2.waitKey(1)
    if key == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()