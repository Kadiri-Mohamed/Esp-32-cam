import cv2
import numpy as np
import urllib.request
import time

ESP32_IP = "192.168.137.57"  # 🔴 CHANGE THIS

# =========================
# COLOR RANGES
# =========================
COLOR_RANGES = {
    "red1": ([0, 120, 70], [10, 255, 255]),
    "red2": ([170, 120, 70], [180, 255, 255]),
    "green": ([36, 100, 100], [86, 255, 255]),
    "yellow": ([15, 100, 100], [35, 255, 255]),
}

# =========================
# GET FRAME (VERY STABLE)
# =========================
def get_frame():
    try:
        url = f"http://{ESP32_IP}/capture"
        req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
        resp = urllib.request.urlopen(req, timeout=2)
        img = np.array(bytearray(resp.read()), dtype=np.uint8)
        frame = cv2.imdecode(img, cv2.IMREAD_COLOR)
        return frame
    except:
        return None

# =========================
# DETECTION FUNCTION
# =========================
def detect_color(frame, color):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower = np.array(COLOR_RANGES[color][0])
    upper = np.array(COLOR_RANGES[color][1])

    mask = cv2.inRange(hsv, lower, upper)

    # Clean noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)

        if area > 5000:  # 🔥 strong filter
            x, y, w, h = cv2.boundingRect(c)

            # Draw box
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)

            center = (x + w//2, y + h//2)
            cv2.circle(frame, center, 5, (0,0,255), -1)

            cv2.putText(frame, color, (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

            return True

    return False

# =========================
# MAIN LOOP
# =========================
last_action = None
last_time = 0

print("🚀 Ultra Stable Detection Started")

while True:
    frame = get_frame()

    if frame is None:
        print("❌ Frame error (retrying...)")
        time.sleep(0.2)
        continue

    # Resize (reduce load)
    frame = cv2.resize(frame, (640, 480))

    # Blur (reduce noise)
    frame = cv2.GaussianBlur(frame, (5,5), 0)

    detected = None

    # -------- RED --------
    if detect_color(frame, "red1") or detect_color(frame, "red2"):
        detected = "RIGHT"
        label = "RED"

    # -------- GREEN --------
    elif detect_color(frame, "green"):
        detected = "FORWARD"
        label = "GREEN"

    # -------- YELLOW --------
    elif detect_color(frame, "yellow"):
        detected = "LEFT"
        label = "YELLOW"

    # -------- STABLE OUTPUT --------
    if detected:
        if detected != last_action or time.time() - last_time > 1:
            print(f"{label} → {detected}")
            last_action = detected
            last_time = time.time()

    cv2.imshow("ESP32-CAM", frame)

    if cv2.waitKey(1) == 27:
        break

    # small delay to avoid overload
    time.sleep(0.05)

cv2.destroyAllWindows()