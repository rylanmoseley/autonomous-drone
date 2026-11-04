import cv2
import time
import numpy as np

# --- Debouncing & State Variables ---
shape_order = []      
order_complete = False

current_candidate = None
candidate_start_time = None
DEBOUNCE_SECONDS = 1.5 

STATE_ORDER = "ORDER"
STATE_IN_FLIGHT = "IN_FLIGHT"
state = STATE_ORDER

POST_ORDER_HOLD_SECONDS = 5.0
order_locked_time = None
last_order_polarity = None


def classify(contour):
    area = cv2.contourArea(contour)
    if area < 800:
        return None

    perim = cv2.arcLength(contour, True)
    if perim == 0:
        return None

    circularity = 4.0 * np.pi * area / (perim * perim)

    (_, _),radius = cv2.minEnclosingCircle(contour)
    circle_area = np.pi * radius * radius
    fill_ratio = area / circle_area if circle_area > 0 else 0

    eps = 0.01 * perim
    approx = cv2.approxPolyDP(contour, eps, True)
    n = len(approx)

    if n >= 7 and circularity > 0.75 and fill_ratio > 0.7:
        return "Circle"
    
    if n == 3:
        return "Triangle"
    elif n == 4:
        _, _, w, h = cv2.boundingRect(approx)
        ar = w / float(h)
        return "Square" if 0.9 < ar < 1.1 else "Rectangle"
    elif n == 5:
        _, _, w, h = cv2.boundingRect(approx)
        ar = w / float(h)
        if 0.7 < ar < 1.3 and circularity > 0.3:
            return "Pentagon"
    elif n == 6:
        _, _, w, h = cv2.boundingRect(approx)
        ar = w / float(h)
        if 0.7 < ar < 1.3 and circularity > 0.3:
            return "Hexagon"
            
    return None

def threshold_auto_polarity(gray):
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, th_bin = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    _, th_inv = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    kernel = np.ones((5,5), np.uint8)
    th_bin = cv2.morphologyEx(th_bin, cv2.MORPH_OPEN, kernel, iterations=1)
    th_inv = cv2.morphologyEx(th_inv, cv2.MORPH_OPEN, kernel, iterations=1)

    return th_bin, th_inv

def process_frame(image):
    global last_order_polarity
    
    h, w = image.shape[:2]
    max_area = 0.9 * h * w 

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    th_bin, th_inv = threshold_auto_polarity(gray)

    def score_mask(th):
        contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = 0.0
        for c in contours:
            area = cv2.contourArea(c)
            if area < 800 or area > max_area:
                continue
            perim = cv2.arcLength(c, True)
            if perim <= 0:
                continue
            circ = 4.0 * np.pi * area / (perim * perim)
            best = max(best, area * min(circ, 1.0))
        return best

    score_bin = score_mask(th_bin)
    score_inv = score_mask(th_inv)
  
    th = th_inv if score_inv > score_bin else th_bin

    pol = "INV" if score_inv > score_bin else "BIN"
    if pol != last_order_polarity:
        last_order_polarity = pol

    contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detected_labels = []
    annotated = image.copy()

    for c in contours:
        area = cv2.contourArea(c)
        if area < 800 or area > max_area:
            continue

        label = classify(c)
        if not label:
            continue

        if label not in detected_labels:
            detected_labels.append(label)

        cv2.drawContours(annotated, [c], -1, (0, 255, 0), 3)

        M = cv2.moments(c)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.putText(annotated, label, (cx - 40, cy), cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 0, 0), 2)

    return annotated, detected_labels, th

def identify_order(detected_shapes_frame):
    global shape_order, order_complete
    global current_candidate, candidate_start_time

    if order_complete:
        return shape_order

    if not detected_shapes_frame:
        current_candidate = None
        candidate_start_time = None
        return shape_order

    candidate = None
    for s in detected_shapes_frame:
        if s not in shape_order:
            candidate = s
            break

    if candidate is None:
        return shape_order

    now = time.time() 
    if candidate != current_candidate:
        current_candidate = candidate
        candidate_start_time = now
        return shape_order

    if candidate_start_time is not None:
        elapsed = now - candidate_start_time
        if elapsed >= DEBOUNCE_SECONDS:
            shape_order.append(current_candidate)
            current_candidate = None
            candidate_start_time = None

            if len(shape_order) == 3:
                order_complete = True

    return shape_order

# ----------------- GATE DETECTION (IN_FLIGHT) -----------------
def preprocess_for_contours(frame, use_clahe=False):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if use_clahe:
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, th = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    kernel = np.ones((5,5), np.uint8)
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel, iterations=1)
    th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel, iterations=2)
    return th

def contour_centroid(contour):
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return None
    return (M["m10"] / M["m00"], M["m01"] / M["m00"])

def pick_best_gate_hole(contours, hierarchy, frame_shape):
    if hierarchy is None or len(contours) == 0:
        return None, None

    H, W = frame_shape[:2]
    hier = hierarchy[0]
    best_parent, best_child = None, None
    best_score = -1e18

    for parent_idx, parent in enumerate(contours):
        parent_area = cv2.contourArea(parent)
        if parent_area < 1000 or parent_area > 0.98 * H * W:
            continue

        child_idx = hier[parent_idx][2]
        while child_idx != -1:
            child = contours[child_idx]
            child_area = cv2.contourArea(child)

            if child_area >= 800:
                hole_ratio = child_area / float(parent_area)
                if 0.15 <= hole_ratio <= 0.95:
                    cxy = contour_centroid(child)
                    if cxy is not None:
                        cx, cy = cxy
                        center_dist = ((cx - W/2)**2 + (cy - H/2)**2) ** 0.5
                        score = child_area - (0.7 * center_dist)
                        if score > best_score:
                            best_score = score
                            best_parent = parent
                            best_child = child
            child_idx = hier[child_idx][0]

    return best_parent, best_child

def classify_opening(contour):
    area = cv2.contourArea(contour)
    if area < 800: return None
    perim = cv2.arcLength(contour, True)
    if perim == 0: return None
    circularity = 4.0 * np.pi * area / (perim * perim)
    eps = 0.01 * perim
    approx = cv2.approxPolyDP(contour, eps, True)
    n = len(approx)

    if n >= 7 and circularity > 0.80: return "Circle"
    if n == 3: return "Triangle"
    if n == 4:
        _, _, w, h = cv2.boundingRect(approx)
        return "Square" if 0.9 < (w / float(h)) < 1.1 else "Rectangle"
    if n == 5: return "Pentagon"
    if n == 6: return "Hexagon"
    return None

def detect_gate_inner(frame):
    H, W = frame.shape[:2]
    th = preprocess_for_contours(frame, use_clahe=True)
    contours, hierarchy = cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    outer, inner = pick_best_gate_hole(contours, hierarchy, frame.shape)
    
    debug = frame.copy()
    if inner is None:
        cv2.putText(debug, "NO GATE DETECTED", (20, 30), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 0, 255), 2)
        return debug, th

    label = classify_opening(inner)
    cxy = contour_centroid(inner)
    if label is None or cxy is None:
        cv2.putText(debug, "FOUND HOLE, FAILED CLASSIFY", (20, 30), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 0, 255), 2)
        return debug, th

    cx, cy = cxy
    cv2.drawContours(debug, [outer], -1, (0, 255, 0), 2)
    cv2.drawContours(debug, [inner], -1, (255, 0, 0), 2)
    cv2.circle(debug, (int(cx), int(cy)), 4, (0, 0, 255), -1)
    
    ex = (cx - W/2) / (W/2)
    ey = (cy - H/2) / (H/2)
    cv2.putText(debug, f"{label}  ex={ex:.2f} ey={ey:.2f}", (20, 30), cv2.FONT_HERSHEY_DUPLEX, 0.7, (255, 255, 255), 2)
    return debug, th

# ----------------- MAIN LOOP -----------------
def main():
    global shape_order, order_complete
    global current_candidate, candidate_start_time
    global state, order_locked_time

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        print("Error: could not open camera.")
        return

    print("--- Shaper-Drone Vision System (Python) ---")
    print("KEYS: 'q'=quit, 'r'=reset, 'n'=force next mode")

    while True:
        ret, frame = cap.read()
        if not ret: break

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'): break
        if key == ord('n'): state = STATE_IN_FLIGHT
        if key == ord('r'):
            shape_order = []
            order_complete = False
            current_candidate = None
            candidate_start_time = None
            order_locked_time = None
            state = STATE_ORDER

        # 1. ORDER LEARNING MODE
        if state == STATE_ORDER:
            annotated, frame_labels, th = process_frame(frame)
            identify_order(frame_labels)

            now = time.time()
            if order_complete and order_locked_time is None:
                order_locked_time = now

            if not order_complete:
                order_locked_time = None

            order_text = "Order: " + " -> ".join(shape_order) if shape_order else "Order: (learning...)"
            cv2.putText(annotated, order_text, (20, 30), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 0, 255), 2)

            if order_complete:
                elapsed = now - order_locked_time if order_locked_time else 0.0
                cv2.putText(annotated, f"ORDER LOCKED - holding {elapsed:0.1f}/{POST_ORDER_HOLD_SECONDS:.1f}s (press 'n')",
                            (20, 60), cv2.FONT_HERSHEY_DUPLEX, 0.65, (0, 200, 0), 2)
                if elapsed >= POST_ORDER_HOLD_SECONDS:
                    state = STATE_IN_FLIGHT

            cv2.imshow("ShaperDrone Vision", annotated)
            cv2.imshow("Processing Mask", th)

        # 2. IN-FLIGHT GATE MODE
        elif state == STATE_IN_FLIGHT:
            debug, th = detect_gate_inner(frame)
            
            cv2.putText(debug, "IN FLIGHT (Gate Hole Code)", (20, 60), cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 200, 0), 2)
            if shape_order:
                cv2.putText(debug, "Mission Order: " + " -> ".join(shape_order), (20, 90), cv2.FONT_HERSHEY_DUPLEX, 0.7, (255, 255, 255), 2)

            cv2.imshow("ShaperDrone Vision", debug)
            cv2.imshow("Processing Mask", th)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
