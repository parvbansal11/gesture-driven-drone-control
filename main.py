import cv2
import mediapipe as mp
import time
import math

# CONFIG
CAMERA_INDEX = 0
WINDOW_NAME = "Gesture Drone (Dummy)"
EXIT_KEY = ord("q")

HOLD_TIME = 1.5       
CMD_COOLDOWN = 0.8   

PINCH_THRESHOLD = 0.05    
PINCH_HOLD_TIME = 1.0      

# MediaPipe setup
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

# Dummy Drone
class DummyDrone:
    def __init__(self):
        self.flying = False

    def takeoff(self):
        if not self.flying:
            self.flying = True
            print("[DRONE] TAKEOFF")

    def land(self):
        if self.flying:
            self.flying = False
            print("[DRONE] LAND")

    def move(self, direction: str):
        if self.flying:
            print(f"[DRONE] MOVE {direction}")


# Gesture Helpers
def get_finger_states(hand_landmarks):
    """
    Returns [thumb, index, middle, ring, pinky] as booleans.
    True = finger up, False = finger down.
    """
    lm = hand_landmarks.landmark

    tips = [4, 8, 12, 16, 20]
    pips = [3, 6, 10, 14, 18]

    fingers = []

    # Thumb: compare x (works well for mirrored selfie view in many cases)
    fingers.append(lm[tips[0]].x < lm[pips[0]].x)

    # Other fingers: tip is above pip (smaller y is "up")
    for t, p in zip(tips[1:], pips[1:]):
        fingers.append(lm[t].y < lm[p].y)

    return fingers


def classify_gesture(f):
    # f = [thumb, index, middle, ring, pinky]
    if f == [True, True, True, True, True]:
        return "PALM"
    if f == [False, False, False, False, False]:
        return "FIST"
    if f == [False, True, False, False, False]:
        return "POINT"
    if f == [False, True, True, False, False]:
        return "V_SIGN"
    return "OTHER"


def pinch_distance(hand_landmarks):
    """
    Distance between thumb tip (4) and index tip (8) in normalized coordinates.
    Smaller distance = pinch.
    """
    lm = hand_landmarks.landmark
    dx = lm[4].x - lm[8].x
    dy = lm[4].y - lm[8].y
    return math.sqrt(dx * dx + dy * dy)


# MAIN
def main():
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_AVFOUNDATION)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print("[ERROR] Camera not opening. Try CAMERA_INDEX=1 or check permissions.")
        return

    drone = DummyDrone()

    armed = False
    hold_start = None
    last_cmd_time = 0

    pinch_start = None
    precision_mode = False

    with mp_hands.Hands(
        max_num_hands=1,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.7
    ) as hands:

        while True:
            ok, frame = cap.read()
            if not ok:
                print("[ERROR] Failed to read frame")
                break

            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(rgb)

            gesture = "NONE"
            now = time.time()

            if results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # Gesture classification (finger rules)
                f = get_finger_states(hand_landmarks)
                gesture = classify_gesture(f)

                # Hold PALM to toggle ARMED
                if gesture == "PALM":
                    if hold_start is None:
                        hold_start = now
                    elif (now - hold_start) >= HOLD_TIME:
                        armed = not armed
                        hold_start = None
                        # turning SAFE should also turn off precision for clarity
                        if not armed:
                            precision_mode = False
                        print("[SYSTEM]", "ARMED ✅" if armed else "DISARMED ❌")
                else:
                    hold_start = None

                # Only allowed when armed
                d = pinch_distance(hand_landmarks)
                is_pinch = d < PINCH_THRESHOLD

                if armed:
                    if is_pinch:
                        if pinch_start is None:
                            pinch_start = now
                        elif (now - pinch_start) >= PINCH_HOLD_TIME and (now - last_cmd_time) >= CMD_COOLDOWN:
                            precision_mode = not precision_mode
                            pinch_start = None
                            print("[DRONE] 🎯 PRECISION MODE", "ON ✅" if precision_mode else "OFF ❌")
                            last_cmd_time = now
                    else:
                        pinch_start = None
                else:
                    pinch_start = None

                # Drone commands (armed + cooldown)
                if armed and (now - last_cmd_time) >= CMD_COOLDOWN:
                    if gesture == "PALM":
                        drone.takeoff()
                        last_cmd_time = now

                    elif gesture == "FIST":
                        drone.land()
                        last_cmd_time = now

                    elif gesture == "POINT":
                        # Precision changes behavior
                        if precision_mode:
                            drone.move("FORWARD_SLOW (Precision)")
                        else:
                            drone.move("FORWARD")
                        last_cmd_time = now

                    elif gesture == "V_SIGN":
                        if precision_mode:
                            drone.move("UP_SLOW (Precision)")
                        else:
                            drone.move("UP")
                        last_cmd_time = now

                # Visual target lock when precision mode is ON
                if precision_mode:
                    h, w = frame.shape[:2]
                    cx, cy = w // 2, h // 2
                    cv2.rectangle(frame, (cx - 70, cy - 70), (cx + 70, cy + 70), (0, 200, 255), 2)
                    cv2.putText(frame, "TARGET LOCK", (cx - 90, cy - 85),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
                    cv2.putText(frame, f"PinchDist: {d:.3f}", (20, 200),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)

            # HUD
            cv2.putText(frame, f"MODE: {'ARMED' if armed else 'SAFE'}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            cv2.putText(frame, f"GESTURE: {gesture}", (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.putText(frame, f"PRECISION: {'ON' if precision_mode else 'OFF'}", (20, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 255), 2)
            cv2.putText(frame, "Hold PALM 1.5s = Arm/Disarm | Pinch hold 1s = Precision", (20, 160),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 2)

            cv2.imshow(WINDOW_NAME, frame)

            if cv2.waitKey(1) & 0xFF == EXIT_KEY:
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()