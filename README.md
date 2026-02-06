# gesture-driven-drone-control
Real-time hand gesture controlled dummy drone using MediaPipe and OpenCV, featuring a safety-based arming system and a unique pinch-and-hold precision mode.

Hand gestures captured from a webcam are interpreted and mapped to drone-like
commands such as takeoff, landing, and movement. A unique gesture is included
to demonstrate originality and safety-focused control.

---

## Features

- Real-time hand tracking using MediaPipe Hands
- Rule-based gesture recognition
- Safety mechanism using arming and disarming gestures
- Command cooldown to prevent accidental spamming
- Unique gesture: Pinch-and-hold to activate Precision Mode
- Dummy drone logic designed for easy future integration with real drone SDKs

---

## Gesture Controls

### Safety
- Open palm held for approximately 1.5 seconds toggles ARMED / SAFE mode

### Commands (active only when ARMED)
- Open palm: Takeoff
- Closed fist: Land
- Index finger up: Move forward
- Index and middle fingers up: Move upward

### Unique Gesture
- Pinch-and-hold (thumb tip touching index finger tip for about 1 second)
  toggles Precision Mode.
- Precision Mode enables slower, controlled movements and visual target lock.

---

## Requirements

- Python 3.10 or newer
- OpenCV
- MediaPipe

---

## Running the Project

Activate your virtual environment and run:

```bash
python main.py
