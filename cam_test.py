import cv2

cap = cv2.VideoCapture(0)
print("Opened:", cap.isOpened())

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to read frame")
        break
    cv2.imshow("Camera Test", frame)
    if cv2.waitKey(1) & 0xFF == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()