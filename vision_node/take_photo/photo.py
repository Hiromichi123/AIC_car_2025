# -*- coding: utf-8 -*-
import cv2
import os
import time

# ----------------- User Settings -----------------

SAVE_DIR = "dataset_images"     # Folder to save captured images
FRAME_INTERVAL = 1              # Save one image every N frames
CAMERA_ID = 7                   # Camera ID (0 for USB camera usually)

# -------------------------------------------------

# Create the save directory if it does not exist
os.makedirs(SAVE_DIR, exist_ok=True)

# Open camera
cap = cv2.VideoCapture(CAMERA_ID)

# Set camera resolution (optional)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Check if camera opened successfully
if not cap.isOpened():
    print("Error: Could not open the camera. Check connection.")
    exit()

print("Camera started successfully. Press 'q' to quit.")

frame_count = 0

while True:
    # Read frame
    ret, frame = cap.read()

    if not ret:
        print("Warning: Failed to read frame. Camera may be disconnected.")
        break

    frame_count += 1

    # Show live preview
    cv2.imshow("Camera", frame)

    # Save frame every FRAME_INTERVAL frames
    if frame_count % FRAME_INTERVAL == 0:
        filename = os.path.join(SAVE_DIR, f"{int(time.time() * 1000)}.jpg")
        cv2.imwrite(filename, frame)
        print(f"Saved image: {filename}")

    # Quit if user presses 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
