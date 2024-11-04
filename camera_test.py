from picamzero import Camera
import time

# Initialize the camera
cam = Camera()

# Start preview if needed (optional)
cam.start_preview()

# Record video continuously until stopped
try:
    print("Recording... Press Ctrl+C to stop.")
    cam.record_video("new_video.mp4", duration=None)  # duration=None may not work in picamzero; check if supported
    while True:
        time.sleep(1)  # Keep the script alive
except KeyboardInterrupt:
    print("Stopping recording...")

# Stop the camera
cam.stop_preview()
