import time
import picamera
with picamera.PiCamera() as camera:
    camera.resolution=(640,480)
    camera.framerate=30
    camera.start_preview()
    try:
        for i, filename in enumerate(camera.capture_continuous('image{counter:02d}.jpg')):
            print(filename)
#            time.sleep(1)
            if i == 59:
                break
    finally:
        camera.stop_preview()
