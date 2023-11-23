from time import time
import cv2

class Camera(object):
    def __init__(self):
        self.frames = [open(f + '.jpg', 'rb').read() for f in ['1', '2', '3']]

        # open camera
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
        # set dimensions
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)

    def get_frame(self):
        return self.frames[int(time()) % 3]

    def get_actual_frame(self):
        # Take frame
        ret, frame = self.cap.read()

        if ret:
            # Convert frame to JPEG format
            _, buffer = cv2.imencode('.jpg', frame)

            # Return the byte array
            return buffer.tobytes()

        return None