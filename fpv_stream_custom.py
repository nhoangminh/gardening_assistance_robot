import time
import cv2
import numpy as np
from flask import Flask, render_template, Response
from imutils.video.pivideostream import PiVideoStream
from picamera import PiCamera, PiCameraError
import webbrowser


# ====================================================================
class VideoCamera(object):
    def __init__(self, flip=False):
        self.vs = PiVideoStream().start()
        self.flip = flip
        time.sleep(2.0)

    def __del__(self):
        self.vs.stop()

    def flip_if_needed(self, frame):
        if self.flip:
            return np.flip(frame, 0)
        return frame

    def get_frame(self):
        frame = self.flip_if_needed(self.vs.read())
        ret, jpeg = cv2.imencode('.jpg', frame)
        return jpeg.tobytes()


def gen(camera):
    # get camera frame
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')


# ====================================================================
try:
    pi_camera = VideoCamera(flip=False)
except PiCameraError:
    print("PiCamera not working")

app = Flask(__name__)


@app.route("/")
def homepage():
    return render_template("index.html")


@app.route("/blog")
def about():
    webbrowser.open("https://blogs.ntu.edu.sg/ps9888-2021-g04/")


@app.route("/video")
def video_feed():
    return Response(gen(pi_camera),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


# ====================================================================
if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=False)
    print("Stream Closed")