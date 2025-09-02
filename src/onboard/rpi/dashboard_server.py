from flask import Flask, Response, render_template
import cv2
import time

app = Flask(__name__)
pipeline = None  # global reference to CVPipeline instance

def gen_frames():
    while True:
        if pipeline and pipeline.latest_debug_frame is not None:
            _, buffer = cv2.imencode('.jpg', pipeline.latest_debug_frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(1 / 30.0)

@app.route('/')
def index():
    return render_template('dashboard.html')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def start_server(cv_pipeline):
    global pipeline
    pipeline = cv_pipeline
    app.run(host='0.0.0.0', port=8000, debug=False, threaded=True)
