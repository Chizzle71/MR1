import cv2
from flask import Flask, Response

app = Flask(__name__)

def generate_frames():
    camera = cv2.VideoCapture(0)  # Will choose normal camera

    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            # Convert frames to JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()

            # Sends across each frame
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    camera.release()  # Incase teh loop breaks

@app.route('/video_feed')  # Provides video feed and sends to webpage
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')  # Provides webpage setting
def index():
    return '''
        <html>
            <head><title>Mars livestream channel</title></head>
            <body>
                <h1>Mars livestream channel</h1>
                <img src="/video_feed" width="800" height="480">
            </body>
        </html>
    '''

def set_host():
    app.run(host='0.0.0.0', port=5050, debug=True)

if __name__ == "__main__":
    set_host()
