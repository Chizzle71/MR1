import cv2
import numpy as np
from flask import Flask, Response

app = Flask(__name__)

def generate_frames():
    camera = cv2.VideoCapture(0)  # Will choose default camera camera

    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            # Make an orange filter
            # BGR not RGB
            orange = np.full(frame.shape, (0, 80, 255), dtype=np.uint8) 
            # Adds the orange/Mars colour to the frame (60/40)
            frame = cv2.addWeighted(frame, 0.6, orange, 0.4, 0)

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
        <head>
            <title>Webcam Stream</title>
            <style>
                body {
                    font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
                    background-color: #f2f2f2;
                    margin: 0;
                    padding: 0;
                    display: flex;
                    justify-content: center;
                    align-items: center;
                    height: 100vh;
                    flex-direction: column;
                }

                h1 {
                    color: #ff6600;
                    margin-bottom: 20px;
                }

                .video-container {
                    background-color: white;
                    padding: 20px;
                    border-radius: 10px;
                    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1);
                    border: 2px solid #ff6600;
                }

                img {
                    max-width: 100%;
                    height: auto;
                    border-radius: 5px;
                }

                footer {
                    margin-top: 30px;
                    color: #888;
                    font-size: 0.9em;
                }
            </style>
        </head>
        <body>
            <div class="video-container">
                <h1>Mars livestream</h1>
                <img src="/video_feed" width="640" height="480">
            </div>
            <footer></footer>
        </body>
    </html>
    '''

def set_host():
    app.run(host='0.0.0.0', port=5050, debug=True)

if __name__ == "__main__":
    set_host()
