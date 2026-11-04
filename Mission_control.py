# Imports
import math
import time
import dash
import cv2
import dash_bootstrap_components as dbc
import plotly.graph_objs as go
from typing import Optional
from collections.abc import Generator
from dash import html, dcc, Input, Output
from flask import Response, Flask
from plotly.graph_objs import Figure
from picamera import PiCamera
from picamera.array import PiRGBArray

# Variables
distance_travelled = 0.0
displacement = 0.0
current_action = "STOP"
speed = 0.005
rotate_speed = math.pi/80  # Radians per tick
rotate_speed2 = math.pi/160  # Radians per tick
angle = 0.0  # Rover heading in radians
x, y = 0.0, 0.0  # Rover position relative to landing site

# Sensor data
temperature_data = []   # Append temperature values here from sensors.py

# Initialise webpage
server = Flask(__name__)
app = dash.Dash(__name__, server=server,
                external_stylesheets=[dbc.themes.DARKLY])
app.title = "Mars Rover Mission Control"

# Picamera setup and convert format to MJPEG
def generate_frames() -> Generator[bytes, None, None]:
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 24
    rawCapture = PiRGBArray(camera, size=(640, 480))

    # Warm up camera
    time.sleep(2)

    for frame in camera.capture_continuous(rawCapture, format="bgr",
                                           use_video_port=True):
        image = frame.array
        _, buffer = cv2.imencode('.jpg', image)
        rawCapture.truncate(0)

        yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n'
                + buffer.tobytes() + b'\r\n')
        
@server.route('/video_feed')
def video_feed() -> Response:
    """Flask route that streams the MJPEG video feed"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# Webpage formatting
app.layout = dbc.Container([
    html.H2("Mars Rover Mission Control", className="text-center mt-4 mb-4"),

    dbc.Row([
        # Video feed and location information beneath video
        dbc.Col([
            html.H5("Live Video Feed"),
            html.Img(src="/video_feed", style={"width": "100%", "border": "2px solid #555", "border-radius": "5px"}),
            html.P("Distance travelled: 0.0 metres", id="distance-display", className="mt-2"),
            html.P("Distance from landing site: 0.0 metres", id="displacement-display", className="mt-2"),
            html.P("Bearing", id="bearing", className="mt-2")
        ], width=7),

        # Right panel for telemetry and controls
        dbc.Col([
            html.H5("Telemetry Data"),
            dcc.Graph(
                id='temperature-graph',
                style={"height": "300px"}  # Customisable height
            ),

            html.Hr(),

            html.H5("Rover Controls"),
            dbc.Container([
                # Pivot left and right
                dbc.Row([
                    dbc.Col(dbc.Button("Rotate Left", id="btn-rotate-left", color="primary", className="w-100"), width=4),
                    dbc.Col(),
                    dbc.Col(dbc.Button("Rotate Right", id="btn-rotate-right", color="primary", className="w-100"), width=4),
                ], className="mb-2"),

                # Empty rows
                dbc.Row([], className="mb-2"),
                dbc.Row([], className="mb-2"),

                # Ackermann forwards
                dbc.Row([
                    dbc.Col(dbc.Button("Ackermann forward left", id="btn-forward-rotate-left", color="primary", className="w-100"), width=5),
                    dbc.Col(),
                    dbc.Col(dbc.Button("Ackermann forward right", id="btn-forward-rotate-right", color="primary", className="w-100"), width=5),
                ], className="mb-2"),

                # Forward left, forward, forward right
                dbc.Row([
                    dbc.Col(dbc.Button("Forward left", id="btn-forward-left", color="secondary", className="w-100"), width=4),
                    dbc.Col(dbc.Button("Forward", id="btn-forward", color="secondary", className="w-100"), width=4),
                    dbc.Col(dbc.Button("Forward right", id="btn-forward-right", color="secondary", className="w-100"), width=4),
                ], className="mb-2"),

                # Left, stop, right
                dbc.Row([
                    dbc.Col(dbc.Button("Left", id="btn-left", color="secondary", className="w-100"), width=4),
                    dbc.Col(dbc.Button("Stop", id="btn-stop", color="danger", className="w-100"), width=4),
                    dbc.Col(dbc.Button("Right", id="btn-right", color="secondary", className="w-100"), width=4),
                ], className="mb-2"),

                # Backward left, backward, backward right
                dbc.Row([
                    dbc.Col(dbc.Button("Backward left", id="btn-backward-left", color="secondary", className="w-100"), width=4),
                    dbc.Col(dbc.Button("Backward", id="btn-backward", color="secondary", className="w-100"), width=4),
                    dbc.Col(dbc.Button("Backward right", id="btn-backward-right", color="secondary", className="w-100"), width=4),
                ], className="mb-2"),

                # Ackermann backwards
                dbc.Row([
                    dbc.Col(dbc.Button("Ackermann backward left", id="btn-backward-rotate-left", color="primary", className="w-100"), width=5),
                    dbc.Col(),
                    dbc.Col(dbc.Button("Ackermann backward right", id="btn-backward-rotate-right", color="primary", className="w-100"), width=5),
                ], className="mb-2"),
            ]),

            html.Hr(),

            dbc.Button("Take Picture", id="btn-picture", color="warning", className="w-100"),
            html.Div(id="action-status", className="mt-3", style={"font-weight": "bold"})
        ], width=5),
    ], className="mb-5"),

    dcc.Interval(id='interval-component', interval=2000, n_intervals=0)
], fluid=True)

# Refresh the telemetry on the dashboard
@app.callback(
    Output('temperature-graph', 'figure'),
    Input('interval-component', 'n_intervals')
)

def update_temperature_graph(n: int) -> Figure:
    """
    Update the rovers temperature graph for the dashboard

    Args:
        n (int): Interval tick count from the Dash `dcc.Interval` component

    Returns:
        Figure: A Plotly figure showing the last 50 temperature readings
    """
    # Wait for new telemetry values to be appended by sensors.py
    if len(temperature_data) == 0:
        y_data = [None]
    else:
        y_data = temperature_data[-50:]  # To maintain graph at max length 50

    fig = go.Figure(go.Scatter(
        y=y_data,
        mode='lines+markers',
        line=dict(color='orange')
    ))
    fig.update_layout(
        title="Rover Temperature (°C)",
        yaxis=dict(range=[10, 40]),  # Range
        template='plotly_dark',
        margin=dict(t=30, l=0, r=0, b=0)
    )
    return fig

@app.callback(
    [Output("action-status", "children"),
     Output("distance-display", "children"),
     Output("displacement-display", "children"),
     Output("bearing", "children")],
    [
        # Interval ticks
        Input("interval-component", "n_intervals"),

        # Control buttons
        Input("btn-forward", "n_clicks"),
        Input("btn-forward-right", "n_clicks"),
        Input("btn-forward-left", "n_clicks"),
        Input("btn-backward", "n_clicks"),
        Input("btn-backward-right", "n_clicks"),
        Input("btn-backward-left", "n_clicks"),
        Input("btn-left", "n_clicks"),
        Input("btn-right", "n_clicks"),
        Input("btn-rotate-left", "n_clicks"),
        Input("btn-rotate-right", "n_clicks"),
        Input("btn-forward-rotate-left", "n_clicks"),
        Input("btn-forward-rotate-right", "n_clicks"),
        Input("btn-backward-rotate-left", "n_clicks"),
        Input("btn-backward-rotate-right", "n_clicks"),
        Input("btn-stop", "n_clicks"),
        Input("btn-picture", "n_clicks")
    ]
)

def update_action_and_distance(
    n_intervals: int,
    forward: Optional[int],
    forward_right: Optional[int],
    forward_left: Optional[int],
    backward: Optional[int],
    backward_right: Optional[int],
    backward_left: Optional[int],
    left: Optional[int],
    right: Optional[int],
    rot_left: Optional[int],
    rot_right: Optional[int],
    forward_rot_left: Optional[int],
    forward_rot_right: Optional[int],
    backward_rot_left: Optional[int],
    backward_rot_right: Optional[int],
    stop: Optional[int],
    picture: Optional[int],
    ) -> tuple[str, str, str, str]:
    """
    Update the rovers action, distance travelled, displacement, and bearing

    This function is triggered either by control button clicks or by the
    `dcc.Interval` component ticks and it updates the rover's position and
    heading based on the current action

    Args:
        n_intervals (int): Tick count from the Dash interval component
        forward, forward_right, forward_left, backward, backward_right,
        backward_left, left, right, rot_left, rot_right,
        forward_rot_left, forward_rot_right, backward_rot_left,
        backward_rot_right, stop, picture: Click counts
        from the corresponding control buttons. Will be none if the
        button has never been clicked

    Returns:
        tuple[str, str, str, str]: A tuple containing:
            - Current action description
            - Distance travelled (metres)
            - Displacement from landing site (metres)
            - Bearing in degrees
    """
    global distance_travelled, current_action, displacement, angle
    global rotate_speed2, rotate_speed, x, y

    ctx = dash.callback_context

    # Increments applied on interval ticks
    increments = {
        "f": speed, "fr": speed, "fl": speed,
        "b": speed, "br": speed, "bl": speed,
        "l": speed, "r": speed,
        "fpl": 0.9*speed, "fpr": 0.9*speed,
        "bpl": 0.9*speed, "bpr": 0.9*speed,
        "pl": 0.0, "pr": 0.0,
        "STOP": 0.0,
        "TAKE_PICTURE": 0.0
    }
    increments2 = {
        "f": 0.0, "fr": 0.0, "fl": 0.0,
        "b": 0.0, "br": 0.0, "bl": 0.0,
        "l": 0.0, "r": 0.0,
        "fpl": -rotate_speed2, "fpr": rotate_speed2,
        "bpl": -rotate_speed2, "bpr": rotate_speed2,
        "pl": -rotate_speed, "pr": rotate_speed,
        "STOP": 0.0,
        "TAKE_PICTURE": 0.0
    }

    # If triggered by a button then update action
    if ctx.triggered and "btn-" in ctx.triggered[0]['prop_id']:
        button_id = ctx.triggered[0]['prop_id'].split('.')[0]

        action_map = {
            "btn-forward": "f",
            "btn-forward-right": "fr",
            "btn-forward-left": "fl",
            "btn-backward": "b",
            "btn-backward-right": "br",
            "btn-backward-left": "bl",
            "btn-left": "l",
            "btn-right": "r",
            "btn-rotate-left": "pl",
            "btn-rotate-right": "pr",
            "btn-forward-rotate-left": "fpl",
            "btn-forward-rotate-right": "fpr",
            "btn-backward-rotate-left": "bpl",
            "btn-backward-rotate-right": "bpr",
            "btn-stop": "STOP",
            "btn-picture": "TAKE_PICTURE"
        }

        current_action = action_map.get(button_id, "STOP")
        print(f"[Command Sent] {current_action}")

    # If triggered by interval then increment distance
    if ctx.triggered and "interval-component" in ctx.triggered[0]['prop_id']:

        # Increment distance
        distance_travelled += increments.get(current_action, 0.0)

        # Update angle
        angle += increments2.get(current_action, 0.0)

        # Movement relative to heading
        if current_action == "f":
            x += math.cos(angle) * increments.get(current_action, 0.0)
            y += math.sin(angle) * increments.get(current_action, 0.0)
        elif current_action in ["fr"]:
            x += math.cos(angle + math.pi/4) * increments.get(current_action, 0.0)
            y += math.sin(angle + math.pi/4) * increments.get(current_action, 0.0)
        elif current_action in ["fl"]:
            x += math.cos(angle - math.pi/4) * increments.get(current_action, 0.0)
            y += math.sin(angle - math.pi/4) * increments.get(current_action, 0.0)
        elif current_action in ["r"]:
            x += math.cos(angle + math.pi/2) * increments.get(current_action, 0.0)
            y += math.sin(angle + math.pi/2) * increments.get(current_action, 0.0)
        elif current_action in ["l"]:
            x += math.cos(angle - math.pi/2) * increments.get(current_action, 0.0)
            y += math.sin(angle - math.pi/2) * increments.get(current_action, 0.0)
        elif current_action in ["br"]:
            x += math.cos(angle + 3 * math.pi/4) * increments.get(current_action, 0.0)
            y += math.sin(angle + 3 * math.pi/4) * increments.get(current_action, 0.0)
        elif current_action in ["bl"]:
            x += math.cos(angle - 3 * math.pi/4) * increments.get(current_action, 0.0)
            y += math.sin(angle - 3 * math.pi/4) * increments.get(current_action, 0.0)
        elif current_action in ["b"]:
            x -= math.cos(angle) * increments.get(current_action, 0.0)
            y -= math.sin(angle) * increments.get(current_action, 0.0)
        elif current_action in ["fpr"]:
            x += math.cos(angle) * increments.get(current_action, 0.0)
            y += math.sin(angle) * increments.get(current_action, 0.0)
        elif current_action in ["fpl"]:
            x += math.cos(angle) * increments.get(current_action, 0.0)
            y += math.sin(angle) * increments.get(current_action, 0.0)
        elif current_action in ["bpr"]:
            x -= math.cos(angle) * increments.get(current_action, 0.0)
            y -= math.sin(angle) * increments.get(current_action, 0.0)
        elif current_action in ["bpl"]:
            x -= math.cos(angle) * increments.get(current_action, 0.0)
            y -= math.sin(angle) * increments.get(current_action, 0.0)

        displacement = math.sqrt(x**2 + y**2)

    return (
        f"Action: {current_action}",
        f"Distance travelled: {distance_travelled:.4f} metres",
        f"Distance from landing site: {displacement:.4f} metres",
        f"Bearing: {(180/math.pi * angle):.1f} degrees"
    )

# Run - shouldn't be used, should be ran from some main file
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
