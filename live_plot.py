import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objs as go
import os

LOG_FILE = "log.txt"  # Update to your actual log path

app = dash.Dash(__name__)
app.layout = html.Div([
    html.H2("Live Pose (XYZ) from Log File"),
    dcc.Graph(id='pose-graph'),
    dcc.Interval(id='interval', interval=1000, n_intervals=0)  # Refresh every 1 sec
])

def parse_log(filepath):
    x_vals, y_vals = [], []
    timestamps = []

    if not os.path.exists(filepath):
        return x_vals, y_vals, timestamps

    with open(filepath, 'r') as f:
        for line in f:
            try:
                vals = [float(v.strip()) for v in line.split(',')]
                if len(vals) < 3:
                    continue
                timestamps.append(vals[0])
                x_vals.append(vals[1])
                y_vals.append(vals[2])
            except Exception as e:
                print("Parse error:", e)
                continue
    # Normalize timestamps for plotting
    if timestamps:
        base = timestamps[0]
        timestamps = [(t - base) for t in timestamps]  # ms → seconds
    return x_vals, y_vals, timestamps

@app.callback(Output('pose-graph', 'figure'),
              Input('interval', 'n_intervals'))
def update_plot(n):
    x_vals, y_vals, t_vals = parse_log(LOG_FILE)

    return {
        'data': [
            go.Scatter(x=t_vals, y=x_vals, name='X', mode='lines'),
            go.Scatter(x=t_vals, y=y_vals, name='Y', mode='lines')
        ],
        'layout': go.Layout(
            title='Pose: X, Y vs Time',
            xaxis={'title': 'Time (s)'},
            yaxis={'title': 'Meters/S'},
            margin={'l': 40, 'r': 10, 't': 40, 'b': 40}
        )
    }

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')
