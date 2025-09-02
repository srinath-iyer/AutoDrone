from flask import Flask, render_template, request, jsonify
from uart_esp32 import OnboardComms
app = Flask(__name__)
comms = OnboardComms()
# Default motor position (could be PWM value or angle)
motor_position = 0

@app.route('/')
def index():
    # Render the main control page
    return render_template('index.html')

@app.route('/set_motor', methods=['POST'])
def set_motor():
    global motor_position

    # Get the value from the slider (sent via fetch/AJAX)
    data = request.get_json()
    motor_position = data.get('value')
    comms.send_command(f"/motor-pwm/{motor_position}\n")
    # TODO: Interface with actual motor controller here
    # Example: pwm.set_duty_cycle(motor_position) or send via serial, etc.
    print(f"[DEBUG] Motor position set to: {motor_position}")

    # Respond with confirmation (could also send back status)
    return jsonify({'status': 'ok', 'value': motor_position})

if __name__ == '__main__':
    # Run the Flask development server (for production, use gunicorn/uwsgi + nginx)
    app.run(debug=True) # Default port of 5000
