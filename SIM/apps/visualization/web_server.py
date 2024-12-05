from flask import Flask, render_template, jsonify, request
from threading import Thread
import os
import sys
import logging
from sim_time import SimTime

# Create Flask app and disable its logging
app = Flask(__name__)
app.logger.disabled = True
log = logging.getLogger('werkzeug')
log.disabled = True
cli = sys.modules['flask.cli']
cli.show_server_banner = lambda *x: None

# Store latest spacecraft state
current_state = {
    'lat': 0,
    'lon': 0,
    'velocity': [0, 0, 0],
    'orbit_path': [],
    'future_path': []
}

# Store server thread for clean shutdown
server_thread = None

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/state')
def get_state():
    return jsonify(current_state)

def update_state(lat, lon, velocity, orbit_path=None, future_path=None):
    """Update spacecraft state, handling date line crossing more efficiently"""
    global current_state
    current_state['lat'] = float(lat)
    current_state['lon'] = float(lon)
    current_state['velocity'] = velocity.tolist()
    current_state['time'] = SimTime.get_time().isoformat()
    
    # Handle orbit path updates more efficiently
    if orbit_path is not None:
        current_state['orbit_path'] = orbit_path
    if future_path is not None:
        # Reduce number of future path points
        if len(future_path) > 60:  # Keep only every 3rd point
            current_state['future_path'] = future_path[::3]
        else:
            current_state['future_path'] = future_path
    
    new_point = [float(lat), float(lon)]
    
    # Only keep last 60 points (30 minutes) instead of 2 hours
    if not current_state.get('orbit_path'):
        current_state['orbit_path'] = []
    
    if current_state['orbit_path']:
        last_point = current_state['orbit_path'][-1]
        last_lon = last_point[1]
        
        # Handle date line crossing more efficiently
        if abs(new_point[1] - last_lon) > 180:
            if new_point[1] < last_lon:
                current_state['orbit_path'].append([new_point[0], -180])
                current_state['orbit_path'].append([new_point[0], 180])
            else:
                current_state['orbit_path'].append([new_point[0], 180])
                current_state['orbit_path'].append([new_point[0], -180])
    
    current_state['orbit_path'].append(new_point)
    
    # Keep only last 60 points instead of 7200
    if len(current_state['orbit_path']) > 60:
        current_state['orbit_path'] = current_state['orbit_path'][-60:]

def start_server():
    """Start the web server in a background thread"""
    global server_thread
    os.makedirs('apps/visualization/templates', exist_ok=True)
    server_thread = Thread(target=lambda: app.run(port=8089, debug=False))
    server_thread.daemon = True  # Thread will be killed when main program exits
    server_thread.start()

def stop_server():
    """Stop the web server cleanly"""
    func = request.environ.get('werkzeug.server.shutdown')
    if func is None:
        raise RuntimeError('Not running with the Werkzeug Server')
    func()