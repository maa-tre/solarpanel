
# ML-Enhanced Server - Isolated Version
from flask import Flask, request, jsonify, render_template_string, Response
from flask_cors import CORS
import os, csv, glob, pandas as pd
from datetime import datetime, timedelta
import logging
import threading
import time
import json

# Try importing ML libraries
try:
    import joblib
    ML_AVAILABLE = True
except ImportError:
    ML_AVAILABLE = False

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[logging.StreamHandler()]
)
logger = logging.getLogger(__name__)

app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}})

# --- Configuration ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# NEW: Store data in a subdirectory to avoid messing with old files
CSV_STORAGE_DIR = os.path.join(SCRIPT_DIR, 'data')
os.makedirs(CSV_STORAGE_DIR, exist_ok=True)
logger.info(f"Storage Directory: {CSV_STORAGE_DIR}")

# ML Model Path
ML_MODEL_PATH = os.path.join(SCRIPT_DIR, 'fault_detector_model.pkl')
ml_model = None

if ML_AVAILABLE and os.path.exists(ML_MODEL_PATH):
    try:
        ml_model = joblib.load(ML_MODEL_PATH)
        logger.info(f"✅ ML: Loaded Fault Detection Model from {ML_MODEL_PATH}")
    except Exception as e:
        logger.error(f"❌ ML: Failed to load model: {e}")
else:
    logger.warning("⚠️ ML: Model not found or libraries missing. Running in standard mode.")

# Settings
DATA_RETENTION_DAYS = 30
MAX_CSV_SIZE_MB = 50
ZERO_DATA_INTERVAL_SECONDS = 60
ZERO_DATA_TIMEOUT_SECONDS = 120

# State
last_data_received = {}
server_start_time = datetime.now()
DATA_CACHE = {}
CACHE_TIMEOUT = 5

# Locks
PENDING_COMMANDS = {}
COMMANDS_LOCK = threading.Lock()
FILE_LOCK = threading.Lock() # NEW: Prevents file corruption

# Thresholds
STATUS_THRESHOLDS = {
    "OVERHEAT_PANEL_TEMP": 60,
    "HIGH_POWER_VOLTAGE": 6.0,
    "HIGH_POWER_CURRENT": 10.0,
    "LOW_VOLTAGE": 3.0,
    "HIGH_HUMIDITY": 85.0,
    "LOW_HUMIDITY": 20.0
}

# --- File Management ---
def get_data_file(sender_id):
    return os.path.join(CSV_STORAGE_DIR, f"sensor_data_station_{sender_id}.csv")

def ensure_csv_file(filepath):
    if not os.path.exists(filepath):
        with FILE_LOCK:
            try:
                with open(filepath, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        "timestamp", "senderId", "ldrValue", "dhtTemp", "humidity",
                        "thermistorTemp", "voltage", "current", "valid", "gateway_timestamp_ms", "ml_prediction"
                    ])
                logger.info(f"Created new CSV file: {filepath}")
            except Exception as e:
                logger.error(f"Failed to create CSV: {e}")

def check_file_size(filepath):
    if os.path.exists(filepath):
        size_mb = os.path.getsize(filepath) / (1024 * 1024)
        if size_mb > MAX_CSV_SIZE_MB:
            return True
    return False

def rotate_csv_file(filepath):
    if os.path.exists(filepath):
        with FILE_LOCK:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            rotated_file = f"{filepath}.{timestamp}.bak"
            try:
                os.rename(filepath, rotated_file)
                ensure_csv_file(filepath)
                return True
            except Exception as e:
                logger.error(f"Failed to rotate file: {e}")
    return False

# --- Data Processing ---
def calculate_system_status(df):
    if df.empty:
        return {"status": "NO DATA", "details": "No recent records."}
    
    latest = df.iloc[-1]
    # Check ML prediction if available in CSV
    ml_pred = latest.get("ml_prediction", "Normal")
    
    status_list = []
    alerts = []
    
    if ml_pred and ml_pred != "Normal" and pd.notna(ml_pred):
        status_list.append(f"🤖 ML DETECTED: {ml_pred}")
        alerts.append("ml_fault")

    # Standard Threshold Checks
    dht_temp = float(latest.get("dhtTemp", 0))
    voltage = float(latest.get("voltage", 0))
    current = float(latest.get("current", 0))
    
    if dht_temp > STATUS_THRESHOLDS["OVERHEAT_PANEL_TEMP"]:
        status_list.append(f"🔥 OVERHEAT ({dht_temp}°C)")
        alerts.append("overheat")
    
    if voltage > STATUS_THRESHOLDS["HIGH_POWER_VOLTAGE"] and current > STATUS_THRESHOLDS["HIGH_POWER_CURRENT"]:
        status_list.append("⚠️ HIGH POWER FAULT")
        alerts.append("high_power")
        
    if not status_list:
        return {"status": "OPERATIONAL", "details": "Normal", "alerts": []}
    
    return {
        "status": "WARNING", 
        "details": "\n".join(status_list),
        "alerts": alerts
    }

def calculate_statistics(df):
    stats = {}
    for col in ["dhtTemp", "humidity", "voltage", "current"]:
        if col in df.columns:
            s = pd.to_numeric(df[col], errors='coerce').dropna()
            if not s.empty:
                stats[col] = {
                    "min": float(s.min()), "max": float(s.max()), "avg": float(s.mean())
                }
    return stats

# --- Background Tasks ---
def log_zero_data_if_needed(station_id):
    now = datetime.now()
    should_log = False
    
    if station_id in last_data_received:
        if (now - last_data_received[station_id]).total_seconds() >= ZERO_DATA_TIMEOUT_SECONDS:
            should_log = True
    elif (now - server_start_time).total_seconds() >= ZERO_DATA_TIMEOUT_SECONDS:
        should_log = True
    
    if should_log:
        file_name = get_data_file(station_id)
        # Check if we already logged recently to avoid spam
        if os.path.exists(file_name):
            try:
                with FILE_LOCK:
                    with open(file_name, 'r') as f:
                        pass # Just checking access
            except:
                pass

        ensure_csv_file(file_name)
        row = [now.isoformat(), station_id, 0, 0, 0, 0, 0, 0, False, int(now.timestamp()*1000), "Offline"]
        
        try:
            with FILE_LOCK:
                with open(file_name, 'a', newline='') as f:
                    csv.writer(f).writerow(row)
            logger.debug(f"Logged zero data for Station {station_id}")
        except Exception as e:
            logger.error(f"Zero log error: {e}")

def zero_data_logging_thread():
    while True:
        time.sleep(ZERO_DATA_INTERVAL_SECONDS)
        for sid in [1, 2]:
            log_zero_data_if_needed(sid)

# --- API ---
@app.route('/log_data', methods=['POST'])
def log_data():
    try:
        data = request.get_json(force=True)
    except:
        return jsonify({"status": "error"}), 400

    if not isinstance(data, list):
        data = [data]

    logged = 0
    for record in data:
        sender_id = record.get("senderId")
        if not sender_id: continue
        
        file_name = get_data_file(sender_id)
        ensure_csv_file(file_name)
        if check_file_size(file_name): rotate_csv_file(file_name)
        
        # Extract Data
        ldr_value = float(record.get("ldrValue") or 0)
        dht_temp = float(record.get("dhtTemp") or 0)
        humidity = float(record.get("humidity") or 0)
        voltage = float(record.get("voltage") or 0)
        # In log_data() function:
current_raw = record.get("current")
# Sender already sends 0 for ≤200mA, so just convert properly
try:
    current_val = float(current_raw or 0)
    if current_val == 0:
        current = 0.0  # Keep as 0
    else:
        current = current_val / 1000  # Convert mA to A
except:
    current = 0.0
       # current = float(current_raw or 0) / 1000  # Convert mA to A
    thermistor_temp = float(record.get("thermistorTemp") or 0)
        
        logger.info(f"Station {sender_id}: Raw current={current_raw}, Converted current={current}")
        
        # ML Prediction
        ml_prediction = "Normal"
        if ml_model:
            try:
                # Features must match training: [dhtTemp, humidity, voltage, current, thermistorTemp, ldrValue]
                pred = ml_model.predict([[dht_temp, humidity, voltage, current, thermistor_temp, ldr_value]])[0]
                ml_prediction = str(pred)
                if ml_prediction != "Normal":
                    logger.warning(f"🤖 ML FAULT DETECTED [Station {sender_id}]: {ml_prediction}")
            except Exception as e:
                logger.error(f"ML Error: {e}")

        row = [
            datetime.now().isoformat(),
            sender_id,
            ldr_value,
            dht_temp,
            humidity,
            thermistor_temp,
            voltage,
            current,
            True, # valid
            int(datetime.now().timestamp() * 1000),
            ml_prediction
        ]
        
        with FILE_LOCK:
            with open(file_name, 'a', newline='') as f:
                csv.writer(f).writerow(row)
        
        logged += 1
        last_data_received[sender_id] = datetime.now()
        
        # Clear cache
        if f"chart_{sender_id}" in DATA_CACHE: del DATA_CACHE[f"chart_{sender_id}"]

    return jsonify({"status": "success", "logged": logged})

@app.route('/chart_data')
def chart_data():
    sender_id = request.args.get('senderId')
    hours = float(request.args.get('hours', 1))
    
    file_name = get_data_file(sender_id)
    if not os.path.exists(file_name):
        return jsonify({"error": "No data"})
        
    try:
        # Read CSV (Inefficient but functional for prototype)
        df = pd.read_csv(file_name)
        df['timestamp'] = pd.to_datetime(df['timestamp'])
        df = df[df['timestamp'] >= (datetime.now() - timedelta(hours=hours))]

        # Clean numeric columns to prevent errors
        for col in ["ldrValue", "dhtTemp", "humidity", "thermistorTemp", "voltage", "current"]:
            if col in df.columns:
                df[col] = pd.to_numeric(df[col], errors='coerce').fillna(0)
        
        # Map negative voltage to zero for dashboard display
        df['voltage'] = df['voltage'].clip(lower=0)
        
        # Current is already in amps from storage, no further conversion needed
        
        status = calculate_system_status(df)
        
        return jsonify({
            "labels": df['timestamp'].dt.strftime('%Y-%m-%dT%H:%M:%S').tolist(),
            "temps": df['dhtTemp'].tolist() if 'dhtTemp' in df.columns else [],
            "humidity": df['humidity'].tolist() if 'humidity' in df.columns else [],
            "ldr": df['ldrValue'].tolist() if 'ldrValue' in df.columns else [],
            "thermistor": df['thermistorTemp'].tolist() if 'thermistorTemp' in df.columns else [],
            "voltage": df['voltage'].tolist() if 'voltage' in df.columns else [],
            "current": df['current'].tolist() if 'current' in df.columns else [],
            "ml_predictions": df.get('ml_prediction', []).fillna("Normal").tolist(),
            "status_info": status,
            "statistics": calculate_statistics(df)
        })
    except Exception as e:
        return jsonify({"error": str(e)})

@app.route('/get_stations')
def get_stations():
    """List all available stations."""
    # Ensure default stations exist
    for station_id in [1, 2]:
        ensure_csv_file(get_data_file(station_id))
    
    csv_pattern = os.path.join(CSV_STORAGE_DIR, "sensor_data_station_*.csv")
    files = glob.glob(csv_pattern)
    stations = set([1, 2])
    
    for f in files:
        try:
            station_id_str = os.path.basename(f).split('_')[-1].replace('.csv', '')
            stations.add(int(station_id_str))
        except ValueError:
            continue
            
    return jsonify({
        "stations": sorted(list(stations))
    })

@app.route('/command', methods=['POST'])
def command():
    try:
        data = request.get_json(force=True)
        station_id = str(data.get("station_id"))
        command = data.get("command")

        if not station_id or not command:
            return jsonify({"status": "error", "message": "station_id and command are required"}), 400

        with COMMANDS_LOCK:
            PENDING_COMMANDS[station_id] = command
        
        logger.info(f"Command '{command}' for station '{station_id}' queued.")
        return jsonify({"status": "success", "message": "Command queued"}), 200

    except Exception as e:
        logger.error(f"Error processing command: {e}")
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/get-command/<path:station_id>', methods=['GET'])
def get_command(station_id):
    command = None
    with COMMANDS_LOCK:
        if station_id in PENDING_COMMANDS:
            command = PENDING_COMMANDS.pop(station_id)
            logger.info(f"Command '{command}' sent to station '{station_id}'.")
    
    if command:
        return jsonify({"station_id": station_id, "command": command}), 200
    else:
        return jsonify({}), 204

@app.route('/export_data')
def export_data():
    sender_id = request.args.get('senderId')
    hours = request.args.get('hours', '24')
    format_type = request.args.get('format', 'csv')
    
    if not sender_id:
        return jsonify({"error": "senderId required"}), 400
    
    file_name = get_data_file(sender_id)
    if not os.path.exists(file_name):
        return jsonify({"error": "No data found"}), 404
    
    try:
        df = pd.read_csv(file_name)
        try:
            hours_val = float(hours)
            df['timestamp'] = pd.to_datetime(df['timestamp'], errors='coerce')
            time_range = datetime.now() - timedelta(hours=hours_val)
            df = df[df['timestamp'] >= time_range]
        except:
            pass
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        if format_type.lower() == 'json':
            json_data = df.to_json(orient='records', date_format='iso')
            return Response(
                json_data,
                mimetype="application/json",
                headers={"Content-disposition": f"attachment; filename=station_{sender_id}_{timestamp}.json"}
            )
        else:
            csv_data = df.to_csv(index=False)
            return Response(
                csv_data,
                mimetype="text/csv",
                headers={"Content-disposition": f"attachment; filename=station_{sender_id}_{timestamp}.csv"}
            )
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/dashboard')
def dashboard():
    # Try to find the dashboard HTML in parent directory or current
    possible_paths = [
        os.path.join(SCRIPT_DIR, 'dashboard_enhanced.html'),
        os.path.join(os.path.dirname(SCRIPT_DIR), 'dashboard_enhanced.html')
    ]
    
    for p in possible_paths:
        if os.path.exists(p):
            with open(p, 'r', encoding='utf-8') as f:
                return f.read()
                
    return "Dashboard HTML file not found. Please copy 'dashboard_enhanced.html' to this folder."

@app.route('/')
def index():
    return f"""
    <h1>ML Enhanced Server Running</h1>
    <p>Data Storage: {CSV_STORAGE_DIR}</p>
    <p>ML Model: {'Loaded' if ml_model else 'Not Loaded'}</p>
    <a href="/dashboard">Go to Dashboard</a>
    """

if __name__ == "__main__":
    # Start background threads
    t = threading.Thread(target=zero_data_logging_thread, daemon=True)
    t.start()
    
    print(f"--- ML SERVER STARTED ---")
    print(f"Storage: {CSV_STORAGE_DIR}")
    print(f"Port: 5000 (Ensure old server is stopped)")
    app.run(host='0.0.0.0', port=5000, threaded=True)
