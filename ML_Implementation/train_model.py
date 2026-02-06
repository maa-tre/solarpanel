# FILE: d:\DEEPSEEEK\ML_Implementation\train_model.py
import pandas as pd
import glob
import os
import joblib
import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report

# --- Configuration ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, 'fault_detector_model.pkl')

# Logic to find ORIGINAL data (Assuming server_enhanced.py was in d:\DEEPSEEEK and stored data in d:\)
# We look 2 levels up from d:\DEEPSEEEK\ML_Implementation -> d:\
OLD_DATA_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..'))

print(f"Looking for training data in: {OLD_DATA_DIR}")

# Thresholds (Must match server logic for consistent labeling)
THRESHOLDS = {
    "OVERHEAT": 60,
    "HIGH_VOLTAGE": 6.0,
    "HIGH_CURRENT": 10.0,
    "LOW_VOLTAGE": 3.0,
    "HIGH_HUMIDITY": 85.0
}

def generate_labels(row):
    """Auto-label historical data to create a training set."""
    # 0. Consistency Check (The power of ML: Detecting relationships)
    if abs(row['dhtTemp'] - row['thermistorTemp']) > 15:
        return "Sensor Mismatch"

    # 0.5. Sensor Error Logic (High Current but Low Voltage/Light)
    # If current is high but voltage is too low to support it, it's likely a sensor glitch.
    if row['current'] > THRESHOLDS["HIGH_CURRENT"] and row['voltage'] < THRESHOLDS["LOW_VOLTAGE"]:
        return "Sensor Error (Invalid Current)"

    # 0.6. Panel Efficiency Fault (Context Aware)
    # Detects if voltage is low DESPITE high light levels (e.g., panel covered/damaged)
    if row['ldrValue'] > 700 and row['voltage'] < 2.5:
        return "Panel Efficiency Fault"

    # 1. Critical Faults
    if row['dhtTemp'] > THRESHOLDS["OVERHEAT"] or row['thermistorTemp'] > THRESHOLDS["OVERHEAT"]:
        return "Overheating"
    
    if row['voltage'] > THRESHOLDS["HIGH_VOLTAGE"] and row['current'] > THRESHOLDS["HIGH_CURRENT"]:
        return "Power Surge"
    
    if row['voltage'] > THRESHOLDS["HIGH_VOLTAGE"]:
        return "High Voltage Warning"

    if row['current'] > THRESHOLDS["HIGH_CURRENT"]:
        return "High Current Warning"
    
    if row['voltage'] < 0:
        return "Sensor Error (Neg Voltage)"
        
    # 2. Warnings
    if row['voltage'] < THRESHOLDS["LOW_VOLTAGE"] and row['voltage'] > 0.1:
        return "Low Battery"
    
    if row['humidity'] > THRESHOLDS["HIGH_HUMIDITY"]:
        return "High Humidity Risk"
        
    # 3. Normal
    return "Normal"

def generate_synthetic_data(n_samples=500):
    """Generate synthetic data to ensure model learns all fault types."""
    print(f"Generating {n_samples} synthetic records per fault type...")
    data = []
    
    # 1. Normal Operation (Safe ranges)
    for _ in range(n_samples):
        data.append({
            'dhtTemp': np.random.uniform(20, 55),
            'humidity': np.random.uniform(20, 80),
            'voltage': np.random.uniform(3.5, 5.5),
            'current': np.random.uniform(0.1, 9.0),
            'thermistorTemp': np.random.uniform(20, 55),
            'ldrValue': np.random.uniform(200, 1000)
        })

    # 2. Overheating (Temp > 60)
    for _ in range(n_samples):
        data.append({
            'dhtTemp': np.random.uniform(61, 90),
            'humidity': np.random.uniform(20, 60),
            'voltage': np.random.uniform(3.3, 5.0),
            'current': np.random.uniform(1.0, 5.0),
            'thermistorTemp': np.random.uniform(61, 90),
            'ldrValue': np.random.uniform(200, 1000)
        })

    # 3. Power Surge (Volt > 6 AND Curr > 10)
    for _ in range(n_samples):
        data.append({
            'dhtTemp': np.random.uniform(30, 50),
            'humidity': np.random.uniform(30, 60),
            'voltage': np.random.uniform(6.1, 12.0),
            'current': np.random.uniform(10.1, 20.0),
            'thermistorTemp': np.random.uniform(30, 50),
            'ldrValue': np.random.uniform(500, 1000)
        })

    # 4. Sensor Error (Volt < 0)
    for _ in range(n_samples):
        data.append({
            'dhtTemp': 0,
            'humidity': 0,
            'voltage': np.random.uniform(-5.0, -0.1),
            'current': 0,
            'thermistorTemp': 0,
            'ldrValue': 0
        })

    # 5. Low Battery (0.1 < Volt < 3.0)
    for _ in range(n_samples):
        data.append({
            'dhtTemp': np.random.uniform(20, 40),
            'humidity': np.random.uniform(30, 60),
            'voltage': np.random.uniform(0.2, 2.9),
            'current': np.random.uniform(0.1, 2.0),
            'thermistorTemp': np.random.uniform(20, 40),
            'ldrValue': np.random.uniform(0, 300)
        })

    # 6. High Humidity (Hum > 85)
    for _ in range(n_samples):
        data.append({
            'dhtTemp': np.random.uniform(20, 35),
            'humidity': np.random.uniform(86, 100),
            'voltage': np.random.uniform(3.3, 5.0),
            'current': np.random.uniform(0.1, 2.0),
            'thermistorTemp': np.random.uniform(20, 35),
            'ldrValue': np.random.uniform(200, 800)
        })

    # 7. High Voltage Warning (V > 6, I < 10)
    for _ in range(n_samples):
        data.append({
            'dhtTemp': np.random.uniform(20, 55),
            'humidity': np.random.uniform(20, 80),
            'voltage': np.random.uniform(6.1, 12.0),
            'current': np.random.uniform(0.1, 9.0),
            'thermistorTemp': np.random.uniform(20, 55),
            'ldrValue': np.random.uniform(500, 1000)
        })

    # 8. High Current Warning (V < 6, I > 10)
    for _ in range(n_samples):
        data.append({
            'dhtTemp': np.random.uniform(20, 55),
            'humidity': np.random.uniform(20, 80),
            'voltage': np.random.uniform(3.5, 5.5),
            'current': np.random.uniform(10.1, 20.0),
            'thermistorTemp': np.random.uniform(20, 55),
            'ldrValue': np.random.uniform(500, 1000)
        })

    # 9. Sensor Mismatch (Both sensors safe, but disagree > 15C)
    for _ in range(n_samples):
        base_temp = np.random.uniform(20, 45) # Safe base temp
        data.append({
            'dhtTemp': base_temp,
            'humidity': np.random.uniform(30, 60),
            'voltage': np.random.uniform(3.5, 5.0),
            'current': np.random.uniform(0.5, 2.0),
            'thermistorTemp': base_temp + np.random.uniform(16, 30), # Diverge by 16-30 degrees
            'ldrValue': np.random.uniform(200, 800)
        })

    # 10. Sensor Error (Invalid Current: High Current + Low Voltage/Light)
    for _ in range(n_samples):
        data.append({
            'dhtTemp': np.random.uniform(20, 40),
            'humidity': np.random.uniform(30, 60),
            'voltage': np.random.uniform(0.1, 2.9), # Low Voltage
            'current': np.random.uniform(10.1, 20.0), # High Current (Impossible with low V)
            'thermistorTemp': np.random.uniform(20, 40),
            'ldrValue': np.random.uniform(0, 200) # Low Light
        })

    # 11. Panel Efficiency Fault (High Light + Low Voltage)
    # This teaches the model that Low Voltage during the DAY is bad.
    for _ in range(n_samples):
        data.append({
            'dhtTemp': np.random.uniform(20, 45),
            'humidity': np.random.uniform(30, 60),
            'voltage': np.random.uniform(0.1, 2.4), # Low Voltage
            'current': np.random.uniform(0.0, 0.5),
            'thermistorTemp': np.random.uniform(20, 45),
            'ldrValue': np.random.uniform(750, 1000) # High Light (Daytime)
        })

    return pd.DataFrame(data)

def train_model():
    print("--- Starting ML Model Training ---")
    
    # 1. Load Data
    csv_pattern = os.path.join(OLD_DATA_DIR, "sensor_data_station_*.csv")
    files = glob.glob(csv_pattern)
    
    if not files:
        print(f"❌ No CSV data files found in {OLD_DATA_DIR} to train on!")
        print("⚠️ Proceeding with SYNTHETIC data only.")
    
    df_list = []
    for f in files:
        try:
            df = pd.read_csv(f)
            # Ensure columns exist
            required_cols = ['dhtTemp', 'humidity', 'voltage', 'current', 'thermistorTemp', 'ldrValue']
            if all(col in df.columns for col in required_cols):
                df_list.append(df)
                print(f"✅ Loaded {len(df)} records from {os.path.basename(f)}")
            else:
                print(f"⚠️ Skipping {os.path.basename(f)} (missing columns)")
        except Exception as e:
            print(f"❌ Error reading {f}: {e}")
    
    # Combine real data (if any)
    full_df = pd.concat(df_list, ignore_index=True) if df_list else pd.DataFrame()

    # Add Synthetic Data to ensure model learns all faults
    synthetic_df = generate_synthetic_data()
    full_df = pd.concat([full_df, synthetic_df], ignore_index=True)
    
    # 2. Preprocessing
    feature_cols = ['dhtTemp', 'humidity', 'voltage', 'current', 'thermistorTemp', 'ldrValue']
    
    # Clean data (convert to numeric, drop NaNs)
    for col in feature_cols:
        full_df[col] = pd.to_numeric(full_df[col], errors='coerce')
    
    full_df = full_df.dropna(subset=feature_cols)
    
    if full_df.empty:
        print("No valid data after cleaning.")
        return

    # 3. Generate Labels
    print("Generating synthetic labels...")
    full_df['fault_label'] = full_df.apply(generate_labels, axis=1)
    print(f"Class Distribution:\n{full_df['fault_label'].value_counts()}")

    # 4. Train
    X = full_df[feature_cols]
    y = full_df['fault_label']
    
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
    
    print(f"\nTraining Random Forest on {len(X_train)} records...")
    clf = RandomForestClassifier(n_estimators=100, random_state=42)
    clf.fit(X_train, y_train)
    
    # 5. Save
    joblib.dump(clf, MODEL_PATH)
    print(f"\n✅ Model saved to: {MODEL_PATH}")
    print("You can now run 'server_ml.py' in this folder.")

if __name__ == "__main__":
    train_model()
