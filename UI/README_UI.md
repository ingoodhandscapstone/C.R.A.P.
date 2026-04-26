# MQTT Session UI

## Install
```bash
python3 -m pip install -r requirements.txt
```

## Run
```bash
python3 mqtt_session_ui.py
```

The app connects to MQTT at `localhost:1883`, publishes commands to `system/command`, waits for calibration complete on `system/calibration_status`, plots selected session topics live, and saves CSV files to `session_exports/` on stop.
