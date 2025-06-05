# Pharmacy Dock Control System

This system provides a web-based interface for controlling AGV docking operations to pharmacy docks. It replaces the GUI-based system with a web server that can handle docking/undocking operations with automatic undocking when switching between docks.

## System Architecture

```
Web Interface (localhost:3000) 
    ↓ HTTP API calls
Pharmacy Dock Server (localhost:5000)
    ↓ ROS2 Actions
AGV Docking System (DockRobot/UndockRobot actions)
```

## Features

- **Web-based Control**: Modern web interface for dock operations
- **Automatic Undocking**: When docking to a new dock, automatically undocks from current dock first
- **Real-time Status**: Live updates of dock status and operation progress
- **4 Pharmacy Docks**: Support for pharmacist_dock1 through pharmacist_dock4
- **Error Handling**: Comprehensive error handling and user feedback
- **ROS2 Integration**: Uses nav2 DockRobot and UndockRobot actions

## Installation

### Prerequisites

1. ROS2 Jazzy
2. Nav2 with docking server
3. Python 3 with Flask and Flask-CORS

### Install Dependencies

#### Python Dependencies (Flask for Web Server)

**Important**: For ROS2 environments, use system package manager instead of pip to avoid conflicts:

```bash
# Install Flask and Flask-CORS using system package manager (recommended for ROS2)
sudo apt update
sudo apt install python3-flask python3-flask-cors

# Verify installation
python3 -c "import flask, flask_cors; print('Flask and Flask-CORS are ready to use')"
```

**Alternative**: If you encounter the "externally-managed-environment" error when using pip:
- This is normal behavior in modern Linux distributions to protect the system Python
- The system package installation above is the preferred method for ROS2 environments
- Do NOT use virtual environments as they may interfere with ROS2 Python paths

### Build the Package

```bash
# Build the ROS2 package
cd /home/meditab/car_ws_new
colcon build --packages-select hey_agv_new
source install/setup.bash
```

## Usage

### Method 1: Using Launch File (Recommended)

```bash
# Start the complete web interface system (includes dock server)
ros2 launch hey_agv_new web_interface.launch.py
```

This will start:
- ROSBridge server (port 9091)
- Pharmacy Dock Server (port 5000) 
- Web interface server with API proxying (port 3000)

### Method 2: Manual Start

#### 1. Start the Pharmacy Dock Server

```bash
# Terminal 1: Start the dock server
cd /home/meditab/car_ws_new
source install/setup.bash
ros2 run hey_agv_new pharmacy_dock_server.py
```

#### 2. Start the Web Interface Server

```bash
# Terminal 2: Start the web interface (No Node.js required)
cd /home/meditab/car_ws_new
source install/setup.bash
ros2 run hey_agv_new web_server.py
```

#### 3. Access the Web Interface

Open your browser and navigate to: http://localhost:3000

## Web Interface Usage

### 1. Select AGV
- Use the "AGV Selection" dropdown to select which AGV to control
- Only proceed once an AGV is selected and connected

### 2. Pharmacy Dock Control Section

#### Dock Status Display
- **Current Dock**: Shows which dock the AGV is currently at (if any)
- **Docked**: Shows whether the AGV is currently docked
- **Operation**: Shows the current operation status

#### Dock Operations
- **Pharmacy Dock 1-4 Buttons**: Click to dock to a specific pharmacy dock
- **Undock Button**: Click to undock from the current dock

### 3. Automatic Behavior

The system handles complex scenarios automatically:

- **If AGV is not docked**: Clicking a dock button will dock to that dock
- **If AGV is docked at dock X and you click dock Y**: 
  1. System automatically undocks from dock X
  2. Waits for undocking to complete
  3. Then docks to dock Y
- **Operation in Progress**: All buttons are disabled during operations

## API Endpoints

The Pharmacy Dock Server provides these HTTP API endpoints:

### GET /status
Returns current dock status:
```json
{
    "current_dock": 1,
    "is_docked": true,
    "operation_in_progress": false,
    "last_operation_status": "Successfully docked to dock 1",
    "last_operation_result": "success"
}
```

### POST /dock/<dock_number>
Initiate docking to pharmacy dock (1-4):
```bash
curl -X POST http://localhost:5000/dock/1
```

### POST /undock
Initiate undocking from current dock:
```bash
curl -X POST http://localhost:5000/undock
```

## Configuration

### Dock IDs and Types
The system uses these dock configurations:
- **Dock IDs**: `pharmacist_dock1`, `pharmacist_dock2`, `pharmacist_dock3`, `pharmacist_dock4`
- **Dock Type**: `pharmacist_docks`
- **AGV Namespace**: `agv1` (configurable in the server code)

### Server Ports
- **Web Interface**: Port 3000
- **Dock Server**: Port 5000
- **ROSBridge**: Port 9091 (if using WebSocket features)

## Troubleshooting

### Common Issues

1. **"Action server not available" Error**
   ```bash
   # Check if docking server is running
   ros2 node list | grep dock
   
   # Check available actions
   ros2 action list | grep dock
   ```

2. **Web Interface Cannot Connect**
   ```bash
   # Test dock server directly
   curl http://localhost:5000/status
   
   # Check if web server is running
   curl http://localhost:3000
   ```

3. **"No such file or directory: 'node'" Error (FIXED)**
   - This error occurred in older versions that required Node.js
   - **Solution**: The system now works entirely with Python - no Node.js needed!
   - If you still see this error, rebuild the package:
   ```bash
   cd /home/meditab/car_ws_new
   colcon build --packages-select hey_agv_new
   source install/setup.bash
   ```

4. **404 Errors on /api/dock/status**
   - This indicates the API proxying isn't working
   - **Solution**: Ensure you're using the updated web_server.py that includes API proxying
   - Check that the pharmacy dock server is running on port 5000

5. **Dock Operation Fails**
   ```bash
   # Check dock server logs
   ros2 run hey_agv_new pharmacy_dock_server.py
   
   # Verify dock configurations exist
   ros2 param list | grep dock
   ```

### Testing the System

Run the test script to verify everything is working:

```bash
cd /home/meditab/car_ws_new/src/hey_agv_new
python3 scripts/test_dock_system.py
```

## Development

### Key Files
- `src/pharmacy_dock_server.py`: Main dock server with ROS2 action clients
- `web_interface/index.html`: Web interface with dock controls
- `web_interface/script.js`: JavaScript for dock functionality
- `web_interface/server.js`: Express server with API proxy
- `launch/web_interface.launch.py`: Complete system launch file (includes dock server)

### Extending the System

To add more docks or modify behavior:

1. **Add More Docks**: Update the dock buttons in `index.html` and add corresponding dock IDs in `pharmacy_dock_server.py`

2. **Change AGV Namespace**: Modify the action client topics in `pharmacy_dock_server.py`:
   ```python
   self._dock_action_client = ActionClient(self, DockRobot, 'agv2/dock_robot')
   ```

3. **Add More AGV Support**: Extend the server to handle multiple AGVs by parameterizing the namespace

## License

Apache-2.0 License (same as the main package)
