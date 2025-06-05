# Flask Installation Guide for ROS2 Environments

## Problem: "externally-managed-environment" Error

When trying to install Flask using pip in modern Linux distributions (Ubuntu 24.04+), you may encounter this error:

```
× This environment is externally managed
╰─> To install Python packages system-wide, try apt install
    python3-xyz, where xyz is the package you are trying to
    install.
```

## Solution: Use System Package Manager

**For ROS2 environments, ALWAYS use the system package manager instead of pip or virtual environments.**

### Step 1: Install Flask via APT

```bash
# Update package list
sudo apt update

# Install Flask and Flask-CORS
sudo apt install python3-flask python3-flask-cors
```

### Step 2: Verify Installation

```bash
# Test that Flask is available
python3 -c "import flask, flask_cors; print('Flask and Flask-CORS are ready to use')"
```

### Step 3: Run Your ROS2 Node

```bash
# Navigate to your ROS2 workspace
cd /home/meditab/car_ws_new/src/hey_agv_new

# Run the pharmacy dock server
python3 src/pharmacy_dock_server.py
```

## Why This Approach?

1. **ROS2 Compatibility**: System packages integrate properly with ROS2's Python environment
2. **No Conflicts**: Avoids conflicts between virtual environments and ROS2 Python paths
3. **System Stability**: Doesn't break system Python installation
4. **Dependency Management**: APT handles dependencies automatically

## What NOT to Do

❌ **Don't use pip with --break-system-packages**
```bash
pip install flask --break-system-packages  # DON'T DO THIS
```

❌ **Don't use virtual environments for ROS2 nodes**
```bash
python3 -m venv venv  # DON'T DO THIS for ROS2
```

❌ **Don't install system-wide with pip**
```bash
sudo pip install flask  # DON'T DO THIS
```

## Available Flask Packages via APT

You can search for available Flask-related packages:

```bash
apt search python3-flask
```

Common packages include:
- `python3-flask` - Core Flask framework
- `python3-flask-cors` - CORS support
- `python3-flask-sqlalchemy` - Database ORM
- `python3-flask-login` - User session management
- `python3-flask-mail` - Email support

## Troubleshooting

### Import Errors
If you still get import errors, check your Python path:
```bash
python3 -c "import sys; print('\n'.join(sys.path))"
```

### ROS2 Environment
Make sure your ROS2 environment is properly sourced:
```bash
source /opt/ros/jazzy/setup.bash
source ~/car_ws_new/install/setup.bash
```

### Package Not Found
If a Flask package isn't available via APT, check the package name:
```bash
apt search flask | grep python3
```

## Summary

For ROS2 development:
1. Use `sudo apt install python3-flask python3-flask-cors`
2. Verify with `python3 -c "import flask, flask_cors; print('Success')"`
3. Run your ROS2 nodes normally
4. Keep your ROS2 environment clean and compatible

This approach ensures your Flask web server works seamlessly with your ROS2 navigation and docking system.
