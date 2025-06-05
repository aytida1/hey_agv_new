#!/bin/bash

# Pharmacy Dock System Startup Script
echo "🚀 Starting Pharmacy Dock Control System..."

# Check if we're in a ROS2 workspace
if [ ! -f "install/setup.bash" ]; then
    echo "❌ Error: Please run this script from the ROS2 workspace root"
    echo "   Expected path: /home/meditab/car_ws_new"
    exit 1
fi

# Source ROS2 workspace
echo "📦 Sourcing ROS2 workspace..."
source install/setup.bash

# Check if required dependencies are installed
echo "🔍 Checking dependencies..."

# Check Python dependencies
python3 -c "import flask, flask_cors" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ Missing Python dependencies. Installing..."
    cd src/hey_agv_new
    pip3 install -r requirements.txt
    cd ../..
fi

# Check Node.js dependencies
if [ ! -f "src/hey_agv_new/web_interface/node_modules/express/package.json" ]; then
    echo "📦 Installing Node.js dependencies..."
    cd src/hey_agv_new/web_interface
    npm install express http-proxy-middleware
    cd ../../..
fi

echo "✅ Dependencies checked"

# Start the system
echo "🌟 Launching Pharmacy Dock System..."
echo "   - Pharmacy Dock Server will start on port 5000"
echo "   - Web Interface will start on port 3000"
echo "   - Open http://localhost:3000 in your browser"
echo ""

# Launch using ROS2 launch file
ros2 launch hey_agv_new web_interface.launch.py
