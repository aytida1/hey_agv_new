#!/usr/bin/env python3
"""
Test script for PSR dock server functionality
"""
import requests
import time
import json

PSR_DOCK_SERVER_URL = "http://localhost:5002"
WEB_API_URL = "http://localhost:3000/api/psr-dock"

def test_psr_dock_server():
    """Test the PSR dock server directly"""
    print("🧪 Testing PSR Dock Server...")
    
    try:
        # Test status endpoint
        print("\n1. Testing PSR dock status endpoint...")
        response = requests.get(f"{PSR_DOCK_SERVER_URL}/status", timeout=5)
        if response.status_code == 200:
            status = response.json()
            print(f"   ✅ PSR Status: {json.dumps(status, indent=2)}")
            return True
        else:
            print(f"   ❌ PSR Status endpoint failed: {response.status_code}")
            return False
            
    except requests.exceptions.ConnectionError:
        print("❌ Cannot connect to PSR dock server. Make sure it's running on localhost:5002")
        return False
    except Exception as e:
        print(f"❌ PSR dock server test failed: {e}")
        return False

def test_web_psr_api():
    """Test the web API proxy for PSR dock"""
    print("\n🌐 Testing Web PSR API Proxy...")
    
    try:
        # Test web server PSR API proxy
        response = requests.get(f"{WEB_API_URL}/status", timeout=5)
        if response.status_code == 200:
            status = response.json()
            print("   ✅ PSR API proxy is working")
            print(f"   📊 PSR Status via proxy: {json.dumps(status, indent=2)}")
            return True
        else:
            print(f"   ❌ PSR API proxy failed: {response.status_code}")
            return False
            
    except requests.exceptions.ConnectionError:
        print("❌ Cannot connect to web interface. Make sure it's running on localhost:3000")
        return False
    except Exception as e:
        print(f"❌ Web PSR API test failed: {e}")
        return False

def test_psr_dock_operation():
    """Test PSR dock operation"""
    print("\n🚢 Testing PSR Dock Operation...")
    
    try:
        # Test dock AGV1 to PSR dock
        print("   Sending PSR dock request for agv1...")
        dock_response = requests.post(f"{PSR_DOCK_SERVER_URL}/dock/agv1", timeout=5)
        if dock_response.status_code == 200:
            result = dock_response.json()
            print(f"   ✅ PSR Dock request: {result['message']}")
            
            # Wait a moment and check status
            time.sleep(2)
            status_response = requests.get(f"{PSR_DOCK_SERVER_URL}/status/agv1", timeout=5)
            if status_response.status_code == 200:
                status = status_response.json()
                print(f"   📊 AGV1 PSR Status: {status['last_operation_status']}")
            
            return True
        else:
            print(f"   ❌ PSR Dock request failed: {dock_response.status_code}")
            return False
            
    except Exception as e:
        print(f"❌ PSR dock operation test failed: {e}")
        return False

if __name__ == "__main__":
    print("🚀 PSR Dock System Test")
    print("=" * 50)
    
    psr_server_ok = test_psr_dock_server()
    web_api_ok = test_web_psr_api()
    
    if psr_server_ok:
        test_psr_dock_operation()
    
    print("\n" + "=" * 50)
    if psr_server_ok and web_api_ok:
        print("🎉 PSR dock system tests passed! System is ready.")
    else:
        print("⚠️  Some PSR dock tests failed. Check the output above.")
        
    print("\n📋 PSR Dock Usage Instructions:")
    print("1. Start the complete system: ros2 launch hey_agv_new web_interface.launch.py")
    print("2. Open http://localhost:3000 in your browser")
    print("3. Select an AGV and use the PSR Dock Control section")
    print("4. PSR dock server runs on port 5002, web interface on port 3000")
