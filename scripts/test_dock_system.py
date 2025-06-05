#!/usr/bin/env python3
"""
Test script for pharmacy dock server functionality
"""
import requests
import time
import json

DOCK_SERVER_URL = "http://localhost:5000"

def test_dock_server():
    print("🧪 Testing Pharmacy Dock Server...")
    
    try:
        # Test status endpoint
        print("\n1. Testing status endpoint...")
        response = requests.get(f"{DOCK_SERVER_URL}/status", timeout=5)
        if response.status_code == 200:
            status = response.json()
            print(f"   ✅ Status: {json.dumps(status, indent=2)}")
        else:
            print(f"   ❌ Status endpoint failed: {response.status_code}")
            return False
        
        # Test dock endpoint (if not already docked)
        if not status.get('operation_in_progress', False):
            print("\n2. Testing dock endpoint...")
            dock_response = requests.post(f"{DOCK_SERVER_URL}/dock/1", timeout=5)
            if dock_response.status_code == 200:
                result = dock_response.json()
                print(f"   ✅ Dock request: {result['message']}")
            else:
                error_data = dock_response.json() if dock_response.headers.get('content-type', '').startswith('application/json') else {'error': 'Unknown error'}
                print(f"   ⚠️  Dock request: {error_data.get('error', 'Unknown error')}")
        else:
            print("\n2. Skipping dock test (operation in progress)")
        
        print("\n✅ Dock server is responding correctly!")
        return True
        
    except requests.exceptions.ConnectionError:
        print("❌ Cannot connect to dock server. Make sure it's running on localhost:5000")
        return False
    except Exception as e:
        print(f"❌ Test failed: {e}")
        return False

def test_web_interface():
    print("\n🌐 Testing Web Interface Server...")
    
    try:
        # Test web server
        response = requests.get("http://localhost:3000", timeout=5)
        if response.status_code == 200:
            print("   ✅ Web interface is accessible")
            
            # Test dock API proxy
            api_response = requests.get("http://localhost:3000/api/dock/status", timeout=5)
            if api_response.status_code == 200:
                print("   ✅ Dock API proxy is working")
                return True
            else:
                print(f"   ❌ Dock API proxy failed: {api_response.status_code}")
                return False
        else:
            print(f"   ❌ Web interface failed: {response.status_code}")
            return False
            
    except requests.exceptions.ConnectionError:
        print("❌ Cannot connect to web interface. Make sure it's running on localhost:3000")
        return False
    except Exception as e:
        print(f"❌ Web interface test failed: {e}")
        return False

if __name__ == "__main__":
    print("🚀 Pharmacy Dock System Test")
    print("=" * 50)
    
    dock_ok = test_dock_server()
    web_ok = test_web_interface()
    
    print("\n" + "=" * 50)
    if dock_ok and web_ok:
        print("🎉 All tests passed! System is ready.")
    else:
        print("⚠️  Some tests failed. Check the output above.")
        
    print("\n📋 Usage Instructions:")
    print("1. Start the pharmacy dock server: python3 src/pharmacy_dock_server.py")
    print("2. Start the web interface: node web_interface/server.js")
    print("3. Open http://localhost:3000 in your browser")
    print("4. Select an AGV and use the Pharmacy Dock Control section")
