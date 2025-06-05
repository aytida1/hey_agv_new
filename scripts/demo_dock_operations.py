#!/usr/bin/env python3
"""
Quick Demo Script for Pharmacy Dock System
Shows how to use the system programmatically
"""

import requests
import time
import json

def demo_dock_operations():
    """Demonstrate the dock system operations"""
    
    server_url = "http://localhost:5000"
    
    print("🎯 Pharmacy Dock System Demo")
    print("=" * 50)
    
    try:
        # 1. Check initial status
        print("1. Checking initial dock status...")
        response = requests.get(f"{server_url}/status")
        status = response.json()
        print(f"   Current dock: {status['current_dock']}")
        print(f"   Is docked: {status['is_docked']}")
        print(f"   Status: {status['last_operation_status']}")
        
        # 2. Demonstrate docking sequence
        if not status['operation_in_progress']:
            print("\n2. Testing dock sequence...")
            
            # Dock to dock 1
            print("   → Docking to Pharmacy Dock 1...")
            response = requests.post(f"{server_url}/dock/1")
            if response.status_code == 200:
                result = response.json()
                print(f"   ✅ {result['message']}")
                
                # Wait and check status
                time.sleep(3)
                status_response = requests.get(f"{server_url}/status")
                new_status = status_response.json()
                print(f"   Status: {new_status['last_operation_status']}")
                
                # If successful and not busy, try switching docks
                if not new_status['operation_in_progress']:
                    print("\n   → Switching to Pharmacy Dock 2...")
                    switch_response = requests.post(f"{server_url}/dock/2")
                    if switch_response.status_code == 200:
                        switch_result = switch_response.json()
                        print(f"   ✅ {switch_result['message']}")
                        print("   (This will automatically undock from dock 1 first)")
            else:
                error = response.json()
                print(f"   ⚠️ {error.get('error', 'Unknown error')}")
        else:
            print("\n2. Skipping demo (operation in progress)")
            
        print("\n✅ Demo completed!")
        
    except requests.exceptions.ConnectionError:
        print("❌ Cannot connect to dock server.")
        print("   Please start the server first:")
        print("   cd /home/meditab/car_ws_new")
        print("   ./src/hey_agv_new/scripts/start_dock_system.sh")
        
    except Exception as e:
        print(f"❌ Demo failed: {e}")

if __name__ == "__main__":
    demo_dock_operations()
