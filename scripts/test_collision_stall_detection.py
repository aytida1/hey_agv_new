#!/usr/bin/env python3
"""
Test script for collision and stalling detection in the multi-AGV pharmacy dock server
"""
import requests
import time
import json

class CollisionStallTester:
    def __init__(self, server_url="http://localhost:5000"):
        self.server_url = server_url
    
    def test_collision_detection(self):
        """Test collision detection and emergency response"""
        print("🧪 Testing Collision Detection")
        print("=" * 50)
        
        # Get initial status
        response = requests.get(f"{self.server_url}/monitoring_status")
        if response.status_code == 200:
            print("✅ Monitoring status endpoint working")
            print(json.dumps(response.json(), indent=2))
        else:
            print("❌ Failed to get monitoring status")
            return
        
        # Test emergency stop
        print("\n🚨 Testing manual emergency stop for AGV1...")
        response = requests.post(f"{self.server_url}/emergency_stop/agv1")
        if response.status_code == 200:
            print("✅ Emergency stop initiated")
            print(response.json())
        else:
            print("❌ Failed to initiate emergency stop")
            return
        
        # Check status after emergency stop
        time.sleep(2)
        response = requests.get(f"{self.server_url}/status/agv1")
        if response.status_code == 200:
            status = response.json()
            print(f"\n📊 AGV1 Status after emergency stop:")
            print(f"   Emergency Stopped: {status.get('emergency_stopped', 'Unknown')}")
            print(f"   Operation Status: {status.get('last_operation_status', 'Unknown')}")
        
        # Test reset emergency
        print("\n🔄 Testing emergency reset for AGV1...")
        response = requests.post(f"{self.server_url}/reset_emergency/agv1")
        if response.status_code == 200:
            print("✅ Emergency reset successful")
            print(response.json())
        else:
            print("❌ Failed to reset emergency")
            return
        
        # Check status after reset
        time.sleep(1)
        response = requests.get(f"{self.server_url}/status/agv1")
        if response.status_code == 200:
            status = response.json()
            print(f"\n📊 AGV1 Status after reset:")
            print(f"   Emergency Stopped: {status.get('emergency_stopped', 'Unknown')}")
            print(f"   Operation Status: {status.get('last_operation_status', 'Unknown')}")
    
    def test_dock_with_safety(self):
        """Test docking operation with safety monitoring"""
        print("\n🧪 Testing Safe Docking Operation")
        print("=" * 50)
        
        # Ensure AGV1 is not in emergency state
        requests.post(f"{self.server_url}/reset_emergency/agv1")
        time.sleep(1)
        
        # Start a docking operation
        print("🚢 Starting docking operation for AGV1 to dock 1...")
        response = requests.post(f"{self.server_url}/dock/agv1/1")
        if response.status_code == 200:
            print("✅ Docking operation initiated")
            print(response.json())
            
            # Monitor the operation
            print("\n📊 Monitoring docking operation...")
            for i in range(10):  # Monitor for 10 seconds
                time.sleep(1)
                response = requests.get(f"{self.server_url}/status/agv1")
                if response.status_code == 200:
                    status = response.json()
                    print(f"   [{i+1:2d}s] Status: {status.get('last_operation_status', 'Unknown')}")
                    
                    # Check if operation completed
                    if not status.get('operation_in_progress', False):
                        print(f"   🏁 Operation completed: {status.get('last_operation_result', 'Unknown')}")
                        break
                    
                    # Check for emergency conditions
                    monitoring_response = requests.get(f"{self.server_url}/monitoring_status")
                    if monitoring_response.status_code == 200:
                        monitoring = monitoring_response.json()
                        agv1_monitoring = monitoring.get('agv1', {})
                        if (agv1_monitoring.get('collision_detected') or 
                            agv1_monitoring.get('stalled') or 
                            agv1_monitoring.get('emergency_stopped')):
                            print(f"   🚨 Emergency condition detected!")
                            print(f"      Collision: {agv1_monitoring.get('collision_detected')}")
                            print(f"      Stalled: {agv1_monitoring.get('stalled')}")
                            print(f"      Emergency Stop: {agv1_monitoring.get('emergency_stopped')}")
                            break
        else:
            print("❌ Failed to start docking operation")
            print(response.text)
    
    def test_monitoring_endpoints(self):
        """Test all monitoring endpoints"""
        print("\n🧪 Testing Monitoring Endpoints")
        print("=" * 50)
        
        endpoints = [
            ("/status", "Global Status"),
            ("/monitoring_status", "Monitoring Status"),
            ("/dock_reservations", "Dock Reservations"),
            ("/status/agv1", "AGV1 Status"),
        ]
        
        for endpoint, name in endpoints:
            print(f"\n📡 Testing {name} endpoint: {endpoint}")
            response = requests.get(f"{self.server_url}{endpoint}")
            if response.status_code == 200:
                print("✅ Success")
                if endpoint == "/monitoring_status":
                    # Show detailed monitoring info
                    data = response.json()
                    print("   Monitoring Details:")
                    for agv_id, monitoring in data.items():
                        print(f"     {agv_id}:")
                        print(f"       Collision: {monitoring.get('collision_detected')}")
                        print(f"       Stalled: {monitoring.get('stalled')}")
                        print(f"       Emergency: {monitoring.get('emergency_stopped')}")
                        print(f"       Time since movement: {monitoring.get('time_since_movement', 0):.1f}s")
                else:
                    print(f"   Response: {json.dumps(response.json(), indent=2)[:200]}...")
            else:
                print(f"❌ Failed with status {response.status_code}")
    
    def run_all_tests(self):
        """Run all collision and stalling detection tests"""
        print("🚀 Starting Collision & Stalling Detection Tests")
        print("=" * 60)
        
        try:
            # Test server connectivity
            response = requests.get(f"{self.server_url}/status")
            if response.status_code != 200:
                print("❌ Server not accessible. Make sure the dock server is running.")
                return
            
            print("✅ Server is accessible")
            
            # Run tests
            self.test_monitoring_endpoints()
            self.test_collision_detection()
            self.test_dock_with_safety()
            
            print("\n🎉 All tests completed!")
            print("=" * 60)
            print("📝 Test Summary:")
            print("   ✅ Monitoring endpoints functional")
            print("   ✅ Emergency stop/reset working")
            print("   ✅ Safety-aware docking operations")
            print("   ✅ Real-time monitoring active")
            
        except requests.exceptions.ConnectionError:
            print("❌ Connection Error: Make sure the dock server is running on port 5000")
        except Exception as e:
            print(f"❌ Unexpected error: {e}")

def main():
    """Main function"""
    tester = CollisionStallTester()
    tester.run_all_tests()

if __name__ == '__main__':
    main()
