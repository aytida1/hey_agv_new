#!/usr/bin/env python3

import yaml
import os
from ament_index_python.packages import get_package_share_directory

def test_nav_params():
    """Test if navigation parameters are properly configured for multi-AGV setup"""
    
    pkg_share = get_package_share_directory('hey_agv_new')
    nav_config_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    print(f"Testing navigation parameters from: {nav_config_path}")
    
    try:
        with open(nav_config_path, 'r') as file:
            config = yaml.safe_load(file)
        
        # Check critical topics for namespace compatibility
        print("\n=== Navigation Parameter Validation ===")
        
        # Check AMCL scan topic
        amcl_scan = config.get('amcl', {}).get('ros__parameters', {}).get('scan_topic', '')
        print(f"✓ AMCL scan topic: '{amcl_scan}' (should be 'scan' for namespace compatibility)")
        
        # Check BT Navigator odom topic
        bt_odom = config.get('bt_navigator', {}).get('ros__parameters', {}).get('odom_topic', '')
        print(f"✓ BT Navigator odom topic: '{bt_odom}' (should be 'odom' for namespace compatibility)")
        
        # Check local costmap scan topic
        local_scan = config.get('local_costmap', {}).get('local_costmap', {}).get('ros__parameters', {}).get('voxel_layer', {}).get('scan', {}).get('topic', '')
        print(f"✓ Local costmap scan topic: '{local_scan}' (should be 'scan' for namespace compatibility)")
        
        # Check global costmap scan topic
        global_scan = config.get('global_costmap', {}).get('global_costmap', {}).get('ros__parameters', {}).get('obstacle_layer', {}).get('scan', {}).get('topic', '')
        print(f"✓ Global costmap scan topic: '{global_scan}' (should be 'scan' for namespace compatibility)")
        
        # Check controller frequency
        controller_freq = config.get('controller_server', {}).get('ros__parameters', {}).get('controller_frequency', 0)
        print(f"✓ Controller frequency: {controller_freq} Hz")
        
        # Check planner frequency
        planner_freq = config.get('planner_server', {}).get('ros__parameters', {}).get('expected_planner_frequency', 0)
        print(f"✓ Planner frequency: {planner_freq} Hz")
        
        print("\n=== Configuration Status ===")
        
        # Validate critical settings
        issues = []
        
        if amcl_scan not in ['scan', '']:
            issues.append(f"AMCL scan_topic should be 'scan', found '{amcl_scan}'")
            
        if bt_odom not in ['odom', '']:
            issues.append(f"BT Navigator odom_topic should be 'odom', found '{bt_odom}'")
            
        if local_scan not in ['scan', '']:
            issues.append(f"Local costmap scan topic should be 'scan', found '{local_scan}'")
            
        if global_scan not in ['scan', '']:
            issues.append(f"Global costmap scan topic should be 'scan', found '{global_scan}'")
        
        if issues:
            print("❌ Issues found:")
            for issue in issues:
                print(f"   - {issue}")
            return False
        else:
            print("✅ All navigation parameters are properly configured for multi-AGV setup!")
            return True
            
    except Exception as e:
        print(f"❌ Error reading configuration: {e}")
        return False

if __name__ == '__main__':
    test_nav_params()
