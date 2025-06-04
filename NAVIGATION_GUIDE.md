# Multi-AGV Navigation Launch Files

## Overview
We've enhanced your launch files to include full Nav2 navigation capabilities alongside AMCL localization.

## Launch Files Created/Modified:

### 1. multi_nav2.launch.py (Enhanced Multi-AGV Launch)
- **Original**: multi_amcl.launch.py (AMCL only)  
- **Enhanced**: Includes full navigation stack for each AGV
- **Components per AGV**:
  - AMCL (localization)
  - Controller Server (path following)
  - Planner Server (path planning)
  - Behavior Server (recovery behaviors)
  - BT Navigator (behavior tree navigation)
  - Waypoint Follower (multi-point navigation)
  - Velocity Smoother (smooth motion)

### 2. single_agv_nav.launch.py (Single AGV Testing)
- For testing navigation with one AGV
- Same navigation stack as multi-AGV but simplified
- Use with: `ros2 launch hey_agv_new single_agv_nav.launch.py namespace:=agv1`

## Navigation Capabilities Added:

### Smart Navigation Features:
- **Path Planning**: Global path planning using A* or other algorithms
- **Obstacle Avoidance**: Dynamic obstacle avoidance using local costmaps
- **Recovery Behaviors**: Automatic recovery when stuck (rotate, backup, etc.)
- **Smooth Motion**: Velocity smoothing for realistic robot movement
- **Multi-Point Navigation**: Navigate through multiple waypoints
- **Behavior Trees**: Complex navigation logic using behavior trees

### Topic Structure (per AGV):
```
/{namespace}/cmd_vel          - Velocity commands to robot
/{namespace}/odom             - Odometry from robot
/{namespace}/scan             - Laser scan data
/{namespace}/initialpose      - Set initial pose
/{namespace}/amcl_pose        - Current localized pose
/{namespace}/goal_pose        - Navigation goal
/{namespace}/navigate_to_pose - Navigation action server
```

## How to Use:

### Launch Multi-AGV Navigation:
```bash
# Launch full navigation for AGV1 and AGV2
ros2 launch hey_agv_new multi_nav2.launch.py

# With custom parameters
ros2 launch hey_agv_new multi_nav2.launch.py \
    map:=/path/to/your/map.yaml \
    use_sim_time:=true
```

### Launch Single AGV for Testing:
```bash
# Launch AGV1 only
ros2 launch hey_agv_new single_agv_nav.launch.py namespace:=agv1

# Launch AGV2 only  
ros2 launch hey_agv_new single_agv_nav.launch.py namespace:=agv2
```

### Send Navigation Goals:
```bash
# Send goal to AGV1
ros2 action send_goal /agv1/navigate_to_pose nav2_msgs/action/NavigateToPose \
    "pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"

# Send goal to AGV2
ros2 action send_goal /agv2/navigate_to_pose nav2_msgs/action/NavigateToPose \
    "pose: {header: {frame_id: 'map'}, pose: {position: {x: -2.0, y: -1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

## Configuration:
- Navigation parameters: `config/nav2_params.yaml`
- Map file: `urdf/sacramento.yaml`
- Launch timing: AGV1 starts after 5s, AGV2 after 8s

## Next Steps:
1. Test with: `ros2 launch hey_agv_new multi_nav2.launch.py`
2. Use RViz to send navigation goals
3. Monitor topics: `ros2 topic list | grep -E "(agv1|agv2)"`
4. Check node status: `ros2 lifecycle get /agv1/controller_server`

## Troubleshooting:
- Check if all nav2 packages are installed: `ros2 pkg list | grep nav2`
- Verify costmap updates: `ros2 topic echo /agv1/local_costmap/costmap`
- Monitor navigation status: `ros2 action list | grep navigate`
