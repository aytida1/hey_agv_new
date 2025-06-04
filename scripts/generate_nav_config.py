#!/usr/bin/env python3
"""
Dynamic parameter configuration generator for multi-AGV setups.
This script generates configuration files for any number of AGVs.
"""

import yaml
import os
import argparse
from typing import List, Dict, Any


def create_base_amcl_config() -> Dict[str, Any]:
    """Create base AMCL configuration that can be used for any AGV."""
    return {
        "ros__parameters": {
            # Motion model parameters
            "alpha1": 0.05,
            "alpha2": 0.05,
            "alpha3": 0.05,
            "alpha4": 0.05,
            "alpha5": 0.02,
            
            # Frame configuration (will be filled per AGV)
            "base_frame_id": "",
            "global_frame_id": "map",
            "odom_frame_id": "",
            
            # Beam skipping for robustness
            "beam_skip_distance": 0.5,
            "beam_skip_error_threshold": 0.5,
            "beam_skip_threshold": 0.3,
            "do_beamskip": True,
            
            # Laser model parameters
            "lambda_short": 0.1,
            "laser_likelihood_max_dist": 4.0,
            "laser_max_range": 100.0,
            "laser_min_range": -1.0,
            "laser_model_type": "likelihood_field",
            
            # Particle filter parameters
            "max_beams": 120,
            "max_particles": 2000,
            "min_particles": 500,
            "pf_err": 0.01,
            "pf_z": 0.99,
            
            # Recovery parameters
            "recovery_alpha_fast": 0.1,
            "recovery_alpha_slow": 0.001,
            
            # Update parameters
            "resample_interval": 1,
            "robot_model_type": "nav2_amcl::DifferentialMotionModel",
            "save_pose_rate": 2.0,
            "sigma_hit": 0.1,
            "tf_broadcast": True,
            "transform_tolerance": 2.0,
            "message_timeout_margin": 1.0,
            "tf_buffer_duration": 30.0,
            "update_min_a": 0.1,
            "update_min_d": 0.1,
            
            # Sensor model weights
            "z_hit": 0.8,
            "z_max": 0.02,
            "z_rand": 0.1,
            "z_short": 0.08,
            
            "scan_topic": "scan",
            "selective_resampling": True,
            "first_map_only": False,
            "kld_err": 0.01,
            "kld_z": 0.99,
            "set_initial_pose": False,
            "max_weight_ratio": 1000.0
        }
    }


def generate_multi_agv_config(agv_namespaces: List[str], output_file: str) -> None:
    """Generate a complete multi-AGV configuration file."""
    
    # Create the complete configuration
    complete_config = {}
    
    # Generate namespace-specific configurations for each AGV
    for namespace in agv_namespaces:
        # AMCL configuration for this AGV
        amcl_config = create_base_amcl_config()
        amcl_config["ros__parameters"]["base_frame_id"] = f"{namespace}/base_link"
        amcl_config["ros__parameters"]["odom_frame_id"] = f"{namespace}/odom"
        complete_config[f"{namespace}/amcl"] = amcl_config
        
        # BT Navigator configuration for this AGV
        bt_nav_config = {
            "ros__parameters": {
                "global_frame": "map",
                "robot_base_frame": f"{namespace}/base_link",
                "odom_topic": f"{namespace}/odom",
                "bt_loop_duration": 10,
                "default_server_timeout": 20,
                "wait_for_service_timeout": 1000,
                "action_server_result_timeout": 900.0,
                "navigators": ["navigate_to_pose", "navigate_through_poses"],
                "navigate_to_pose": {
                    "plugin": "nav2_bt_navigator::NavigateToPoseNavigator"
                },
                "navigate_through_poses": {
                    "plugin": "nav2_bt_navigator::NavigateThroughPosesNavigator"
                },
                "error_code_names": [
                    "compute_path_error_code",
                    "follow_path_error_code"
                ]
            }
        }
        complete_config[f"{namespace}/bt_navigator"] = bt_nav_config
        
        # Controller Server configuration for this AGV
        controller_config = {
            "ros__parameters": {
                "controller_frequency": 30.0,
                "costmap_update_timeout": 0.30,
                "min_x_velocity_threshold": 0.001,
                "min_y_velocity_threshold": 0.5,
                "min_theta_velocity_threshold": 0.001,
                "failure_tolerance": 0.3,
                "progress_checker_plugins": ["progress_checker"],
                "goal_checker_plugins": ["general_goal_checker"],
                "controller_plugins": ["FollowPath"],
                "use_realtime_priority": False,
                
                "progress_checker": {
                    "plugin": "nav2_controller::SimpleProgressChecker",
                    "required_movement_radius": 0.5,
                    "movement_time_allowance": 10.0
                },
                
                "general_goal_checker": {
                    "stateful": True,
                    "plugin": "nav2_controller::SimpleGoalChecker",
                    "xy_goal_tolerance": 0.05,
                    "yaw_goal_tolerance": 0.01
                },
                
                "FollowPath": {
                    "plugin": "nav2_rotation_shim_controller::RotationShimController",
                    "primary_controller": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
                    "angular_dist_threshold": 0.785,
                    "forward_sampling_distance": 0.5,
                    "angular_disengage_threshold": 0.3925,
                    "rotate_to_heading_angular_vel": 1.8,
                    "max_angular_accel": 3.2,
                    "simulate_ahead_time": 1.0,
                    "rotate_to_goal_heading": True,
                    "desired_linear_vel": 1.0,
                    "lookahead_dist": 0.6,
                    "min_lookahead_dist": 0.3,
                    "max_lookahead_dist": 0.9,
                    "lookahead_time": 1.5,
                    "transform_tolerance": 0.1,
                    "use_velocity_scaled_lookahead_dist": False,
                    "min_approach_linear_velocity": 0.05,
                    "approach_velocity_scaling_dist": 0.6,
                    "use_collision_detection": True,
                    "max_allowed_time_to_collision_up_to_carrot": 1.0,
                    "use_regulated_linear_velocity_scaling": True,
                    "use_fixed_curvature_lookahead": False,
                    "curvature_lookahead_dist": 0.25,
                    "use_cost_regulated_linear_velocity_scaling": False,
                    "regulated_linear_scaling_min_radius": 0.9,
                    "regulated_linear_scaling_min_speed": 0.25,
                    "use_rotate_to_heading": True,
                    "allow_reversing": False,
                    "rotate_to_heading_min_angle": 0.785,
                    "max_robot_pose_search_dist": 10.0,
                    "stateful": True
                }
            }
        }
        complete_config[f"{namespace}/controller_server"] = controller_config
        
        # Local Costmap configuration for this AGV
        local_costmap_config = {
            "local_costmap": {
                "ros__parameters": {
                    "update_frequency": 5.0,
                    "publish_frequency": 2.0,
                    "global_frame": f"{namespace}/odom",
                    "robot_base_frame": f"{namespace}/base_link",
                    "rolling_window": True,
                    "width": 3,
                    "height": 3,
                    "resolution": 0.05,
                    "robot_radius": 0.26,
                    "plugins": ["voxel_layer", "inflation_layer"],
                    "inflation_layer": {
                        "plugin": "nav2_costmap_2d::InflationLayer",
                        "cost_scaling_factor": 3.0,
                        "inflation_radius": 0.70
                    },
                    "voxel_layer": {
                        "plugin": "nav2_costmap_2d::VoxelLayer",
                        "enabled": True,
                        "publish_voxel_map": True,
                        "origin_z": 0.0,
                        "z_resolution": 0.05,
                        "z_voxels": 16,
                        "max_obstacle_height": 2.0,
                        "mark_threshold": 0,
                        "observation_sources": "scan",
                        "scan": {
                            "topic": f"{namespace}/scan",
                            "max_obstacle_height": 2.0,
                            "clearing": True,
                            "marking": True,
                            "data_type": "LaserScan",
                            "raytrace_max_range": 3.0,
                            "raytrace_min_range": 0.0,
                            "obstacle_max_range": 2.5,
                            "obstacle_min_range": 0.0
                        }
                    },
                    "always_send_full_costmap": True
                }
            }
        }
        complete_config[f"{namespace}/local_costmap"] = local_costmap_config
        
        # Global Costmap configuration for this AGV
        global_costmap_config = {
            "global_costmap": {
                "ros__parameters": {
                    "update_frequency": 1.0,
                    "publish_frequency": 1.0,
                    "global_frame": "map",
                    "robot_base_frame": f"{namespace}/base_link",
                    "robot_radius": 0.26,
                    "resolution": 0.05,
                    "track_unknown_space": True,
                    "plugins": ["static_layer", "obstacle_layer", "inflation_layer"],
                    "obstacle_layer": {
                        "plugin": "nav2_costmap_2d::ObstacleLayer",
                        "enabled": True,
                        "observation_sources": "scan",
                        "scan": {
                            "topic": f"{namespace}/scan",
                            "max_obstacle_height": 2.0,
                            "clearing": True,
                            "marking": True,
                            "data_type": "LaserScan",
                            "raytrace_max_range": 3.0,
                            "raytrace_min_range": 0.0,
                            "obstacle_max_range": 2.5,
                            "obstacle_min_range": 0.0
                        }
                    },
                    "static_layer": {
                        "plugin": "nav2_costmap_2d::StaticLayer",
                        "map_subscribe_transient_local": True
                    },
                    "inflation_layer": {
                        "plugin": "nav2_costmap_2d::InflationLayer",
                        "cost_scaling_factor": 3.0,
                        "inflation_radius": 0.7
                    },
                    "always_send_full_costmap": True
                }
            }
        }
        complete_config[f"{namespace}/global_costmap"] = global_costmap_config
        
        # Planner Server configuration for this AGV
        planner_config = {
            "ros__parameters": {
                "expected_planner_frequency": 20.0,
                "planner_plugins": ["GridBased"],
                "costmap_update_timeout": 1.0,
                "GridBased": {
                    "plugin": "nav2_navfn_planner::NavfnPlanner",
                    "tolerance": 0.5,
                    "use_astar": False,
                    "allow_unknown": True
                }
            }
        }
        complete_config[f"{namespace}/planner_server"] = planner_config
        
        # Behavior Server configuration for this AGV
        behavior_config = {
            "ros__parameters": {
                "local_costmap_topic": f"{namespace}/local_costmap/costmap_raw",
                "global_costmap_topic": f"{namespace}/global_costmap/costmap_raw",
                "local_footprint_topic": f"{namespace}/local_costmap/published_footprint",
                "global_footprint_topic": f"{namespace}/global_costmap/published_footprint",
                "cycle_frequency": 10.0,
                "behavior_plugins": ["spin", "backup", "drive_on_heading", "assisted_teleop", "wait"],
                "spin": {"plugin": "nav2_behaviors::Spin"},
                "backup": {"plugin": "nav2_behaviors::BackUp"},
                "drive_on_heading": {"plugin": "nav2_behaviors::DriveOnHeading"},
                "wait": {"plugin": "nav2_behaviors::Wait"},
                "assisted_teleop": {"plugin": "nav2_behaviors::AssistedTeleop"},
                "local_frame": f"{namespace}/odom",
                "global_frame": "map",
                "robot_base_frame": f"{namespace}/base_link",
                "transform_tolerance": 0.1,
                "simulate_ahead_time": 2.0,
                "max_rotational_vel": 1.0,
                "min_rotational_vel": 0.4,
                "rotational_acc_lim": 3.2
            }
        }
        complete_config[f"{namespace}/behavior_server"] = behavior_config
        
        # Velocity Smoother configuration for this AGV
        velocity_smoother_config = {
            "ros__parameters": {
                "smoothing_frequency": 20.0,
                "scale_velocities": False,
                "feedback": "OPEN_LOOP",
                "max_velocity": [4.0, 0.0, 2.0],
                "min_velocity": [-4.0, 0.0, -2.0],
                "max_accel": [2.5, 0.0, 3.2],
                "max_decel": [-2.5, 0.0, -3.2],
                "odom_topic": f"{namespace}/odom",
                "odom_duration": 0.1,
                "deadband_velocity": [0.0, 0.0, 0.0],
                "velocity_timeout": 1.0
            }
        }
        complete_config[f"{namespace}/velocity_smoother"] = velocity_smoother_config
    
    # Add shared Waypoint Follower configuration (can be shared across AGVs)
    complete_config["waypoint_follower"] = {
        "ros__parameters": {
            "loop_rate": 20,
            "stop_on_failure": False,
            "action_server_result_timeout": 900.0,
            "waypoint_task_executor_plugin": "wait_at_waypoint",
            "wait_at_waypoint": {
                "plugin": "nav2_waypoint_follower::WaitAtWaypoint",
                "enabled": True,
                "waypoint_pause_duration": 200
            }
        }
    }
    
    # Write to file
    with open(output_file, 'w') as f:
        yaml.dump(complete_config, f, default_flow_style=False, sort_keys=False)
    
    print(f"Generated configuration for {len(agv_namespaces)} AGVs: {', '.join(agv_namespaces)}")
    print(f"Configuration saved to: {output_file}")


def main():
    parser = argparse.ArgumentParser(description='Generate multi-AGV navigation configuration')
    parser.add_argument('--agvs', nargs='+', default=['agv1', 'agv2', 'agv3', 'agv4', 'agv5'],
                       help='List of AGV namespaces (default: agv1 agv2 agv3 agv4 agv5)')
    parser.add_argument('--output', default='nav2_params_generated.yaml',
                       help='Output configuration file (default: nav2_params_generated.yaml)')
    
    args = parser.parse_args()
    
    generate_multi_agv_config(args.agvs, args.output)


if __name__ == '__main__':
    main()
