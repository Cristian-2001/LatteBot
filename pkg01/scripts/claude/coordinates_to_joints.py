#!/usr/bin/env python3
"""
Script to convert Cartesian coordinates to joint positions using MoveIt IK solver.
Finds joint angles needed to reach a specific (x, y, z) position with desired orientation.
"""

import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion
import sys
from math import pi

def cartesian_to_joints(x, y, z, roll=0, pitch=pi/2, yaw=0):
    """
    Convert Cartesian coordinates to joint positions using IK.
    
    Args:
        x, y, z: Target position in meters (relative to world_platform)
        roll, pitch, yaw: Gripper orientation in radians
                         Default: pitch=90° (gripper pointing down)
    
    Returns:
        List of joint positions or None if IK fails
    """
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cartesian_to_joints', anonymous=True)
    
    # Initialize MoveIt
    robot = moveit_commander.RobotCommander()
    arm_group = moveit_commander.MoveGroupCommander("manipulator")
    
    print("\n" + "="*60)
    print("INVERSE KINEMATICS SOLVER")
    print("="*60)
    print(f"Target Position: x={x:.3f}, y={y:.3f}, z={z:.3f}")
    print(f"Target Orientation (RPY): roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}")
    print("-"*60)
    
    # Create target pose
    target_pose = Pose()
    target_pose.position = Point(x=x, y=y, z=z)
    
    # Convert RPY to quaternion (simplified - for accurate conversion use tf.transformations)
    from math import sin, cos
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    
    target_pose.orientation.w = cr * cp * cy + sr * sp * sy
    target_pose.orientation.x = sr * cp * cy - cr * sp * sy
    target_pose.orientation.y = cr * sp * cy + sr * cp * sy
    target_pose.orientation.z = cr * cp * sy - sr * sp * cy
    
    # Set pose target
    arm_group.set_pose_target(target_pose)
    
    # Attempt to compute IK solution
    print("Computing inverse kinematics...")
    plan = arm_group.plan()
    
    # Extract joint positions from plan
    if plan and hasattr(plan, 'joint_trajectory') and len(plan.joint_trajectory.points) > 0:
        # Get the final joint configuration from trajectory
        final_joints = plan.joint_trajectory.points[-1].positions
        joint_names = arm_group.get_active_joints()
        
        print("\n✓ IK Solution Found!")
        print("-"*60)
        print("Joint Positions:")
        print("-"*60)
        
        joint_dict = {}
        for name, value in zip(joint_names, final_joints):
            joint_dict[name] = value
            print(f"  {name:25s}: {value:8.4f} rad ({value*180/pi:7.2f}°)")
        
        print("\n" + "="*60)
        print("SRDF FORMAT (copy to ur10e.srdf):")
        print("="*60)
        print('<group_state name="your_pose_name" group="manipulator">')
        for name, value in zip(joint_names, final_joints):
            print(f'    <joint name="{name}" value="{value:.6f}"/>')
        print('</group_state>')
        
        print("\n" + "="*60)
        print("PYTHON FORMAT (use in scripts):")
        print("="*60)
        print("joint_positions = [")
        for value in final_joints:
            print(f"    {value:.6f},")
        print("]")
        
        # Ask if user wants to visualize/execute
        print("\n" + "="*60)
        response = input("Execute motion to visualize? (y/n): ")
        if response.lower() == 'y':
            print("Executing motion...")
            arm_group.execute(plan, wait=True)
            print("Motion complete!")
        
        moveit_commander.roscpp_shutdown()
        return list(final_joints)
        
    else:
        print("\n✗ IK Solution NOT Found!")
        print("-"*60)
        print("Possible reasons:")
        print("  - Target position is out of reach")
        print("  - Target orientation is not achievable")
        print("  - Position causes collision")
        print("  - Try adjusting position or orientation")
        
        # Show reachability info
        current_pose = arm_group.get_current_pose().pose
        print(f"\nCurrent end-effector position:")
        print(f"  x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")
        
        moveit_commander.roscpp_shutdown()
        return None


def find_bucket_grasp_poses():
    """
    Find joint positions for bucket grasping sequence.
    Bucket is at (2, 2, 2) in world coordinates.
    """
    print("\n" + "#"*60)
    print("# BUCKET GRASP SEQUENCE - IK SOLUTIONS")
    print("#"*60)
    
    # Bucket location
    bucket_x, bucket_y, bucket_z = 2.0, 2.0, 2.0
    
    # Sequence of poses
    poses = [
        {
            "name": "approach_bucket",
            "x": bucket_x,
            "y": bucket_y,
            "z": bucket_z + 0.3,  # 30cm above bucket
            "pitch": pi/2,  # Gripper pointing down
            "description": "Approach from above"
        },
        {
            "name": "pre_grasp",
            "x": bucket_x,
            "y": bucket_y,
            "z": bucket_z + 0.15,  # 15cm above bucket
            "pitch": pi/2,
            "description": "Just above handle"
        },
        {
            "name": "grasp",
            "x": bucket_x,
            "y": bucket_y,
            "z": bucket_z,  # At bucket handle height
            "pitch": pi/2,
            "description": "At handle level"
        },
        {
            "name": "lift",
            "x": bucket_x,
            "y": bucket_y,
            "z": bucket_z + 0.4,  # Lift 40cm
            "pitch": pi/2,
            "description": "Lifted bucket"
        }
    ]
    
    results = {}
    
    for pose_info in poses:
        print("\n" + "="*60)
        print(f"POSE: {pose_info['name'].upper()}")
        print(f"Description: {pose_info['description']}")
        print("="*60)
        
        joints = cartesian_to_joints(
            x=pose_info['x'],
            y=pose_info['y'],
            z=pose_info['z'],
            pitch=pose_info.get('pitch', pi/2),
            roll=pose_info.get('roll', 0),
            yaw=pose_info.get('yaw', 0)
        )
        
        if joints:
            results[pose_info['name']] = joints
        
        input("\nPress Enter to continue to next pose...")
    
    # Summary
    print("\n" + "#"*60)
    print("# SUMMARY - ALL POSES")
    print("#"*60)
    print(f"\nSuccessfully computed {len(results)} out of {len(poses)} poses")
    
    if results:
        print("\nAdd these to your ur10e.srdf file:")
        print("-"*60)
        for pose_name, joints in results.items():
            print(f'\n<group_state name="{pose_name}" group="manipulator">')
            joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
            for name, value in zip(joint_names, joints):
                print(f'    <joint name="{name}" value="{value:.6f}"/>')
            print('</group_state>')


if __name__ == '__main__':
    try:
        print("\nMODE SELECTION:")
        print("1. Find specific pose (enter coordinates)")
        print("2. Generate bucket grasp sequence")
        
        mode = input("\nSelect mode (1 or 2): ")
        
        if mode == "1":
            # Manual coordinate entry
            print("\nEnter target coordinates:")
            x = float(input("  x (meters): "))
            y = float(input("  y (meters): "))
            z = float(input("  z (meters): "))
            
            print("\nOrientation (press Enter for defaults):")
            roll_input = input(f"  roll (default=0): ")
            pitch_input = input(f"  pitch (default={pi/2:.3f} = gripper down): ")
            yaw_input = input(f"  yaw (default=0): ")
            
            roll = float(roll_input) if roll_input else 0
            pitch = float(pitch_input) if pitch_input else pi/2
            yaw = float(yaw_input) if yaw_input else 0
            
            cartesian_to_joints(x, y, z, roll, pitch, yaw)
            
        elif mode == "2":
            # Bucket grasp sequence
            find_bucket_grasp_poses()
        else:
            print("Invalid selection!")
            
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nAborted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()