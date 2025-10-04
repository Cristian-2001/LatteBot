#!/usr/bin/env python

"""
Verification script to check if gripper controller is properly configured
with both left and right finger joints.
"""

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def verify_gripper_setup():
    rospy.init_node('verify_gripper_setup')
    
    print("\n" + "="*60)
    print("GRIPPER CONTROLLER VERIFICATION")
    print("="*60 + "\n")
    
    # Step 1: Check if controller manager is available
    print("Step 1: Checking controller manager...")
    try:
        rospy.wait_for_service('/ur10e_robot/controller_manager/list_controllers', timeout=5.0)
        print("✓ Controller manager is available")
    except rospy.ROSException:
        print("✗ Controller manager not available - Is Gazebo running?")
        return False
    
    # Step 2: Check controller list
    print("\nStep 2: Checking loaded controllers...")
    try:
        from controller_manager_msgs.srv import ListControllers
        list_controllers = rospy.ServiceProxy('/ur10e_robot/controller_manager/list_controllers', ListControllers)
        response = list_controllers()
        
        gripper_found = False
        for controller in response.controller:
            if 'gripper' in controller.name:
                print(f"✓ Found controller: {controller.name}")
                print(f"  State: {controller.state}")
                print(f"  Type: {controller.type}")
                gripper_found = True
        
        if not gripper_found:
            print("✗ No gripper controller found!")
            return False
            
    except Exception as e:
        print(f"✗ Error checking controllers: {e}")
        return False
    
    # Step 3: Check joint states
    print("\nStep 3: Checking joint states...")
    try:
        msg = rospy.wait_for_message('/ur10e_robot/joint_states', rospy.AnyMsg, timeout=5.0)
        from sensor_msgs.msg import JointState
        joint_state = rospy.wait_for_message('/ur10e_robot/joint_states', JointState, timeout=5.0)
        
        left_found = 'left_finger_joint' in joint_state.name
        right_found = 'right_finger_joint' in joint_state.name
        
        if left_found:
            print("✓ left_finger_joint found in joint states")
        else:
            print("✗ left_finger_joint NOT found in joint states")
            
        if right_found:
            print("✓ right_finger_joint found in joint states")
        else:
            print("✗ right_finger_joint NOT found in joint states")
            
        if not (left_found and right_found):
            return False
            
    except Exception as e:
        print(f"✗ Error checking joint states: {e}")
        return False
    
    # Step 4: Check action server
    print("\nStep 4: Checking gripper action server...")
    client = actionlib.SimpleActionClient(
        '/ur10e_robot/gripper_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction
    )
    
    if client.wait_for_server(timeout=rospy.Duration(5.0)):
        print("✓ Gripper action server is available")
    else:
        print("✗ Gripper action server not available")
        return False
    
    # Step 5: Test basic command
    print("\nStep 5: Testing basic gripper command...")
    try:
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['left_finger_joint', 'right_finger_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [0.01, 0.01]  # Small movement
        point.time_from_start = rospy.Duration(1.0)
        goal.trajectory.points.append(point)
        
        client.send_goal(goal)
        success = client.wait_for_result(timeout=rospy.Duration(3.0))
        
        if success:
            result = client.get_result()
            print("✓ Gripper command executed successfully")
        else:
            print("✗ Gripper command timed out")
            return False
            
    except Exception as e:
        print(f"✗ Error testing gripper command: {e}")
        return False
    
    # Final summary
    print("\n" + "="*60)
    print("VERIFICATION COMPLETE - ALL TESTS PASSED! ✓")
    print("="*60)
    print("\nBoth finger joints are properly configured and working.")
    print("You can now use MoveIt to control the gripper.\n")
    
    return True

if __name__ == '__main__':
    try:
        verify_gripper_setup()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}\n")
