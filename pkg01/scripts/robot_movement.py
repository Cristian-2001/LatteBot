#!/usr/bin/env python3
import sys

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import moveit_commander

from move_platform import move_platform


# define poses
HOME = "home"
INTERMEDIATE_GRAB = "intermediate_grab"
GRAB = "grab"
INTERMEDIATE_POSE = "intermediate_pose"
LEAVE = "leave"
OPEN = "open"
CLOSE = "close"

class RobotMovementPipeline:
    def __init__(self):
        rospy.init_node('robot_listener', anonymous=True)

        # Initialize MoveIt commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        
        # Store current joint states
        self.current_joint_states = None
        self.home_position = 0.0
        
        # Subscribe to joint states
        self.joint_state_sub = rospy.Subscriber(
            '/ur10e_robot/joint_states', 
            JointState, 
            self._joint_state_callback
        )
        
        # Subscribe to cow numbers
        self.cow_sub = rospy.Subscriber(
            'cow_num', 
            String, 
            self.process
        )

        # check if the platform is in home position, otherwise move it there
        if self.get_platform_position() != 0:
            self._go_home()
        
        rospy.loginfo("Robot listener node started, waiting for cow numbers...")
        
    def get_platform_position(self):
        """Get current platform position."""
        return self.platform_position
    
    def _get_cow_position(self, cow_num: str):
        """Map cow number to platform position and move."""
        try:
            num = int(cow_num)
            if 0 <= num <= 10:
                position = float(num)  # TODO: Map to actual cow positions
                return position
            else:
                rospy.logwarn("Cow number out of range (0-10). Returning home")
                return -1
        except ValueError:
            rospy.logwarn("Invalid cow number received. Returning home")
            return -1
        
    def _go_home(self):
        move_platform(self.home_position)

    def _joint_state_callback(self, msg):
        """Store the latest joint states."""
        self.current_joint_states = msg
        
        # Extract platform_joint position
        try:
            idx = msg.name.index('platform_joint')
            self.platform_position = msg.position[idx]
        except ValueError:
            rospy.logwarn_once("platform_joint not found in joint_states")

    def _move_arm_to_pose(self, pose: str) -> bool:
        """Move the UR10e arm to the position named "pose" using MoveIt."""
        try:
            rospy.loginfo(f"Moving arm to {pose} position...")
            
            # Use the 'pose' named target defined in SRDF
            self.move_group.set_named_target(pose)
            
            # Plan and execute
            plan = self.move_group.go(wait=True)
            
            # Stop any residual movement
            self.move_group.stop()
            
            # Clear targets
            self.move_group.clear_pose_targets()
            
            if plan:
                rospy.loginfo(f"Arm successfully moved to {pose} position")
                return True
            else:
                rospy.logwarn(f"Failed to plan path to {pose} position")
                return False
                
        except Exception as e:
            rospy.logerr(f"Error moving arm to {pose}: {e}")
            return False
        
    def _move_gripper(self, pose: str) -> bool:
        """Opens or close the gripper"""
        try:
            rospy.loginfo(f"Moving arm to {pose} position...")
            
            # Use the 'pose' named target defined in SRDF
            self.gripper_group.set_named_target(pose)
            
            # Plan and execute
            plan = self.gripper_group.go(wait=True)
            
            # Stop any residual movement
            self.gripper_group.stop()
            
            # Clear targets
            self.gripper_group.clear_pose_targets()
            
            if plan:
                rospy.loginfo(f"Gripper successfully moved to {pose} position")
                return True
            else:
                rospy.logwarn(f"Failed to plan path to {pose} position")
                return False
                
        except Exception as e:
            rospy.logerr(f"Error moving gripper to {pose}: {e}")
            return False

    def process(self, cow_num_msg):
        """Implements the order of operations the robot has to do"""
        # get the cow_number
        cow_num = str(cow_num_msg.data)

        cow_position = self._get_cow_position(cow_num)
        if cow_position == -1:
            # invalid position, returns -1
            print(f"Invalid cow position: {cow_position}")
            return

        # check if the platform is in home position, otherwise move it there
        if self.get_platform_position() != 0:
            self._go_home

        # Execute movement sequence
        sequence = [
            ("arm", INTERMEDIATE_GRAB),
            ("gripper", OPEN),
            ("arm", GRAB),
            ("gripper", CLOSE),
            ("arm", INTERMEDIATE_POSE),
            ("platform", cow_position),
            ("arm", INTERMEDIATE_POSE),
            ("arm", LEAVE),
            ("gripper", OPEN),
            ("arm", INTERMEDIATE_POSE),
            ("arm", HOME)
        ]
        
        for action_type, target in sequence:
            if action_type == "arm":
                if not self._move_arm_to_pose(target):
                    rospy.logerr(f"Failed to move arm to {target}")
                    return
                elif action_type == "gripper":
                    if not self._move_gripper(target):
                        rospy.logerr(f"Failed to move gripper to {target}")
                        return
                    elif action_type == "platform":
                        if not move_platform(target):
                            rospy.logerr(f"Failed to move platform to {target}")
                            return
    
    def run(self):
        """Keep the node running."""
        rospy.spin()

if __name__ == '__main__':
    try:
        pipeline = RobotMovementPipeline()
        pipeline.run()
    except rospy.ROSInterruptException:
        pass