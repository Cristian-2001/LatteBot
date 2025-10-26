#!/usr/bin/env python3
import sys
import time

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import moveit_commander

from move_platform import move_platform


# define poses
HOME = "home"
INTERMEDIATE_GRASP = "intermediate_grasp"
GRASP = "grasp"
INTERMEDIATE_PLACE = "intermediate_place"
PLACE = "place"
OPEN = "open"
CLOSE = "close"
GRASP_HANDLE = "grasp_handle"  # Intermediate gripper position for bucket handle

positions = {
    0: 1,
    1: 3,
    2: 5,
    3: 7,
    4: 9,
    5: 11,
    6: 13,
    7: 15,
    8: 17,
    9: 19
}

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
        self.platform_position = 0.0
        
        # Subscribe to joint states
        self.joint_state_sub = rospy.Subscriber(
            '/ur10e_robot/joint_states', 
            JointState, 
            self._joint_state_callback
        )
        
        # Subscribe to calf numbers
        self.calf_sub = rospy.Subscriber(
            'calf_num', 
            String, 
            self.process
        )

        # check if the platform is in home position, otherwise move it there
        if abs(self.platform_position - self.home_position) > 0.01:  # 1cm tolerance
            rospy.loginfo(f"Platform not at home ({self.platform_position:.3f}m), moving to home...")
            self._go_home()
            # Wait for platform to reach home
            rate = rospy.Rate(10)
            timeout = rospy.Duration(30)
            start_time = rospy.Time.now()
            while not rospy.is_shutdown():
                if abs(self.platform_position - 0.0) < 0.01:
                    rospy.loginfo("Platform returned to home position")
                    break
                if rospy.Time.now() - start_time > timeout:
                    rospy.logerr("Platform home movement timeout")
                    return
                rate.sleep()
        
        rospy.loginfo("Robot listener node started, waiting for calf numbers...")

    def _base2cow(self, cow_pos_end):
        sequence_base2cow = [
            ("platform", self.home_position)
            ("manipulator", INTERMEDIATE_GRASP),
            ("gripper", OPEN),
            ("manipulator", GRASP),
            ("gripper", CLOSE),
            ("manipulator", INTERMEDIATE_GRASP),
            ("platform", cow_pos_end),
            ("manipulator", INTERMEDIATE_PLACE),
            ("manipulator", PLACE),
            ("gripper", OPEN),
            ("manipulator", INTERMEDIATE_PLACE),
            ("manipulator", HOME)
        ]

        return sequence_base2cow
    
    def _cow2cow(self, cow_pos_start, cow_pos_end):
        sequence_cow2cow = [
            ("platform", cow_pos_start),
            ("manipulator", INTERMEDIATE_PLACE),
            ("gripper", OPEN),
            ("manipulator", PLACE),
            ("gripper", CLOSE),
            ("manipulator", INTERMEDIATE_PLACE),
            ("manipulator", INTERMEDIATE_GRASP),
            ("platform", cow_pos_end),
            ("manipulator", INTERMEDIATE_PLACE),
            ("manipulator", PLACE),
            ("gripper", OPEN),
            ("manipulator", INTERMEDIATE_PLACE)
            ("manipulator", HOME)
        ]

        return sequence_cow2cow
    
    def _cow2base(self, cow_pos_start):
        sequence_cow2base = [
            ("platform", cow_pos_start),
            ("gripper", OPEN),
            ("manipulator", INTERMEDIATE_PLACE),
            ("manipulator", PLACE),
            ("gripper", CLOSE),
            ("manipulator", INTERMEDIATE_PLACE),
            ("manipulator", INTERMEDIATE_GRASP),
            ("platform", self.home_position),
            ("manipulator", GRASP),
            ("gripper", OPEN),
            ("manipulator", INTERMEDIATE_GRASP)
            ("manipulator", HOME)
        ]

        return sequence_cow2base
    
    def _get_calf_position(self, calf_num: str):
        """Map calf number to platform position and move."""
        try:
            num = int(calf_num)
            if 0 <= num <= 9:
                position = positions.get(num, 0)
            elif num == -1:
                return self.home_position
            else:
                rospy.logwarn("Cow number out of range (0-10). Returning home")
                return -1
        except ValueError:
            rospy.logwarn("Invalid calf number received. Returning home")
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

    def _move_manipulator_to_pose(self, pose: str) -> bool:
        """Move the UR10e manipulator to the position named "pose" using MoveIt."""
        try:
            rospy.loginfo(f"Moving manipulator to {pose} position...")
            
            # Use the 'pose' named target defined in SRDF
            self.move_group.set_named_target(pose)
            
            # Plan and execute
            plan = self.move_group.go(wait=True)
            
            # Stop any residual movement
            self.move_group.stop()
            
            # Clear targets
            self.move_group.clear_pose_targets()
            
            if plan:
                rospy.loginfo(f"Manipulator successfully moved to {pose} position")
                return True
            else:
                rospy.logwarn(f"Failed to plan path to {pose} position")
                return False
                
        except Exception as e:
            rospy.logerr(f"Error moving manipulator to {pose}: {e}")
            return False
        
    def _move_gripper(self, pose: str) -> bool:
        """Opens or close the gripper"""
        try:
            rospy.loginfo(f"Moving gripper to {pose} position...")
            
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

    def process(self, calf_num_msg, task):
        """Implements the order of operations the robot has to do"""
        # get the starting and ending calf number
        calf_numbers = str(calf_num_msg.data)
        calf_num_start, calf_num_end = calf_numbers.split('-')

        calf_position_start = self._get_calf_position(calf_num_start)
        calf_position_end = self._get_calf_position(calf_num_end)
        for pos in [calf_position_start, calf_position_end]:
            if pos == -1:
                # invalid position, returns -1
                print(f"Invalid calf position: {pos}")
                return

        # Execute movement sequence
        if task == "base2cow":
            sequence = self._base2cow(calf_position_end)
        elif task == "cow2cow":
            sequence = self._cow2cow(calf_position_start, calf_position_end)
        elif task == "cow2base":
            sequence = self._cow2base(calf_position_start)
        else:
            print("ERROR: wrong task")
            return -1
        
        for action_type, target in sequence:
            rospy.loginfo(f"Executing {action_type} to {target}")
            if action_type == "manipulator":
                if not self._move_manipulator_to_pose(target):
                    rospy.logerr(f"Failed to move manipulator to {target}")
                    return
            elif action_type == "gripper":
                if not self._move_gripper(target):
                    rospy.logerr(f"Failed to move gripper to {target}")
                    return
            elif action_type == "platform":
                rospy.loginfo(f"Moving platform to position {target}...")
                if not move_platform(target):
                    rospy.logerr(f"Failed to move platform to {target}")
                    return
                # Wait for platform to reach target position
                rate = rospy.Rate(10)  # 10 Hz
                timeout = rospy.Duration(30)  # 30 second timeout
                start_time = rospy.Time.now()
                while not rospy.is_shutdown():
                    if abs(self.platform_position - target) < 0.01:  # 1cm tolerance
                        rospy.loginfo(f"Platform reached target position: {self.platform_position:.3f}m")
                        break
                    if rospy.Time.now() - start_time > timeout:
                        rospy.logerr("Platform movement timeout")
                        return
                    rate.sleep()
                # Update internal platform position tracker
                rospy.loginfo(f"Platform position updated: {self.platform_position:.3f}m")
            elif action_type == "wait":
                rospy.loginfo(f"Waiting {target} seconds for settling...")
                time.sleep(target)
                continue  # Skip the standard 1s pause
            time.sleep(1)  # brief pause between actions

        print("Movement sequence completed successfully. Ready for next command.")
    
    def run(self):
        """Keep the node running."""
        rospy.spin()

if __name__ == '__main__':
    try:
        pipeline = RobotMovementPipeline()
        pipeline.run()
    except rospy.ROSInterruptException:
        pass