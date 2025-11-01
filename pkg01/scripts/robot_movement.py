#!/usr/bin/env python3
import sys
import time
import select
import termios
import tty

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
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
        
        # Initialize Gazebo delete model service
        rospy.wait_for_service('/gazebo/delete_model')
        self.delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        rospy.loginfo("Gazebo delete_model service ready")
        
        # Initialize Gazebo spawn model service
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        self.spawn_model_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        rospy.loginfo("Gazebo spawn_model service ready")
        
        # Store bucket model path
        import rospkg
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('pkg01')
        self.bucket_model_path = pkg_path + '/models/bucket/model.sdf'
        
        # Counter for spawning multiple buckets
        self.bucket_counter = 0
        
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
            ("platform", self.home_position),
            ("manipulator", INTERMEDIATE_GRASP),
            ("gripper", OPEN),
            ("manipulator", GRASP),
            ("wait", 3.0),  # Increased wait for gripper to stabilize around bucket handle
            ("gripper", CLOSE),
            ("wait", 4.0),  # Increased wait for grasp plugin to activate (40 update cycles)
            ("manipulator", INTERMEDIATE_GRASP),
            ("platform", cow_pos_end),
            ("manipulator", INTERMEDIATE_PLACE),
            ("manipulator", PLACE),
            ("wait", 2.0),  # Wait before releasing
            ("gripper", OPEN),
            ("wait", 2.0),  # Wait for release
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
            ("wait", 3.0),  # Increased wait for gripper to stabilize
            ("gripper", CLOSE),
            ("wait", 4.0),  # Increased wait for grasp plugin (40 update cycles)
            ("manipulator", INTERMEDIATE_PLACE),
            ("manipulator", INTERMEDIATE_GRASP),
            ("platform", cow_pos_end),
            ("manipulator", INTERMEDIATE_PLACE),
            ("manipulator", PLACE),
            ("wait", 2.0),  # Wait before releasing
            ("gripper", OPEN),
            ("wait", 2.0),  # Wait for release
            ("manipulator", INTERMEDIATE_PLACE),
            ("manipulator", HOME)
        ]

        return sequence_cow2cow
    
    def _cow2base(self, cow_pos_start):
        sequence_cow2base = [
            ("platform", cow_pos_start),
            ("gripper", OPEN),
            ("manipulator", INTERMEDIATE_PLACE),
            ("manipulator", PLACE),
            ("wait", 2.0),  # Wait for gripper to stabilize
            ("gripper", CLOSE),
            ("wait", 2.0),  # Wait for grasp plugin
            ("manipulator", INTERMEDIATE_PLACE),
            ("manipulator", INTERMEDIATE_GRASP),
            ("platform", self.home_position),
            ("manipulator", INTERMEDIATE_PLACE),
            ("manipulator", PLACE),
            ("wait", 1.0),  # Wait before releasing
            ("gripper", OPEN),
            ("wait", 1.0),  # Wait for release
            ("manipulator", INTERMEDIATE_PLACE),
            ("manipulator", HOME)
        ]

        return sequence_cow2base
    
    def _get_calf_position(self, calf_num: str):
        """Map calf number to platform position and move."""
        try:
            num = int(calf_num)
            if 0 <= num <= 9:
                position = positions.get(num, 0)
                return position
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
    
    def _delete_bucket_after_delay(self, delay_seconds=5.0):
        """Delete the bucket model from Gazebo after a specified delay."""
        rospy.loginfo(f"Scheduling bucket deletion in {delay_seconds} seconds...")
        time.sleep(delay_seconds)
        
        try:
            rospy.loginfo("Deleting bucket from Gazebo...")
            response = self.delete_model_service('bucket')
            if response.success:
                rospy.loginfo("Bucket successfully deleted from simulation")
            else:
                rospy.logwarn(f"Failed to delete bucket: {response.status_message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
    
    def _spawn_bucket(self):
        """Spawn a new bucket at the initial position."""
        try:
            # Read the bucket model SDF file
            with open(self.bucket_model_path, 'r') as f:
                bucket_sdf = f.read()
            
            # Increment counter for unique bucket name
            self.bucket_counter += 1
            bucket_name = f'bucket_{self.bucket_counter}'
            
            # Define bucket pose (initial position from farm.world)
            # <pose>-0.7 0 0 1.5707963267948966 0 1.5707963267948966</pose>
            bucket_pose = Pose()
            bucket_pose.position = Point(x=-0.7, y=0.0, z=0.0)
            
            # Quaternion for rotation (roll=90°, pitch=0°, yaw=90°)
            # Using the rotation from the world file
            import tf.transformations as tf_trans
            quaternion = tf_trans.quaternion_from_euler(1.5707963267948966, 0, 1.5707963267948966)
            bucket_pose.orientation = Quaternion(
                x=quaternion[0],
                y=quaternion[1],
                z=quaternion[2],
                w=quaternion[3]
            )
            
            rospy.loginfo(f"Spawning new bucket '{bucket_name}' at initial position...")
            response = self.spawn_model_service(
                model_name=bucket_name,  # Use unique name
                model_xml=bucket_sdf,
                robot_namespace='',
                initial_pose=bucket_pose,
                reference_frame='world'
            )
            
            if response.success:
                rospy.loginfo(f"Bucket '{bucket_name}' successfully spawned!")
            else:
                rospy.logwarn(f"Failed to spawn bucket '{bucket_name}': {response.status_message}")
                
        except FileNotFoundError:
            rospy.logerr(f"Bucket model file not found: {self.bucket_model_path}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        except Exception as e:
            rospy.logerr(f"Error spawning bucket: {e}")

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
            
            # Set reasonable tolerances
            self.move_group.set_goal_position_tolerance(0.01)
            self.move_group.set_goal_joint_tolerance(0.01)
            self.move_group.set_planning_time(10.0)
            
            # Use the 'pose' named target defined in SRDF
            self.move_group.set_named_target(pose)
            
            # Plan first to check if path is valid
            plan = self.move_group.plan()
            
            # Check if planning succeeded
            if isinstance(plan, tuple):
                success, trajectory, planning_time, error_code = plan
                if not success:
                    rospy.logwarn(f"Failed to plan manipulator movement to {pose} - error code: {error_code}")
                    return False
            
            # Execute
            success = self.move_group.go(wait=True)
            
            # Stop any residual movement
            self.move_group.stop()
            
            # Clear targets
            self.move_group.clear_pose_targets()
            
            if success:
                rospy.loginfo(f"Manipulator successfully moved to {pose} position")
                return True
            else:
                rospy.logwarn(f"Failed to execute manipulator movement to {pose}")
                return False
                
        except Exception as e:
            rospy.logerr(f"Error moving manipulator to {pose}: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
            return False
        
    def _move_gripper(self, pose: str) -> bool:
        """Opens or close the gripper"""
        try:
            rospy.loginfo(f"Moving gripper to {pose} position...")
            
            # Set increased tolerances for gripper
            self.gripper_group.set_goal_position_tolerance(0.01)  # 1cm tolerance
            self.gripper_group.set_goal_joint_tolerance(0.05)     # More relaxed joint tolerance
            self.gripper_group.set_planning_time(10.0)            # More time to plan
            
            # Slow down gripper movement for better contact detection
            self.gripper_group.set_max_velocity_scaling_factor(0.3)  # 30% of max speed
            self.gripper_group.set_max_acceleration_scaling_factor(0.3)  # 30% of max accel
            
            # Use the 'pose' named target defined in SRDF
            self.gripper_group.set_named_target(pose)
            
            # Plan first to check if path is valid
            plan = self.gripper_group.plan()
            
            # Check if planning succeeded (plan is a tuple in newer MoveIt versions)
            if isinstance(plan, tuple):
                success, trajectory, planning_time, error_code = plan
                if not success:
                    rospy.logwarn(f"Failed to plan gripper movement to {pose} - error code: {error_code}")
                    return False
            
            # Execute the plan
            success = self.gripper_group.go(wait=True)
            
            # Stop any residual movement
            self.gripper_group.stop()
            
            # Clear targets
            self.gripper_group.clear_pose_targets()
            
            if success:
                rospy.loginfo(f"Gripper successfully moved to {pose} position")
                # Add delay to ensure gripper settles and contacts stabilize
                time.sleep(1.0)
                return True
            else:
                rospy.logwarn(f"Failed to execute gripper movement to {pose}")
                return False
                
        except Exception as e:
            rospy.logerr(f"Error moving gripper to {pose}: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
            return False

    def process(self, calf_num_msg):
        """Implements the order of operations the robot has to do"""
        # get the starting and ending calf number
        calf_numbers = str(calf_num_msg.data)
        calf_num_start, calf_num_end = calf_numbers.split('_')

        calf_num_start = int(calf_num_start)
        calf_num_end = int(calf_num_end)

        if calf_num_start == -1:
            task = "base2cow"
        elif calf_num_end == -1:
            task = "cow2base"
        else:
            task = "cow2cow"

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
        
        # If this was a cow2base task (bucket placed at cow 0 position), schedule deletion
        if task == "cow2base":
            rospy.loginfo("Bucket placed - scheduling automatic deletion...")
            self._delete_bucket_after_delay(delay_seconds=5.0)
    
    def _keyboard_listener(self):
        """Listen for keyboard input in a separate thread."""
        rospy.loginfo("Keyboard listener started. Press 'a' to spawn bucket, 'q' to quit")
        
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            tty.setcbreak(sys.stdin.fileno())
            
            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    
                    if key.lower() == 'a':
                        rospy.loginfo("'a' pressed - spawning bucket...")
                        self._spawn_bucket()
                    # elif key.lower() == 'q':
                    #     rospy.loginfo("👋 'q' pressed - shutting down...")
                    #     rospy.signal_shutdown("User requested shutdown")
                    #     break
                        
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    def run(self):
        """Keep the node running with keyboard listener."""
        import threading
        
        # Start keyboard listener in separate thread
        keyboard_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        keyboard_thread.start()
        
        rospy.spin()

if __name__ == '__main__':
    try:
        pipeline = RobotMovementPipeline()
        pipeline.run()
    except rospy.ROSInterruptException:
        pass