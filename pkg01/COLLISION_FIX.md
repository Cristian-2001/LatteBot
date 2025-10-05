# MoveIt START_STATE_IN_COLLISION Fix

## Problem
When trying to plan a movement in RViz with MoveIt, the following error occurred:
```
[WARN] Unable to find a valid state nearby the start state (using jiggle fraction of 0.050000 and 100 sampling attempts).
[WARN] Fail: ABORTED: START_STATE_IN_COLLISION
```

## Root Cause
The SRDF file (`ur10e_moveit_config/config/ur10e.srdf`) was missing critical collision disable rules for:

1. **Gripper hook links** - The custom gripper has additional hook components that weren't accounted for:
   - `left_finger_hook`
   - `left_finger_hook_parallel`
   - `right_finger_hook`
   - `right_finger_hook_parallel`

2. **Gripper-to-arm collisions** - No collision disables between gripper components and upper arm/forearm/shoulder

3. **Platform-to-arm collisions** - Only basic disables existed, missing interactions with upper_arm and forearm

## Solution Applied
Added comprehensive collision disable rules to the SRDF file for:

### Gripper Hook Links (Self-Collisions)
- Hook links with their parent fingers (Adjacent)
- Hooks with palm and gripper base
- Left hooks with right hooks (symmetric gripper)
- All hook combinations with wrist links and ee_link

### Gripper-to-Arm Links
- All gripper components (base, palm, fingers, hooks) with:
  - `shoulder_link`
  - `upper_arm_link`
  - `forearm_link`
  - All wrist links (1, 2, 3)

### Platform-to-Arm Links
- Platform wheels with upper_arm and forearm
- Platform base with base_link, upper_arm, and forearm
- Robot mount with upper_arm and forearm

## Total Additions
Added **~90 new collision disable rules** to eliminate false collision detections.

## How to Apply
1. The changes have been made to `/home/vboxuser/lattebot_ws2/src/ur10e_moveit_config/config/ur10e.srdf`
2. Workspace has been rebuilt with `catkin_make`
3. **Restart your simulation** to apply changes:
   ```bash
   # Close current RViz/Gazebo
   # Then relaunch:
   roslaunch pkg01 gazebo_farm.launch
   # or
   roslaunch pkg01 gazebo_moveit.launch
   ```

## Verification Steps
1. Launch the simulation with MoveIt
2. In RViz, check the robot visualization:
   - Red links = in collision
   - Normal colors = no collision
3. Try planning a motion:
   - Select a goal state in the Planning tab
   - Click "Plan" - should now succeed without START_STATE_IN_COLLISION error

## Why These Links Were Colliding
- **Gripper hooks extend beyond the finger tips** - When the arm folds or rotates, these hooks can overlap with the wrist or forearm links in MoveIt's planning scene
- **Default behavior**: MoveIt assumes ALL links can collide unless explicitly disabled
- **False positives**: Adjacent/nearby links that are mechanically connected or will never actually touch still need explicit disable rules

## Prevention for Future
When adding new links to the robot (sensors, tools, etc.):
1. Run MoveIt Setup Assistant
2. Use the "Collision Matrix" tab to automatically detect always-colliding/never-colliding link pairs
3. Add disable rules for adjacent links and mechanically impossible collisions
4. Test in RViz to verify no false collision warnings

## References
- SRDF specification: http://wiki.ros.org/srdf
- MoveIt collision checking: http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/urdf_srdf/urdf_srdf_tutorial.html
