# Copilot Instructions Update Summary

## Changes Made (October 18, 2025)

Updated `.github/copilot-instructions.md` to reflect the **Robotiq 2F-140 gripper integration** that replaced the custom simple gripper.

### Key Updates

1. **Project Overview**
   - Updated description: "Robotiq 2F-140 gripper" instead of "custom gripper"
   - Added `robotiq/robotiq_2f_140_gripper_visualization` to primary packages
   - Updated documentation path to `pkg01/claude_explanations/*.md`

2. **System Architecture**
   - Updated link hierarchy: `robotiq_arg2f_base_link` instead of `gripper_base`
   - Documented Robotiq linkage mechanism

3. **Robot Model Section**
   - Updated gripper include path to Robotiq package
   - Added gripper mount orientation: `rpy="0 0 ${PI/2}"`
   - Replaced custom gripper documentation with Robotiq 2F-140 details:
     - Single actuated joint (`finger_joint`)
     - Mimic joint mechanism
     - Complex link structure (knuckles, fingers, pads)
     - Plugin: `libroboticsgroup_gazebo_mimic_joint_plugin.so`

4. **Controller Configuration**
   - Updated transmission: Single `finger_joint` instead of two independent fingers
   - Updated controller specs: P=1000, I=50, D=100 (Robotiq-specific)
   - Documented mimic joint behavior

5. **MoveIt Integration**
   - Updated SRDF gripper group: `finger_joint` only
   - Added gripper states: `open` (0.0), `close` (0.7)
   - Updated collision rules documentation for Robotiq linkage

6. **Testing Section**
   - Replaced custom gripper tests with Robotiq gripper tests
   - Added `test_robotiq_gripper.py` script reference
   - Updated expected behavior: mimic joints follow automatically

7. **Critical Implementation Details**
   - Rewrote gripper control section for Robotiq
   - Single actuated joint with mimic constraints
   - Only `finger_joint` in trajectory commands

8. **Common Pitfalls**
   - Updated gripper-related pitfalls
   - Replaced rigidity issues with mimic joint plugin issues
   - Added Robotiq-specific troubleshooting

9. **New Section: Robotiq Package Structure**
   - Documented enabled packages (visualization only)
   - Listed disabled packages (hardware dependencies)
   - Explained simulation vs. real robot deployment pattern

## Preserved Content

- All platform-related documentation (unchanged)
- UR10e arm configuration (unchanged)
- MoveIt workflow patterns (unchanged)
- Gazebo model and world documentation (unchanged)
- Build and launch workflows (unchanged)
- Custom model physics requirements (unchanged)

## Validation

The updated instructions accurately reflect:
- Current URDF structure (`ur10e.urdf.xacro`)
- Controller configuration (`ur10e_controllers.yaml`)
- MoveIt configuration (`ur10e.srdf`, `ros_controllers.yaml`)
- Package dependencies (CATKIN_IGNORE files)
- Available test scripts (`test_robotiq_gripper.py`)

## References

- Full integration details: `pkg01/claude_explanations/ROBOTIQ_GRIPPER_INTEGRATION.md`
- Quick reference: `pkg01/claude_explanations/ROBOTIQ_QUICKREF.md`
- Test script: `pkg01/scripts/test_robotiq_gripper.py`

## Next Steps for Future Updates

When making significant changes to the system:
1. Update `.github/copilot-instructions.md` immediately
2. Add detailed explanation in `pkg01/claude_explanations/`
3. Update test scripts to reflect new behavior
4. Validate all code examples in instructions still work
5. Check for outdated terminology or deprecated patterns
