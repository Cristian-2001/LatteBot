# Mobile Platform - Troubleshooting & Fixes

## Issues Resolved

### Issue 1: "This robot has a joint named 'platform_joint' which is not in the gazebo model"

**Root Cause:** 
The URDF elements were in the wrong order. The `robot_mount_to_base` joint tried to reference `robot_mount` link before it was created by the `mobile_platform` macro.

**Fix Applied:**
Reordered the URDF structure in `ur10e.urdf.xacro`:
1. First: Include and instantiate `mobile_platform` macro
2. Then: Connect robot `base_link` to `robot_mount` 
3. Finally: Include gripper

**Before (wrong order):**
```xml
<joint name="robot_mount_to_base" .../>  <!-- ❌ robot_mount doesn't exist yet -->
<xacro:mobile_platform prefix=""/>       <!-- Creates robot_mount here -->
```

**After (correct order):**
```xml
<xacro:mobile_platform prefix=""/>       <!-- Creates robot_mount first -->
<joint name="robot_mount_to_base" .../>  <!-- ✓ robot_mount now exists -->
```

### Issue 2: Static Link with Controlled Joint

**Root Cause:**
The `world_platform` link had `<static>true</static>` defined in the main URDF file, but Gazebo cannot control joints connected to static links via `ros_control`.

**Fix Applied:**
- Removed the `<gazebo reference="world_platform">` block from `ur10e.urdf.xacro`
- Moved it into the `mobile_platform.urdf.xacro` macro where it belongs
- This ensures proper scoping and makes the platform macro self-contained

**Key Principle:**
In Gazebo, a static link acts as the fixed world. Joints connecting to it can exist in the kinematic chain, but ros_control expects to control only non-static links. The `world_platform` being static is correct—it anchors the platform to the ground.

### Issue 3: "Robot semantic description not found"

**Root Cause:**
This is actually a **warning** (not critical error) when launching without MoveIt. However, the SRDF file referenced a `world` frame that no longer exists.

**Fix Applied:**
Updated `/ur10e_moveit_config/config/ur10e.srdf`:
- Changed virtual joint parent from `world` to `world_platform`
- Added collision disable rules for platform links

**Before:**
```xml
<virtual_joint name="virtual_joint" type="fixed" parent_frame="world" child_link="base_link"/>
```

**After:**
```xml
<virtual_joint name="virtual_joint" type="fixed" parent_frame="world_platform" child_link="base_link"/>
```

## Verification Steps

After fixes, verify with:

```bash
cd ~/lattebot_ws2
source devel/setup.bash

# 1. Check URDF compiles
rosrun xacro xacro src/pkg01/urdf/ur10e.urdf.xacro > /tmp/test.urdf
check_urdf /tmp/test.urdf

# 2. Verify kinematic chain
# Should show: world_platform -> platform_base -> robot_mount -> base_link -> ...

# 3. Check for platform_joint
grep "platform_joint" /tmp/test.urdf

# 4. Check transmission
grep "platform_trans" /tmp/test.urdf
```

## Current Robot Structure (Correct)

```
world_platform (static, Gazebo anchor)
  └─ platform_joint (prismatic, 0-10m, controlled by ros_control)
      └─ platform_base (moving platform, 50kg)
          └─ platform_to_mount (fixed joint)
              └─ robot_mount (mounting plate)
                  └─ robot_mount_to_base (fixed joint)
                      └─ base_link (UR10e arm base)
                          └─ [6-DOF arm joints...]
                              └─ [gripper...]
```

## Key Learnings

### 1. Xacro Processing Order Matters
Links and joints must be defined before being referenced. Use xacro macros early in the file if other elements depend on them.

### 2. Static Links in Gazebo
- `<static>true</static>` makes a link fixed in world coordinates
- Joints connected to static links exist in URDF but ros_control cannot actuate them
- Solution: Static parent + controlled joint + moving child ✓

### 3. Transmission Requirements
Every joint controlled by ros_control needs:
1. A `<joint>` definition with limits
2. A `<transmission>` element
3. A matching controller in the YAML config
4. The joint must NOT be on a static link

### 4. SRDF Virtual Joints
When using MoveIt, the SRDF virtual joint must reference a link that exists in the URDF. Update it when changing the base structure.

## Testing the Fix

Launch the simulation:
```bash
roslaunch pkg01 gazebo_ur10e.launch
```

**Expected behavior:**
- ✓ Gazebo launches without errors
- ✓ All controllers load successfully
- ✓ Platform appears in Gazebo
- ✓ Robot is mounted on platform
- ⚠️ "robot_description_semantic" warning is OK (only needed for MoveIt)

Test platform control:
```bash
# In new terminal (after sourcing workspace)
rosrun pkg01 move_platform.py _position:=5.0
```

**Expected behavior:**
- ✓ Platform moves smoothly along X-axis
- ✓ Robot moves with platform
- ✓ No errors in terminal

## Files Modified in Fix

1. **pkg01/urdf/ur10e.urdf.xacro**
   - Reordered: platform include before robot mounting
   - Removed: `<gazebo reference="world_platform">` block

2. **pkg01/urdf/mobile_platform.urdf.xacro**
   - Added: Gazebo static property for world_platform inside macro

3. **ur10e_moveit_config/config/ur10e.srdf**
   - Updated: virtual_joint parent_frame to world_platform
   - Added: collision disable rules for platform links

## Additional Notes

### Why Not Make world_platform Non-Static?

If `world_platform` is not static, Gazebo will apply physics to it (gravity, etc.), and it will fall through the ground. The static property keeps it fixed as a world reference point.

### Platform Joint Control Flow

```
Controller YAML (platform_controller)
    ↓
ROS Control (controller_manager)
    ↓
Gazebo ros_control plugin
    ↓
Transmission (platform_trans)
    ↓
Joint (platform_joint) - NOT on static link ✓
    ↓
Link (platform_base) - moves in simulation
```

### Future Enhancements

- Add joint limits enforcement in controller
- Add visual range markers in Gazebo
- Create coordinated platform+arm trajectory planner
- Add safety stops at 0m and 10m positions

---

**Status:** ✅ All issues resolved. Platform ready for use.
**Last Updated:** October 4, 2025
