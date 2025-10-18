# Bucket Physics Fix - Collision and Friction

## Problem
The bucket was slipping away during gripper pick-up operations because it had **no surface physics parameters** defined in its SDF model. This caused:
- Zero/minimal friction between gripper and bucket
- Poor contact response (objects passing through each other)
- Unstable grasping behavior

## Root Cause
The bucket model (`models/bucket/model.sdf`) was missing Gazebo surface properties:
- **No friction coefficients** (`mu`, `mu2`) on any collision geometries
- **No contact parameters** (`kp`, `kd`, `max_vel`, `min_depth`)
- Gripper had `mu1=2.0, mu2=2.0` but bucket had default (near-zero) values

## Solution Applied

### 1. Bucket Body Physics
Added moderate friction to the main bucket cylinder:
```xml
<surface>
  <friction>
    <ode>
      <mu>1.5</mu>        <!-- Coulomb friction coefficient -->
      <mu2>1.5</mu2>      <!-- Secondary friction direction -->
    </ode>
  </friction>
  <contact>
    <ode>
      <kp>1000000.0</kp>   <!-- Contact stiffness (spring constant) -->
      <kd>100.0</kd>       <!-- Contact damping -->
      <max_vel>0.1</max_vel>
      <min_depth>0.001</min_depth>
    </ode>
  </contact>
</surface>
```

### 2. Bucket Handle Physics
Added **high friction** (`mu=2.0`) to all 7 handle collision geometries:
- `bucket_handle_horiz_mid` (center horizontal bar)
- `bucket_handle_vertical_dn` (left vertical down)
- `bucket_handle_horiz_n` (left horizontal)
- `bucket_handle_vertical_un` (left vertical up)
- `bucket_handle_vertical_dp` (right vertical down)
- `bucket_handle_horiz_p` (right horizontal)
- `bucket_handle_vertical_up` (right vertical up)

**Rationale**: Higher friction on handle matches gripper finger friction (2.0) for secure gripping.

### 3. Gripper Hook Surfaces
Added Gazebo physics to gripper hook links that were missing properties:
- `left_finger_hook` and `right_finger_hook`
- `left_finger_hook_parallel` and `right_finger_hook_parallel`

All now have:
```xml
<mu1>2.0</mu1>
<mu2>2.0</mu2>
<kp>1000000.0</kp>
<kd>100.0</kd>
<minDepth>0.001</minDepth>
<maxVel>0.1</maxVel>
```

## Physics Parameters Explained

### Friction Coefficients
- **`mu` (mu1)**: Primary Coulomb friction coefficient (0.0 = ice, 1.0 = rubber on concrete)
- **`mu2`**: Secondary friction direction (for anisotropic surfaces)
- **Values used**:
  - Bucket body: `1.5` (moderate grip)
  - Bucket handle: `2.0` (high grip, matches gripper)
  - Gripper fingers/hooks: `2.0` (high grip)

### Contact Parameters
- **`kp`**: Contact stiffness (spring constant). Higher = more rigid contact
  - Value: `1000000.0` (very stiff, prevents sinking/interpenetration)
- **`kd`**: Contact damping. Prevents oscillation/bouncing
  - Value: `100.0` (moderate damping)
- **`max_vel`**: Maximum interpenetration correction velocity
  - Value: `0.1` m/s (prevents explosive separation)
- **`min_depth`**: Minimum penetration depth before contact force applies
  - Value: `0.001` m (1mm, allows small overlap for stable contact)

## Testing Instructions

### 1. Rebuild and Source
```bash
cd /home/vboxuser/lattebot_ws2
catkin_make
source devel/setup.bash
```

### 2. Launch Farm World
```bash
roslaunch pkg01 gazebo_farm.launch
```

### 3. Test Gripping
Try to grasp the bucket handle with the gripper. You should now observe:
- ✅ Gripper fingers hold the handle securely
- ✅ Bucket moves with gripper (no slipping)
- ✅ No interpenetration or ghost collisions
- ✅ Stable contact during lift/transport

### 4. Verify Bucket Position
The bucket spawns at position `(2, 2, 2)` in the farm world. Use MoveIt to plan gripper approach.

## Expected Behavior

### Before Fix
- Bucket would slip away when gripper closed
- Handle would pass through fingers
- Unstable/jittery contact
- Bucket would fall or slide out

### After Fix
- Solid contact between gripper and handle
- No slipping during lift
- Stable grasp throughout motion
- Predictable physics response

## Technical Details

### Friction Force Calculation
Contact friction force = `mu * normal_force`
- With `mu=2.0` and typical gripper closing force (~10-50N), friction can reach 20-100N
- This is sufficient to hold a 0.5kg bucket (requires ~5N to counteract gravity)

### Contact Stiffness Trade-offs
- **High `kp`** (1000000): Prevents sinking but may cause instability if too high
- **Moderate `kd`** (100): Damps oscillation without making contact spongy
- Balance chosen for stable manipulation without jitter

## Related Files Modified
1. `/home/vboxuser/lattebot_ws2/src/pkg01/models/bucket/model.sdf`
   - Added `<surface>` blocks to all 8 collision geometries
2. `/home/vboxuser/lattebot_ws2/src/pkg01/urdf/simple_gripper.urdf.xacro`
   - Added Gazebo properties for 4 hook links

## Troubleshooting

### If bucket still slips:
1. **Check Gazebo restart**: Models cache in Gazebo. Do a full restart:
   ```bash
   killall gzserver gzclient
   roslaunch pkg01 gazebo_farm.launch
   ```

2. **Increase gripper force**: Ensure gripper controller is applying sufficient closing effort

3. **Check collision visualization**: In Gazebo View menu, enable "Collisions" to verify geometry

4. **Verify friction applied**: In Gazebo, right-click bucket → View → Physics properties

### If bucket is too sticky:
- Reduce friction coefficients (try `mu=1.0` instead of `2.0`)
- Increase `max_vel` to allow easier separation

## Future Improvements
- Add material properties for different surface types (metal, plastic, wood)
- Implement variable friction based on contact force
- Add slip detection and adaptive grip adjustment
- Create physics presets for different object types

---
**Fix Date**: 2025-10-05  
**Issue**: Bucket slipping during gripper pick-up  
**Solution**: Added comprehensive surface physics to bucket and gripper models
