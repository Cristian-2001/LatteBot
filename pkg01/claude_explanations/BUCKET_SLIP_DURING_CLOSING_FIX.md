# Bucket Slipping Fix - Anti-Slip During Gripper Closing

## Problem
The bucket **slips away when the gripper closes**, even with high friction settings. The bucket moves out of position before a firm grasp can be established.

## Root Causes

### 1. **Gripper-Bucket Friction Imbalance**
- Bucket had `mu=10.0` friction
- Gripper had `mu=10.0` friction  
- **Equal friction = no grip dominance** - Bucket can slide along fingers during closing

### 2. **Insufficient Contact Stiffness on Gripper**
- Gripper: `kp=1000000` (1M)
- Bucket: `kp=1000000` (1M)
- **Equal stiffness = symmetric contact** - Neither surface "wins" during collision

### 3. **Contact Resolution Speed**
- `maxVel=0.001` was still too fast for gentle contact
- Rapid corrections cause impulse forces that push bucket away
- Physics solver needs more time per contact update

### 4. **Contact Detection Threshold**
- `minDepth=0.0001` (0.1mm) detection threshold
- For thin handle (15mm) and fine finger pads, need even finer detection

### 5. **Gripper Closing Dynamics**
- Even with 6-second goal time, initial contact forces can be large
- Two-finger closing creates lateral forces if not perfectly aligned
- Bucket has low mass (2kg), easily pushed by unbalanced forces

## Solution Applied

### A. Enhanced Gripper Physics (gripper_gazebo.xacro)

#### 1. EXTREME Friction on Finger Pads
```xml
<!-- Was: mu1=10.0, mu2=10.0 -->
<mu1>15.0</mu1>  <!-- 50% higher than bucket -->
<mu2>15.0</mu2>
```
**Effect**: Gripper "grabs" bucket surface instead of sliding along it

#### 2. Doubled Contact Stiffness
```xml
<!-- Was: kp=1000000 (1M) -->
<kp>2000000.0</kp>  <!-- 2M - gripper dominates contact -->
<kd>2000.0</kd>     <!-- Double damping too -->
```
**Effect**: Gripper surface acts "harder" than bucket, resists bucket motion

#### 3. Ultra-Slow Contact Correction
```xml
<!-- Was: maxVel=0.001 (1mm/s) -->
<maxVel>0.0005</maxVel>  <!-- 0.5mm/s - half the speed -->
```
**Effect**: Reduces impulse forces, smoother contact engagement

#### 4. Ultra-Fine Contact Detection
```xml
<!-- Was: minDepth=0.0001 (0.1mm) -->
<minDepth>0.00005</minDepth>  <!-- 0.05mm - twice as sensitive -->
```
**Effect**: Detects contact earlier, prevents deep interpenetration

#### 5. Disable Self-Collision
```xml
<selfCollide>false</selfCollide>
```
**Effect**: Prevents internal gripper links from interfering with each other

### B. Applied to All Gripper Contact Surfaces

**Finger Pads** (primary contact):
- `mu1/mu2=15.0` - EXTREME friction
- `kp=2M, kd=2000` - Maximum stiffness

**Inner/Outer Fingers** (secondary contact):
- `mu1/mu2=12.0` - Very high friction
- `kp=2M, kd=2000` - Same stiffness as pads

**Outer Knuckles**:
- `mu1/mu2=6.0` - Moderate friction
- `kp=1M, kd=1000` - Standard stiffness

### C. Gripper Controller Settings (Already Optimal)

Current settings in `ur10e_controllers.yaml`:
```yaml
gripper_controller:
  constraints:
    goal_time: 6.0  # Very slow 6-second closing
    finger_joint: {trajectory: 0.15, goal: 0.005}  # Tight tolerance
  
  gains:
    finger_joint: {p: 1500.0, i: 100.0, d: 150.0}  # High gains for full closure
```

**No changes needed** - Already configured for slow, controlled closing.

## Physics Comparison: Before vs After

| Component | Parameter | Before | After | Change |
|-----------|-----------|--------|-------|--------|
| **Gripper Pads** | Friction (mu) | 10.0 | 15.0 | +50% |
| | Stiffness (kp) | 1M | 2M | 2x |
| | Damping (kd) | 1000 | 2000 | 2x |
| | Max correction | 0.001 m/s | 0.0005 m/s | 2x slower |
| | Min detection | 0.1 mm | 0.05 mm | 2x finer |
| **Gripper Fingers** | Friction (mu) | 8.0 | 12.0 | +50% |
| | Stiffness (kp) | 1M | 2M | 2x |
| | Damping (kd) | 1000 | 2000 | 2x |
| **Bucket Handle** | Friction (mu) | 10.0 | 10.0 | No change |
| | Stiffness (kp) | 1M | 1M | No change |

**Result**: Gripper now has **grip dominance** over bucket.

## Testing Instructions

### Quick Test with Monitoring Script

```bash
# 1. Launch simulation
roslaunch pkg01 gazebo_farm.launch

# 2. In Gazebo: Enable collision visualization
#    View → Collisions (see pink boxes on handle and fingers)

# 3. Position gripper around bucket handle
#    - Use MoveIt or manual joint control
#    - Handle should be between fingers (not just tips)
#    - Gripper perpendicular to handle

# 4. Run gentle grasp test
rosrun pkg01 test_gentle_grasp.py
```

The script will:
1. Open gripper fully
2. Wait for you to position it around bucket
3. Close in gradual steps (20% → 40% → 60% → 80%)
4. Monitor position at each step
5. Hold for 5 seconds to verify stability

### Manual Testing

```bash
# Open gripper
rostopic pub -1 /ur10e_robot/gripper_controller/command trajectory_msgs/JointTrajectory "{
  joint_names: ['finger_joint'],
  points: [{positions: [0.0], time_from_start: {secs: 3}}]
}"

# Position around bucket (use MoveIt)

# Close gripper SLOWLY (6 seconds)
rostopic pub -1 /ur10e_robot/gripper_controller/command trajectory_msgs/JointTrajectory "{
  joint_names: ['finger_joint'],
  points: [{positions: [0.56], time_from_start: {secs: 6}}]
}"
```

### Verification Checklist

**Before Closing:**
- [ ] Gripper positioned around handle (not bucket body)
- [ ] Handle visible between left and right fingers
- [ ] Small gap on each side (1-2cm)
- [ ] Gripper perpendicular to handle (90° approach)

**During Closing:**
- [ ] Bucket remains stationary (no lateral movement)
- [ ] No sudden jumps or vibrations
- [ ] Smooth, gradual contact engagement
- [ ] Collision boxes (pink) overlapping properly

**After Closing:**
- [ ] Bucket firmly held
- [ ] Handle compressed between fingers
- [ ] No slipping during 5-second hold
- [ ] Finger position stable (not drifting)

## Troubleshooting

### Problem: Bucket Still Slips Away

**Possible Causes:**

#### 1. Misalignment
- **Symptom**: Bucket pushed sideways immediately on contact
- **Check**: Gripper not perpendicular to handle
- **Fix**: Approach from different angle, ensure 90° to handle

#### 2. Poor Position
- **Symptom**: Bucket slips through fingers
- **Check**: Handle not between fingers, only touching fingertips
- **Fix**: Move gripper down so handle is in middle of finger length

#### 3. Gazebo Cache Issue
- **Symptom**: Changes to URDF/SDF not taking effect
- **Check**: Old physics parameters still loaded
- **Fix**: 
  ```bash
  killall -9 gzserver gzclient
  rm -rf ~/.gazebo/models/bucket
  cd ~/lattebot_ws2
  catkin_make
  source devel/setup.bash
  roslaunch pkg01 gazebo_farm.launch
  ```

#### 4. Too Fast Closing
- **Symptom**: Bucket fine until gripper moves, then slides
- **Check**: Using default trajectory timing instead of slow close
- **Fix**: Ensure 6+ second duration in trajectory commands

#### 5. Collision Disabled
- **Symptom**: Gripper passes through bucket
- **Check**: Collisions not enabled, no contact forces
- **Fix**: Enable collision visualization, verify pink boxes present

### Problem: Bucket Vibrates/Jitters

**Cause**: Stiffness too high, physics solver oscillating  
**Fix**: Reduce gripper stiffness slightly:
```xml
<kp>1500000.0</kp>  <!-- Was 2M, try 1.5M -->
```

### Problem: Gripper Opens During Hold

**Cause**: Controller gains too low, can't maintain position under load  
**Fix**: Already at high gains (P=1500), check for:
- Joint limits reached
- Hardware interface errors
- Controller timing issues

### Problem: Bucket Jumps on Initial Contact

**Cause**: Large impulse force from sudden contact  
**Fix**: Increase damping on bucket handle:
```xml
<!-- In bucket model.sdf -->
<kd>2000.0</kd>  <!-- Was 1000, match gripper -->
```

## Advanced Tuning

### If Bucket is Too Slippery

Increase gripper friction even more:
```xml
<mu1>20.0</mu1>  <!-- From 15.0 -->
<mu2>20.0</mu2>
```

### If Gripper Too Gentle (Doesn't Close Fully)

Increase controller gains:
```yaml
finger_joint: {p: 2000.0, i: 150.0, d: 200.0}  # From 1500/100/150
```

### If Bucket Moves During Lift (Not During Close)

That's a different issue - see `GRIPPER_LIFT_FIX.md`. This fix addresses:
- ✅ Slipping during closing
- ❌ Slipping during lift (different solution needed)

### If Need Faster Closing (Trade-off)

Reduce goal time, but increases slip risk:
```yaml
goal_time: 4.0  # From 6.0, but less stable
```

## Related Issues

- **Bucket passes through gripper**: See `BUCKET_PASSTHROUGH_FIX.md`
- **Bucket slips during lift**: See `GRIPPER_LIFT_FIX.md`  
- **Gripper doesn't close fully**: See `GRIPPER_CLOSING_FIX.md`
- **Physics parameters**: See `PHYSICS_PARAMS_QUICKREF.md`

## Implementation Details

### Why Higher Gripper Friction Works

**Physical Intuition**:
- Two surfaces in contact: gripper (mu=15) vs bucket (mu=10)
- During closing, fingers slide along handle
- Friction force = mu × normal_force
- Higher mu on gripper = more resistance to sliding
- Bucket "sticks" to fingers instead of sliding away

**Key**: Asymmetric friction favors gripper motion, opposes bucket motion.

### Why Higher Stiffness on Gripper Works

**Contact Mechanics**:
- When two bodies collide, both deform
- Stiffer body (higher kp) deforms less
- Softer body (lower kp) deforms more
- Net effect: gripper "pushes into" bucket less
- Bucket pushed less → stays in place better

**Key**: Gripper acts as "rigid jaw", bucket as "soft object".

### Why Slower Correction Velocity Works

**Solver Dynamics**:
- `maxVel` limits how fast solver corrects interpenetration
- Fast correction = large impulse force = bucket jumps
- Slow correction = gentle force = smooth contact
- Physics solver has more iterations to resolve contact

**Key**: Gives physics time to find equilibrium gradually.

## Summary

**Core Principle**: Make gripper "stickier" and "harder" than bucket.

**Implementation**:
1. ✅ Gripper friction 50% higher than bucket (15 vs 10)
2. ✅ Gripper stiffness 2x bucket (2M vs 1M)  
3. ✅ Ultra-slow contact correction (0.5mm/s)
4. ✅ Ultra-fine contact detection (0.05mm)
5. ✅ Slow gripper closing (6 seconds)

**Expected Outcome**: Bucket remains stationary during closing, firm grasp achieved.

---

**Fix Date**: 2025-10-19  
**Issue**: Bucket slips away when gripper closes  
**Status**: Physics parameters updated, test script created  
**Critical**: Must rebuild workspace and clear Gazebo cache for changes to take effect
