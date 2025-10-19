# Quick Reference: Balanced Gripper-Handle Physics

## Current Configuration (Prevents Both Slipping AND Explosions)

### Bucket Handle Contact Physics
```xml
<mu>5.0</mu>                     <!-- Friction coefficient -->
<mu2>5.0</mu2>
<kp>500000.0</kp>                <!-- Contact stiffness (500k) -->
<kd>500.0</kd>                   <!-- Damping -->
<max_vel>0.01</max_vel>          <!-- Max correction velocity -->
<min_depth>0.0005</min_depth>    <!-- Min penetration depth (0.5mm) -->
```

### Gripper Finger Physics (NEW)
```xml
<!-- Finger pads (primary contact) -->
<mu1>5.0</mu1>
<mu2>5.0</mu2>
<kp>500000.0</kp>
<kd>500.0</kd>
<maxVel>0.01</maxVel>
<minDepth>0.0005</minDepth>

<!-- Inner/outer fingers -->
<mu1>4.0</mu1>
<mu2>4.0</mu2>
<kp>500000.0</kp>
<kd>500.0</kd>
```

## Files Modified
- ✅ `models/bucket/model.sdf` - Updated all 7 handle collision segments
- ✅ `urdf/gripper_gazebo.xacro` - NEW file with gripper physics
- ✅ `urdf/ur10e.urdf.xacro` - Includes gripper_gazebo.xacro

## Testing
```bash
# Quick restart with cache clear
./scripts/test_gripper_physics.sh

# Or manual:
killall -9 gzserver gzclient
rm -rf ~/.gazebo/models/bucket
cd /home/vboxuser/lattebot_ws2
source devel/setup.bash
roslaunch pkg01 gazebo_farm.launch
```

## Tuning Guide

### If handles still slip:
- Increase `mu` to 7.0 or 10.0 (both bucket and gripper)
- Increase `kp` to 750000 (more rigid)

### If explosive separation returns:
- Decrease `kp` to 350000 or 250000 (more compliant)
- Increase `kd` to 1000 (more damping)
- Decrease `max_vel` to 0.005 (slower correction)

## Key Insight
**Both surfaces need high friction!** Friction is a two-surface property - having high friction only on the bucket handles wasn't enough. Adding matching high friction to the gripper fingers completes the secure grasp system.

## Physics Scale
- `kp=10M` → Ultra-rigid (old) ❌ Explosive
- `kp=500k` → Balanced (current) ✅ Stable + Secure
- `kp=5k` → Too soft (your test) ❌ Slipping
