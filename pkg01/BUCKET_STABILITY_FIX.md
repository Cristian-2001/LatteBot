# Bucket Stability Fix - Static Object Solution

## Problem
The bucket was moving on its own in the Gazebo simulation, drifting or sliding unexpectedly even when not being manipulated by the robot.

## Root Cause
The issue was caused by two factors:
1. **Dynamic object instability**: The bucket was set to `<static>false</static>`, making it a fully dynamic physics object susceptible to numerical instability
2. **Over-constrained contact parameters**: Extremely high stiffness values (`kp=50000000.0`) and very restrictive contact parameters (`max_vel=0.001`, `min_depth=0.00001`) were causing numerical instability in the physics solver

## Solution Applied

### 1. Set Bucket to Static
Changed the bucket model from dynamic to static:
```xml
<static>true</static>  <!-- Static until grasped - prevents drifting -->
```

**Effect**: The bucket now remains perfectly stationary on the ground until the robot interacts with it. This is appropriate since the bucket should rest in place until grasped.

### 2. Normalized Contact Physics Parameters
Updated all collision geometries (bucket body + all handle sections) with stable, standard contact parameters:

**Bucket Body**:
- Friction: `mu=1.5`, `mu2=1.5` (moderate friction for body)
- Contact stiffness: `kp=1000000.0` (reduced from 50M)
- Contact damping: `kd=100.0` (reduced from 5000)
- Max velocity: `max_vel=0.1` (increased from 0.001)
- Min depth: `min_depth=0.001` (increased from 0.00001)

**Handle Sections**:
- Friction: `mu=2.0`, `mu2=2.0` (high friction for gripping)
- Same contact parameters as body (stable values)

**Removed parameters**:
- `fdir1` (friction direction vector)
- `soft_cfm` and `soft_erp` (constraint force mixing)

These simplified, stable parameters prevent physics solver instability while maintaining good contact behavior.

## Files Modified
- `/home/vboxuser/lattebot_ws2/src/pkg01/models/bucket/model.sdf`

## Testing
After applying the fix and restarting Gazebo:
```bash
cd /home/vboxuser/lattebot_ws2/src/pkg01/scripts
./restart_gazebo_farm.sh
```

The bucket should now:
- ✅ Remain perfectly stationary on the ground
- ✅ Not drift, slide, or move unexpectedly
- ✅ Still be graspable by the robot (static objects can be grasped in Gazebo)
- ✅ Have stable contact physics when manipulated

## Technical Notes

### Why Static Works for Graspable Objects
In Gazebo, `<static>true</static>` doesn't prevent grasping - it prevents the object from being affected by gravity and external forces when not in contact. Once the gripper makes contact and applies forces, the object can still be manipulated through the physics engine.

### Contact Parameter Guidelines
For stable Gazebo simulations:
- **Contact stiffness (kp)**: 10^6 is standard; higher values can cause instability
- **Contact damping (kd)**: 100-1000 range provides good damping without over-constraint
- **Max velocity (max_vel)**: 0.1 allows realistic contact without artificial speed limits
- **Min depth (min_depth)**: 0.001 provides reasonable penetration depth for stable contacts

### When to Use Dynamic vs Static
- **Static**: Objects that should rest in place (furniture, tools on shelf, etc.)
- **Dynamic**: Objects that need to respond to gravity/external forces immediately (falling objects, rolling balls, etc.)

## Alternative Solution (Not Used)
If you need the bucket to be dynamic (respond to gravity, roll, etc.) while preventing drift:
1. Keep `<static>false</static>`
2. Use moderate contact parameters (as applied)
3. Increase mass significantly (e.g., 10-20 kg)
4. Ensure the ground plane has matching friction properties

For this use case, static is the simpler and more stable solution.

## Related Documentation
- See `BUCKET_PHYSICS_FIX.md` for earlier friction/contact tuning
- See `GAZEBO_REFRESH_GUIDE.md` for cache clearing procedures
