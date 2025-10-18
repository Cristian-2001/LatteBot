# Compliant Gripper Quick Reference

## What Changed?

**The gripper now uses COMPLIANT GRASPING to prevent handle jumping:**
- ✅ Controller gains reduced 10× (P: 5000→500, I: 200→20, D: 100→50)
- ✅ Joint dynamics softened (damping: 50→10, effort: 2000→500, velocity: 0.5→0.2)
- ✅ Grasp position loosened (gap: 16mm→24mm, -0.012m→-0.008m)
- ✅ Trajectory timing adjusted (slower approach, more permissive tolerance)

## Key Parameters

### Controller Gains (`pkg01/controller/ur10e_controllers.yaml`)
```yaml
gripper_controller:
  gains:
    left_finger_joint:  {p: 500.0, i: 20.0, d: 50.0, i_clamp: 50.0}
    right_finger_joint: {p: 500.0, i: 20.0, d: 50.0, i_clamp: 50.0}
  constraints:
    goal_time: 3.0
    stopped_velocity_tolerance: 0.05
```

### Joint Dynamics (`pkg01/urdf/simple_gripper.urdf.xacro`)
```xml
<limit lower="-0.025" upper="0.070" effort="500" velocity="0.2"/>
<dynamics damping="10.0" friction="5.0"/>
```

### Grasp Position (`ur10e_moveit_config/config/ur10e.srdf`)
```xml
<group_state name="grasp_handle" group="gripper">
    <joint name="left_finger_joint" value="-0.008"/>
    <joint name="right_finger_joint" value="-0.008"/>
</group_state>
```

## Quick Test

```bash
# 1. Launch simulation
source devel/setup.bash
./src/pkg01/scripts/restart_compliant_test.sh

# 2. Run automated test
rosrun pkg01 test_compliant_grasp.py

# Expected: Stable gripper motion with std_dev < 1mm
```

## Manual Test in RViz

1. **Move to grasp pose** (manipulator group → "grasp" state)
2. **Close gripper** (gripper group → "grasp_handle" state)
3. **Observe:** Smooth closing, no jumping, handle stays put

## Troubleshooting

### Still seeing jumping?
→ Reduce gains further in `ur10e_controllers.yaml`:
```yaml
p: 500.0 → 300.0
i: 20.0 → 10.0
```

### Gripper won't close fully?
→ This is normal with compliant control!
→ Check actual position: `rostopic echo /ur10e_robot/joint_states`
→ Acceptable range: -0.006m to -0.010m

### Handle slips when lifting?
→ Tighten slightly: -0.008m → -0.009m or -0.010m
→ Check blocking pads are present (80×80mm at finger tips)
→ Verify friction: μ=3.0 on fingers and pads

## Why It Works

**Before:** Stiff gripper + high gains + tight grip = force explosion → jumping
**After:** Soft gripper + low gains + gentle grip = compliant contact → stable

The key insight: **friction holds better than force**
- With μ=3.0 friction, light pressure (100N) can hold 300N load
- Blocking pads prevent geometric escape
- No need to crush the handle!

## Related Files

- `GRIPPER_JUMPING_FIX.md` - Detailed explanation
- `GRIPPER_ANTI_SLIP_FIX.md` - Blocking pad design
- `test_compliant_grasp.py` - Automated test script
- `restart_compliant_test.sh` - Quick restart for testing

## Status

✅ **FIXED** - Compliant grasping implemented (2025-10-13)

Handle jumping eliminated through:
- Reduced controller gains (10× softer)
- Compliant joint dynamics (5× lower damping)
- Gentler grasp position (50% more clearance)
- Slower approach velocity (2.5× slower)
