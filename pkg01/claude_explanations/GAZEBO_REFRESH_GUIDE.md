# Gazebo Model & World Refresh Guide

## Problem
Changes to `model.sdf`, `farm.world`, or other Gazebo files don't appear when relaunching `gazebo_farm.launch`.

## Root Cause
Gazebo caches models and worlds in memory. Simply stopping and restarting the launch file may not reload the files because:
1. Gazebo server processes may still be running in the background
2. Gazebo may cache model files in `~/.gazebo/models/`
3. ROS nodes may hold references to old model data

## Solutions

### Option 1: Use the Helper Script (Easiest)
```bash
cd /home/vboxuser/lattebot_ws2/src/pkg01/scripts
./restart_gazebo_farm.sh
```

This script automatically:
- Kills all Gazebo processes
- Clears cached models
- Sources the workspace
- Relaunches Gazebo with fresh files

### Option 2: Manual Process
If you prefer to do it manually:

```bash
# 1. Kill all Gazebo processes
killall -9 gzserver gzclient

# 2. Wait a moment for processes to fully terminate
sleep 1

# 3. Clear any cached models (optional but recommended)
rm -rf ~/.gazebo/models/bucket

# 4. Source workspace
cd /home/vboxuser/lattebot_ws2
source devel/setup.bash

# 5. Relaunch
roslaunch pkg01 gazebo_farm.launch
```

### Option 3: Use Ctrl+C Properly
When stopping Gazebo:
1. Press `Ctrl+C` in the terminal running the launch file
2. **Wait** for all nodes to shutdown (may take 5-10 seconds)
3. Verify processes are gone: `ps aux | grep gz`
4. If any remain, kill them: `killall -9 gzserver gzclient`
5. Relaunch

## Workflow for Editing Models

When modifying `model.sdf` or `farm.world`:

1. **Make your changes** to the files
2. **Save all files** (Ctrl+S)
3. **Kill Gazebo completely**: `killall -9 gzserver gzclient`
4. **Relaunch**: `roslaunch pkg01 gazebo_farm.launch`

## Verification

After relaunching, verify your changes:

### For `farm.world` changes (e.g., bucket position):
- Look at the bucket's position in Gazebo
- Check world coordinates match your `<pose>` tag

### For `model.sdf` changes (e.g., collision geometry):
- Enable collision visualization in Gazebo: View → Collisions
- Check if collision shapes match your SDF definitions
- Test physics interactions (e.g., can the robot grasp it?)

### For visual changes (e.g., mesh scale):
- Look at the bucket's visual appearance
- Verify scale, rotation, and mesh rendering

## Debugging Tips

### Check if Gazebo processes are running:
```bash
ps aux | grep gz
```

### Check if model is being found:
```bash
echo $GAZEBO_MODEL_PATH
ls /home/vboxuser/lattebot_ws2/src/pkg01/models/bucket/
```

### Check Gazebo logs:
```bash
# Launch with verbose output
roslaunch pkg01 gazebo_farm.launch verbose:=true
```

### Test world file directly:
```bash
gazebo /home/vboxuser/lattebot_ws2/src/pkg01/world/farm.world --verbose
```

## Common Issues

### Issue: "Model 'bucket' not found"
- **Cause**: GAZEBO_MODEL_PATH not set correctly
- **Fix**: Check launch file has: `<env name="GAZEBO_MODEL_PATH" value="$(find pkg01)/models:$(optenv GAZEBO_MODEL_PATH)"/>`

### Issue: Old bucket position/appearance persists
- **Cause**: Gazebo still running or cached
- **Fix**: `killall -9 gzserver gzclient` then relaunch

### Issue: Collision not working as expected
- **Cause**: Collision geometry not matching visual
- **Fix**: Enable View → Collisions in Gazebo to visualize collision shapes

### Issue: Mesh not loading
- **Cause**: Wrong mesh path in model.sdf
- **Fix**: Use `model://bucket/meshes/Bucket.dae` (not absolute paths)

## Files Affected by Caching

These files require complete Gazebo restart to reload:
- ✓ `/world/farm.world` - World file
- ✓ `/models/bucket/model.sdf` - Model definition
- ✓ `/models/bucket/model.config` - Model metadata
- ✓ `/models/bucket/meshes/Bucket.dae` - Mesh file

These files can sometimes reload without restart:
- ✗ `/urdf/ur10e.urdf.xacro` - Robot model (uses robot_description parameter)
- ✗ `/controller/ur10e_controllers.yaml` - Controller configs (loaded to parameter server)

## Best Practice

**Always use the helper script** when iterating on world/model changes:
```bash
# Edit your files
vim src/pkg01/world/farm.world
vim src/pkg01/models/bucket/model.sdf

# Relaunch properly
./src/pkg01/scripts/restart_gazebo_farm.sh
```

This ensures a clean restart every time!
