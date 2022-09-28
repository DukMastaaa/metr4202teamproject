# MUST DO
- clarify x, y, and z directions by echoing to /desired_pose
    - y is upward (opposite gravity)
    - x is facing forward from base
    - by right-hand rule, z must be to the right
- see if we need to change any of the IK code
- transformation from camera to joint 1
- camera recognises aruco tag and pose
- camera recognises colour
- make a way to save location of drop-off points, possibly by moving arm to correct position instead of hard-coding?
- ensure gripper works as expected

# NICE TO HAVE
- URDF model would be nice, to set up rviz so we can see the motion from home
- set up package structure and launch file (so we don't need to open 8 terminals lmao)

# Task Summary
All blocks have randomised colours.
1. rotating, 1 block
2. rotating, 3 blocks
3. two parts:
    a. not rotating, 4 blocks possibly obstructing each other
    b. rotating but stops for 1 second, 3 blocks