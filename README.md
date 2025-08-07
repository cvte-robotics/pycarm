# pycarm

Python interface for cvte arm.

# Install

```
pip install carm
```

# Usage

```
import carm

arm = carm.Carm("ws://localhost:8090")

print("version:",carm.version)
print("limits:", carm.limit)
print("state:", carm.state)

carm.track_joint(carm.joint_pos)

carm.move_joint(carm.joint_pos)

carm.track_pose(carm.cart_pose)

carm.move_pose(carm.cart_pose)
```


