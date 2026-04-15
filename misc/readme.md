# CS 424/524 – Intelligent Mobile Robotics
## Assignment 2 – `p2_dthirukondac`

### Team / Collaborators
| Name   | Role          |
|--------|---------------|
| Dilip  | Developer     |
| Riddhi | Collaborator  |

---

### Package Overview

| Part | File | Description |
|------|------|-------------|
| 1 | `script/navigation_node.py` | Autonomous waypoint navigation (L1→L2→L3→L1) via `move_base` |
| 2 | `src/vision_follower.cpp`   | Depth-primary red-ball follower; maintains ~1 m standoff |

---

### Quick Start

**Build:**
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

**Part 1 – Autonomous Navigation:**
```bash
roslaunch p2_dthirukondac p2a.launch
```

**Part 2 – Ball Follower:**
```bash
roslaunch p2_dthirukondac p2b.launch
```

---

### Configuration Notes

- **Waypoints** (`script/navigation_node.py`): Update the `WAYPOINTS` dictionary with real map-frame coordinates before running on the physical robot. Coordinates should be ~25 ft (7.62 m) apart.
- **Camera topics** (`launch/p2b.launch`): Set `depth_topic`, `rgb_topic`, and `cmd_vel_topic` args to match your robot's actual published topics.
- **Bonus (20 %)**: `vision_follower` uses depth as the primary signal. It works even when the RGB camera is covered or lights are off.

---

### Misc Folder Contents

| File | Purpose |
|------|---------|
| `readme.md` | This file |
| `message.txt` | Message text (fill in as needed) |
| `initials_placeholder.txt` | Reminder to upload initials image |
