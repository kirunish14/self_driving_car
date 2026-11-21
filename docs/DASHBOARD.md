# Web dashboard + indoor lane camera

Drive the car from a **phone, tablet or laptop browser** - no RViz needed.
The dashboard only *talks to* your running navigation (same topics RViz uses). No navigation / driving logic is changed.

## 1. Start

```bash
cd ~/ros2_ws && colcon build --packages-select self_driving_car && source install/setup.bash

# car + navigation + dashboard (replaces real_navigation.launch.py)
ros2 launch self_driving_car real_dashboard.launch.py map:=$HOME/car_maps/my_map.yaml
```
Open **`http://<car-ip>:8080`** on any device on the same Wi-Fi (find the IP with `hostname -I`).
Tip: make the car computer a Wi-Fi hotspot, or give it a fixed IP, so the address never changes.
No internet is needed on the phone - everything is served by the car.

| Option | Meaning |
|---|---|
| `map:=...yaml` | map the navigation starts with |
| `pin:=1234` | the page asks for this PIN. **Use it when other people share the Wi-Fi - anyone who opens the page can drive the car.** |
| `port:=8080` | web port |
| `lane:=true camera:=0` | also start the USB camera + lane detection (video shows on the dashboard) |
| `nav:=false` | dashboard only (navigation already running in another terminal) |

## 2. Using the page

1. **Pick a map** in the top bar and press **Load map** (maps are found in `~/car_maps/` and the package `maps/` folder). Not possible while the car is navigating.
2. **Set pose** - touch where the car really is on the map and *drag toward the direction it faces*, release. (Same as RViz "2D Pose Estimate".) The chip turns green: **localized**. Repeat if the lidar dots do not match the walls.
3. **Set goal** - touch the destination and drag toward the direction the car should face at the end, release. The car drives; you see the car, the green path, the orange goal, the lidar (red dots), speed and distance left.
4. **STOP** (big red button, or `Esc` on a keyboard) cancels the goal and sends zero speed. It is always visible.
5. **Camera** opens the lane-detection video; the square icon makes it bigger.
6. Move the view: drag with one finger / mouse, pinch or mouse-wheel to zoom, **Fit map** resets, **Follow car** keeps the car centred.
7. **Confirm goals** (checkbox) asks "Go / Discard" before the car moves.

After loading a different map you must set the initial pose again (AMCL starts over).

## 3. Indoor lane camera (detection only)

`indoor_lane.py` finds the corridor floor, both wall lines and a centre path in a camera picture and gives a steering value (-1 left ... +1 right). **It only detects - it does not steer the car.**

```bash
ros2 launch self_driving_car indoor_lane.launch.py                 # USB camera 0 (/dev/video0)
ros2 launch self_driving_car indoor_lane.launch.py device:=1       # another camera
ros2 launch self_driving_car indoor_lane.launch.py width:=480 publish_fps:=10     # lighter on a Raspberry Pi
```
Topics: `/indoor_lane/image/compressed` (video with the drawing) and `/indoor_lane/steer` (Float32).
Test on a recorded video without ROS: `python3 self_driving_car/indoor_lane.py --source Indoor.mp4`.
Useful options: `hood:=0.8` (row where the car bonnet starts in the picture), `car_x:=0.5` (car centre in the picture), `fourcc:=''` if the camera does not support MJPG.

## 4. Start automatically when the car powers on (optional)

`/etc/systemd/system/car-dashboard.service`
```ini
[Unit]
Description=Car navigation + dashboard
After=network-online.target
Wants=network-online.target

[Service]
User=YOUR_USER
Environment=HOME=/home/YOUR_USER
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source /home/YOUR_USER/ros2_ws/install/setup.bash && exec ros2 launch self_driving_car real_dashboard.launch.py map:=/home/YOUR_USER/car_maps/my_map.yaml pin:=1234 lane:=true'
Restart=on-failure

[Install]
WantedBy=multi-user.target
```
`sudo systemctl daemon-reload && sudo systemctl enable --now car-dashboard`

## 5. Troubleshooting

| Symptom | Fix |
|---|---|
| Page does not open | same Wi-Fi? `hostname -I` on the car; firewall: `sudo ufw allow 8080/tcp` |
| Red dot in the top bar / "no connection" | launch stopped, or Wi-Fi dropped - the page reconnects by itself |
| "not localized - set pose" | normal until you set the pose; if it never turns green check that AMCL runs: `ros2 topic echo /amcl_pose --once` |
| Goal: "Nav2 is not running" | navigation not started / still starting (wait for `bt_navigator` active) |
| Car appears but lidar dots do not match the walls | set the pose again, more exactly (position **and** direction) |
| Map list empty | put the `.yaml` + `.pgm` pairs in `~/car_maps/` |
| "Load map" fails: map_server not running | start with `real_dashboard.launch.py` (or `real_navigation.launch.py`) |
| Camera panel says "no camera video" | `lane:=true` or run `indoor_lane.launch.py`; check `ls /dev/video*`; `sudo usermod -aG video $USER` and log in again; `ros2 topic hz /indoor_lane/image/compressed` |
| Video is slow on the Pi | `width:=480 publish_fps:=8` (and keep `fourcc:=MJPG`) |

## 6. What was added to the package

New files only: `dashboard_node.py`, `web/` (the page), `self_driving_car/indoor_lane.py` (your file, unchanged), `self_driving_car/indoor_lane_node.py` (camera + ROS wrapper), `launch/real_dashboard.launch.py`, `launch/indoor_lane.launch.py`, this document.
`CMakeLists.txt` / `package.xml` only install the new files and list their dependencies (`python3-opencv`, `nav2_msgs`, ...).
