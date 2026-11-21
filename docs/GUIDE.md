# self_driving_car - complete guide (simulation + real kids ride-on car)

One package, two worlds. The simulation files are unchanged; the real-car files are added next to them.

## 1. Launch files - what to run for what

| What you want | Command |
|---|---|
| **SIM** - build a map | `ros2 launch self_driving_car sim_mapping.launch.py` |
| **SIM** - navigate to a goal on the saved map | `ros2 launch self_driving_car sim_navigation.launch.py` (same as `autonomous_car.launch.py`) |
| **SIM** - only Gazebo + robot | `ros2 launch self_driving_car launch_sim.launch.py` |
| **REAL** - hardware test (lidar + ESP32 + TF, no SLAM/Nav2) | `ros2 launch self_driving_car real_bringup.launch.py` |
| **REAL** - build a map | `ros2 launch self_driving_car real_mapping.launch.py` |
| **REAL** - navigate to a goal on the saved map | `ros2 launch self_driving_car real_navigation.launch.py map:=$HOME/car_maps/my_map.yaml` |

Real-car settings live in `config/real/` (car.yaml = the one file you edit). Simulation settings stay in `config/` and `config/nav2/`.

## 2. One-time install (Ubuntu 22.04 + ROS 2 Humble)

    sudo apt update
    sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
         ros-humble-nav2-smac-planner ros-humble-teleop-twist-keyboard ros-humble-rplidar-ros \
         ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros2-control \
         ros-humble-ros2-controllers ros-humble-xacro python3-serial
    sudo usermod -aG dialout $USER        # then log out and in again (needed for /dev/ttyUSB*)

    mkdir -p ~/car_ws/src
    cd ~/car_ws/src && unzip ~/Downloads/self_driving_car_full.zip      # creates src/self_driving_car
    ls ~/car_ws/src/self_driving_car                                     # must show package.xml
    cd ~/car_ws && source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source install/setup.bash
    echo "source ~/car_ws/install/setup.bash" >> ~/.bashrc

Run every command from a terminal where `source ~/car_ws/install/setup.bash` was done.

## 3. SIMULATION

### 3.1 Map
    ros2 launch self_driving_car sim_mapping.launch.py
Wait ~20 s (Gazebo + RViz open). In a second terminal:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard
Press `u` / `o` (forward + left/right). Drive the whole track slowly, finish where you started.
In RViz: Fixed Frame = `map`, add `Map` (/map) to watch it grow.

Save the map (second terminal):

    ros2 run nav2_map_server map_saver_cli -f ~/car_ws/src/self_driving_car/maps/my_map
Press Ctrl+C on the mapping launch.

### 3.2 Navigate
    ros2 launch self_driving_car sim_navigation.launch.py
Wait ~25 s. In RViz click **2D Pose Estimate** where the car is, then **Nav2 Goal**.
Put an object in front of the car in Gazebo to test avoidance.

## 4. REAL CAR - hardware

### 4.1 Parts
* Kids ride-on car: 4 DC motors, front Ackermann steering linkage, battery (normally 12 V)
* 1 x L298N module  (see the current warning below)
* 1 x ESP32 DevKit + **USB data cable** to the computer
* 1 x 25 kg steering servo (6 V class, e.g. DS3225) with metal horn
* 1 x buck converter 6.0 V, **>= 5 A** for the servo (e.g. XL4015)
* 1 x buck converter 5 V >= 3 A for the computer (skip if you use a laptop / power bank)
* 1 x 1000-2200 uF 16 V capacitor (across the servo supply), 1 x 15 A fuse, 1 x kill switch (>= 20 A)
* 1 x RPLIDAR A1 (or A2) with its USB adapter, 1 x computer with Ubuntu 22.04 (Raspberry Pi 4/5 8 GB, mini-PC or laptop)

### 4.2 !!! L298N current warning (read this) !!!
An L298N handles about **2 A continuous (3 A peak) per channel** and loses ~2 V. Each channel here runs TWO
ride-on motors in parallel. Ride-on motors usually take 3-6 A each when running and 15 A+ when starting or
stalled. That is 3-10 times too much: the L298N will overheat, shut down or burn.
* Measure the real current first (clamp meter, or a 20 A inline meter) with the car on the ground.
* For a car that carries a child, use a **BTS7960 / IBT-2 (43 A) driver** instead (one per side). Tell me and I will
  change the firmware for it - the rest of the system stays the same.
* Until then: test lifted off the ground or empty, very low speed, short runs, feel the heat sink.

### 4.3 Block diagram

    BATTERY + --[KILL SWITCH]--[FUSE 15A]--+--------------------> L298N  "+12V" terminal
                                           +--> BUCK-A (set 6.0 V) --> SERVO  red (+)   (+ 1000uF capacitor)
                                           +--> BUCK-B (5 V) -------> computer power (or use laptop battery)
    BATTERY - ------------------------------+--> L298N  GND
                                             +--> BUCK-A GND --> SERVO brown/black (-)
                                             +--> BUCK-B GND
    ESP32 GND --------------------------------> L298N GND         <- signal ground, MANDATORY

    computer --USB data cable--> ESP32          (power + serial for the ESP32)
    computer --USB-------------> RPLIDAR adapter

### 4.4 ESP32 -> L298N / servo (pin table)

| ESP32 pin | Goes to | Purpose |
|---|---|---|
| GPIO25 | L298N **ENA** | front motors speed (PWM) |
| GPIO26 | L298N **IN1** | front motors direction |
| GPIO27 | L298N **IN2** | front motors direction |
| GPIO33 | L298N **ENB** | rear motors speed (PWM) |
| GPIO14 | L298N **IN3** | rear motors direction |
| GPIO13 | L298N **IN4** | rear motors direction |
| GPIO18 | Servo signal (orange/yellow) | steering |
| GND | L298N GND **and** servo supply GND | common ground |
| USB | computer | power + data |

* **Remove the two jumpers on ENA and ENB** of the L298N, otherwise PWM speed control does nothing.
* Keep the 5 V-enable jumper on the L298N (battery <= 12 V) - it powers the L298N logic. Leave the L298N "+5V" pin **unconnected** (the ESP32 gets its power from USB).
* L298N **OUT1 / OUT2** -> both FRONT motors in parallel (all M+ together, all M- together).
* L298N **OUT3 / OUT4** -> both REAR motors in parallel.
* Servo: red -> BUCK-A output +, brown/black -> BUCK-A output -, signal -> GPIO18. Never power the servo from the ESP32 pin.
* Motor wires must be thick (>= 1.5 mm2 / 16 AWG); keep signal wires away from motor wires.
* All grounds (battery -, L298N, buck converters, servo, ESP32) must meet. No common ground = random behaviour.

### 4.5 Before the first power-on
1. Lift the car so all wheels are in the air. Kill switch OFF.
2. Check polarity of every connection twice. Check with a multimeter that BUCK-A outputs 6.0 V BEFORE connecting the servo.
3. Servo horn: centre the servo (it centres itself when the ESP32 boots at 97 deg), then attach the horn so the wheels point straight.
   Check the steering linkage does not hit its end stops at 77 deg and 117 deg (if it does, reduce the range).

## 5. REAL CAR - software

### 5.1 Flash the ESP32
Arduino IDE -> Boards Manager: install "esp32" (Espressif) -> Library Manager: install "ESP32Servo" ->
open `firmware/esp32_car/esp32_car.ino` -> board "ESP32 Dev Module" -> select the port -> Upload.
(Linux: if the upload fails, hold the BOOT button while "Connecting..." appears.)

### 5.2 Bench test with the Serial Monitor (no ROS)
Power only via USB first (kill switch OFF): the Serial Monitor (115200, "Newline") shows `READY`.
Now switch the battery ON and type:

    C,0,0        servo goes to centre (97 deg)
    C,0,1        steering full LEFT  (servo 77)
    C,0,-1       steering full RIGHT (servo 117)
    C,0.3,0      slow forward (stops by itself after 0.4 s - that is the safety watchdog)
Send `C,0.3,0` repeatedly (or hold Enter) while checking:
* both front and rear wheels turn FORWARD. If one pair turns backwards -> set `INVERT_A` or `INVERT_B` to `true` in the sketch (or swap that pair's wires).
* if it hums but does not move: raise `MIN_PWM` (70 -> 90).
* steering left/right is the right way round (`C,0,1` must turn the wheels to the LEFT when looking from behind the car). If not, swap `SERVO_LEFT` and `SERVO_RIGHT` values.

### 5.3 Measure the car and edit `config/real/car.yaml`
| Key | How to get it |
|---|---|
| `wheelbase` | distance front axle centre -> rear axle centre (m) |
| `front_overhang`, `rear_overhang`, `width` | from the REAR axle centre to the front / back of the body; full width (m) |
| `max_steer_deg` | real WHEEL angle when the servo is at 77 / 117 deg (phone protractor, wheels lifted) |
| `v_at_full_cmd` | put the car on the floor, send `C,1,0` for 3 s several times, measure distance / 3 s. Do this at the same battery charge level |
| `lidar_x/y/z/yaw` | lidar position from the rear axle centre (x forward, y left, z up) |
| `esp32_port`, `lidar_port` | `ls -l /dev/serial/by-id/` and use the long `by-id` names so they never swap |
| `max_speed` | keep 0.3-0.4 m/s until everything works |

Minimum turning radius = wheelbase / tan(max_steer_deg). Example: 0.76 / tan(20 deg) = 2.1 m. The car cannot turn
tighter than that, so plan open spaces. Nav2 reads all of this automatically - you do not change Nav2 files.

### 5.4 Mount the lidar
Flat, level, at the top, nothing blocking the 360 deg view (pillars of the roof, your cables). Cable-tie USB cables
so they cannot touch the motors or wheels.

### 5.5 Hardware test with ROS
    ros2 launch self_driving_car real_bringup.launch.py
Expected: `ESP32 connected`, the lidar starts spinning. In another terminal:

    ros2 topic echo /scan --once                       # lidar data
    ros2 run teleop_twist_keyboard teleop_twist_keyboard   # drive with u / o / i, speed ~0.2
    ros2 topic echo /odom                              # changes while driving

### 5.6 Map the room / track
    ros2 launch self_driving_car real_mapping.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard        # second terminal: u / o keys
* Speed about 0.2-0.3 m/s, turn value about 0.1 (turn bigger = hits steering limit).
* Drive along walls, make loops/overlaps so slam_toolbox can close loops. Avoid driving fast or sliding.
* View on a laptop: `rviz2` -> Fixed Frame `map` -> add Map `/map`, LaserScan `/scan`.
* When the map looks good:

      mkdir -p ~/car_maps
      ros2 run nav2_map_server map_saver_cli -f ~/car_maps/my_map
      # creates my_map.yaml + my_map.pgm
Then stop the mapping launch with Ctrl+C.

### 5.7 Navigate to a goal
    ros2 launch self_driving_car real_navigation.launch.py map:=$HOME/car_maps/my_map.yaml
In RViz (Fixed Frame `map`; add Map, LaserScan, Path, `/global_costmap/costmap`, `/local_costmap/costmap`):
1. Click **2D Pose Estimate**, click where the car really is and drag in the direction it faces. Wait until the green particles collapse into a tight cloud (drive a metre if needed).
2. Click **Nav2 Goal** and set the goal.
3. The car plans a smooth path (minimum turning radius respected) and follows it. With an obstacle: it re-plans around it if there is free space.
Stop-and-wait mode: in `launch/real_navigation.launch.py` set `USE_WAIT_BT = True`, rebuild.

### 5.8 Watching from a laptop
Both machines on the same Wi-Fi, same domain: `export ROS_DOMAIN_ID=7` on both (put it in `~/.bashrc`), then run `rviz2` on the laptop.
If topics do not appear, disable firewall (`sudo ufw disable`) or use a hotspot.

## 6. Tuning cheat sheet

| Problem | Change |
|---|---|
| Car turns more/less than planned | correct `max_steer_deg` (measure the real wheel angle) |
| Robot thinks it moved further/shorter than it did | correct `v_at_full_cmd` |
| Map is bent / doubled | slower driving, more loops, correct `v_at_full_cmd` and `max_steer_deg` |
| Starts avoiding too late | `obstacle_max_range` in `config/real/nav2_params.yaml` (global_costmap) |
| "No path" in narrow places | lower `inflation_radius` (0.9 -> 0.7) in both costmaps; remember the 2.1 m turning radius |
| Jerky start/stop | lower `accel_limit` in car.yaml |
| Steering too jittery | lower `steer_rate_limit` |
| Too slow | raise `max_speed` slowly (0.4 -> 0.6) |

## 7. Troubleshooting

| Symptom | Fix |
|---|---|
| `cannot open /dev/ttyUSB0` | wrong port / not in dialout group; use `/dev/serial/by-id/...`; log out and in after `usermod` |
| ESP32 restarts when ROS starts | normal first time only; bridge keeps DTR low; if it persists add a 10 uF capacitor between EN and GND on the ESP32 |
| Motors twitch then nothing | MIN_PWM too low, or battery weak, or L298N overloaded (see 4.2) |
| Steering buzzes / jitters | servo supply too weak: add capacitor, thicker wires, >= 5 A buck |
| Lidar not spinning | wrong `lidar_port` / baud (A1/A2 = 115200, A3/S1 = 256000), power the adapter from a USB 3 / powered hub |
| RViz "Frame [map] does not exist" | slam / amcl not running yet, or no 2D Pose Estimate set |
| `bt_navigator` stays `inactive` | `ros2 lifecycle get /bt_navigator`; read the first red error in the terminal; keep `USE_WAIT_BT = False` |
| Car does not move when goal is set | `ros2 topic echo /cmd_vel` - empty = Nav2 problem (look at planner/controller messages); not empty but no motion = bridge / ESP32 / battery |
| SIM: laser not seen in costmap | check `/scan` frame is `laser_frame` and TF exists |

## 8. Safety checklist
- [ ] Kill switch within reach, tested
- [ ] Fuse installed
- [ ] First tests with wheels lifted, then on the floor in an empty area
- [ ] Nobody sits in the car while it is autonomous
- [ ] ESP32 watchdog tested: unplug the USB cable while moving -> motors stop within 0.4 s
- [ ] Speed limited (`max_speed`) until mapping and navigation are reliable
- [ ] L298N temperature checked after 1 minute of driving

## 9. Web dashboard (phone / tablet / laptop)
Drive and watch the car from a browser, select the map, set the initial pose and goals, see lidar and the lane camera:
`ros2 launch self_driving_car real_dashboard.launch.py map:=$HOME/car_maps/my_map.yaml` then open `http://<car-ip>:8080`.
Full instructions: `docs/DASHBOARD.md`.

## 10. Reverse-and-turn (K-turn)
When the goal is behind the car (or it needs more room than one forward turn), the car now plans and drives **forward arc -> stop -> reverse -> stop -> forward arc**, repeated as often as the plan needs.
* **Planner:** `SmacPlannerHybrid` with `REEDS_SHEPP` (was `DUBIN`, which never reverses) - `config/real/nav2_params.yaml`.
* **Controller:** Regulated Pure Pursuit with `allow_reversing: true` (DWB cannot reverse). `use_rotate_to_heading` must stay `false`.
* **car_bridge:** *steers first* - before it starts, or switches between forward and reverse, the car waits (max 1.5 s) until the servo is within `steer_settle_deg` of the commanded angle. Reverse is limited to `max_reverse_speed` (0.20 m/s). Both are in `config/real/car.yaml`.
* The reverse **distance** comes from the planned path, not a timer (at 0.2 m/s, 2 s = 0.4 m). Repeating = the next plan contains another reverse segment.
* **Space:** the planner uses a turning radius of ~2.4 m, so a K-turn needs very roughly 5 m of free width. In a narrower place the planner reports "no path".
* Set the goal **with a direction** (drag the arrow): the final heading is part of the plan now.

| Symptom | Try |
|---|---|
| Reverses too often / for tiny turns | `reverse_penalty` 3 - 5 (planner section) |
| Never reverses | `reverse_penalty` 1.3 - 1.5; check the planner log says REEDS_SHEPP |
| Chatters at the forward/reverse change | `lookahead_dist` 0.5 |
| Waits too long before moving | lower `steer_settle_timeout`, or raise `steer_settle_deg` |
| Controller log: "Disabling reversing" | `use_rotate_to_heading` is true somewhere - it must be false |

Go back to forward-only: copy `config/real/nav2_params_forward_only_BACKUP.yaml` over `nav2_params.yaml` and, in `launch/real_navigation.launch.py`, replace the two `desired_linear_vel` / `regulated_linear_scaling_min_radius` lines with
`c['max_vel_x'] = vmax; c['max_speed_xy'] = vmax; c['max_vel_theta'] = round(vmax / r_min, 3); c['acc_lim_x'] = acc; c['decel_lim_x'] = -acc * 1.5`.

## 11. Obstacles: divert first, wait + reverse only when stuck - and straight driving
Behavior tree: `behavior_trees/navigate_divert_reverse.xml` (chosen by `USE_DIVERT_BT = True` in `launch/real_navigation.launch.py`).

**What the car does when the lidar sees something on its path**
1. **Divert:** the lidar obstacle enters the costmap (seen from `obstacle_max_range: 5.0` m), the old path becomes invalid, a new path *around* the obstacle is planned and followed. It does not wait for the obstacle to leave.
2. **Too close to divert** (no path / the controller stops before a collision): **wait 3 s** -> try again. Still blocked -> **reverse 0.5 m** (straight, slow, `backup_speed` 0.15 m/s), try to plan a way around again; up to 5 reverses (2.5 m back). If something is behind the car and it cannot reverse it waits 3 s and tries again.
3. After 7 such steps the goal is given up (dashboard: FAILED). The dashboard shows "obstacle: wait/reverse step N" while this runs.
4. Reversing uses the normal bridge rules: wheels are turned first (straight for BackUp), reverse speed is capped at 0.2 m/s, and Nav2 refuses to reverse into something it sees.

**Straight driving / no weaving** - two causes were fixed:
* The old trees planned a new path every 0.5 - 1 s; every new plan starts with a slightly different curve and the car follows each one (left, then right...). Now the path is planned once and re-planned **only when it is blocked** (or a new goal arrives).
* Pure-pursuit lookahead is longer (`lookahead_dist` 1.2 m, was 0.8) - the same twitchiness as a short lookahead on a long car. Still weaves: raise it (1.4 - 1.6). Cuts corners / wide turns: lower it (1.0).

| Want | Change |
|---|---|
| Wait longer / shorter before reversing | `<Wait wait_duration="3"/>` in the tree |
| Reverse further per step / more steps | `backup_dist`, number of `<BackUp>` lines, `number_of_retries` |
| Keep more distance from obstacles | `inflation_radius` in both costmaps (0.9 now) - too big = no path in narrow places |
| See obstacles earlier | `obstacle_max_range` (global costmap, 5.0 m) |
| Old behaviour | `USE_DIVERT_BT = False` |

If this Nav2 has no `IsPathValid` node the launch file prints a message and uses `navigate_divert_reverse_basic.xml`: same wait/reverse logic but it re-plans once per second (a little more weaving).
