#!/usr/bin/env bash
# Start the whole drone simulation: 4 terminals inside ONE tmux window,
# plus Gazebo / QGroundControl / RViz placed on the screen.
#
# Usage: ./start_drone.sh [world] [px4_target]
#   ./start_drone.sh                         -> world "custom", model gz_x500
#   ./start_drone.sh custom gz_x500_lidar_2d

# ---------------- EDIT THESE FOR YOUR MACHINE ----------------
SESSION=drone
PX4_DIR="$HOME/PX4-Autopilot"
WS="$HOME/ros2_ws"
PKG=my_robot_controller
WORLD="${1:-custom}"
TARGET="${2:-gz_x500}"
QGC="${QGC_PATH:-$HOME/Downloads/QGroundControl.AppImage}"
RVIZ_CFG="$WS/src/$PKG/my_robot_controller/drone.rviz"   
# --------------------------------------------------------------

ROS_SETUP="source /opt/ros/humble/setup.bash && source $WS/install/setup.bash"

# ---- clean up a previous run (also frees port 8888) ----
tmux kill-session -t "$SESSION" 2>/dev/null
pkill -x MicroXRCEAgent 2>/dev/null
sleep 1

# ---- create 4 panes in a 2x2 grid; keep their IDs ----
#   P0 Micro-XRCE-DDSAgent                               |   P1 PX4 + GAZEBO
#   P2 Position + Communication with RViz                |   P3 Keyboard control 
P0=$(tmux new-session -d -s "$SESSION" -x 200 -y 50 -P -F '#{pane_id}')
P1=$(tmux split-window -h -t "$P0" -P -F '#{pane_id}')
P2=$(tmux split-window -v -t "$P0" -P -F '#{pane_id}')
P3=$(tmux split-window -v -t "$P1" -P -F '#{pane_id}')

# ---- labels on the pane borders, mouse support (click a pane to focus it) ----
tmux set-option -t "$SESSION" mouse on
tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format " #{pane_title} "
tmux select-pane -t "$P0" -T "Micro-XRCE-DDSAgent"
tmux select-pane -t "$P1" -T "PX4 + GAZEBO"
tmux select-pane -t "$P2" -T "Position & Odom Node"
tmux select-pane -t "$P3" -T "Keyboard control"

# ---- start each program, in the right order ----
tmux send-keys -t "$P0" "MicroXRCEAgent udp4 -p 8888" C-m
tmux send-keys -t "$P1" "sleep 3; cd $PX4_DIR && PX4_GZ_WORLD=$WORLD make px4_sitl $TARGET" C-m
tmux send-keys -t "$P2" "$ROS_SETUP && ros2 run $PKG px4_to_odom" C-m
tmux send-keys -t "$P3" "$ROS_SETUP && sleep 25 && ros2 run $PKG control_node" C-m
tmux select-pane -t "$P3"

# ---- graphical apps run in the background ----
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
if [ -f "$RVIZ_CFG" ]; then
  nohup rviz2 -d "$RVIZ_CFG" > /tmp/rviz.log 2>&1 &
else
  echo "[start_drone] RViz config not found: $RVIZ_CFG (RViz opens empty)"
  nohup rviz2 > /tmp/rviz.log 2>&1 &
fi
if [ -x "$QGC" ]; then
  nohup "$QGC" > /tmp/qgc.log 2>&1 &
else
  echo "[start_drone] QGroundControl not found: $QGC (skipped)"
fi

# ---- one terminal window that shows the tmux session ----
gnome-terminal --title="DRONE-TERM" -- tmux attach -t "$SESSION" &


