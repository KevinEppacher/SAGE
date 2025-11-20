#!/bin/bash

SESSION=sage

echo "Starting new tmux session '${SESSION}'..."
tmux kill-session -t $SESSION 2>/dev/null

###########################################################
# Create session with pane 0
###########################################################
tmux new-session -d -s $SESSION -n main

###########################################################
# Create all additional panes (total 5)
###########################################################
tmux split-window -h   -t $SESSION:0        # Pane 1
tmux split-window -v   -t $SESSION:0.0      # Pane 2
tmux split-window -v   -t $SESSION:0.1      # Pane 3
tmux split-window -v   -t $SESSION:0.3      # Pane 4

tmux select-layout -t $SESSION:0 tiled

###########################################################
# EXECUTE ENTRYPOINT via /entrypoint.sh (guaranteed path)
###########################################################

###########################################################
# Pane 0 — LAVIS
###########################################################
tmux send-keys -t $SESSION:0.0 "docker exec -it ros2_lavis_container bash
" C-m
tmux send-keys -t $SESSION:0.0 "ros2 launch blip2_ros blip2_ros.launch.py"

###########################################################
# Pane 1 — EXPLOITATION
###########################################################
tmux send-keys -t $SESSION:0.1 "docker exec -it ros2_exploitation_container bash" C-m
tmux send-keys -t $SESSION:0.1 "ros2 launch sage_commander exploitation_ws.launch.py"

###########################################################
# Pane 2 — EXPLORER
###########################################################
tmux send-keys -t $SESSION:0.2 "docker exec -it ros2_explorer_container bash" C-m
tmux send-keys -t $SESSION:0.2 "ros2 launch sage_commander explorer_ws.launch.py"

###########################################################
# Pane 3 — DETECTION
###########################################################
tmux send-keys -t $SESSION:0.3 "docker exec -it ros2_detection_container bash" C-m
tmux send-keys -t $SESSION:0.3 "ros2 launch sage_commander detection_ws.launch.py"

###########################################################
# Pane 4 — NVTOP
###########################################################
tmux send-keys -t $SESSION:0.4 "nvtop" C-m

###########################################################
# Attach
###########################################################
tmux select-pane -t $SESSION:0.0
tmux attach -t $SESSION
