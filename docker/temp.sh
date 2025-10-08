#!/bin/bash

SESSION=sage

echo "Starting new tmux session '${SESSION}'..."

# Kill existing session if it exists to ensure a fresh start
tmux kill-session -t $SESSION 2>/dev/null

# Create new session with one window
tmux new-session -d -s $SESSION

# Split into 4 panes
tmux split-window -h -t $SESSION:0.0
tmux split-window -v -t $SESSION:0.0
tmux split-window -v -t $SESSION:0.1

# Set tiled layout
tmux select-layout -t $SESSION:0 tiled

# Send commands to each pane
tmux send-keys -t $SESSION:0.0 "docker exec -it -u root ros2_lavis_container bash -c 'cd /app && source install/setup.bash && clear && ros2 launch blip2_ros blip2_ros.launch.py'" C-m
tmux send-keys -t $SESSION:0.1 "docker exec -it -u root ros2_detection_container bash -c 'cd /app && source install/setup.bash && clear && ros2 run gui gui'" C-m
tmux send-keys -t $SESSION:0.2 "docker exec -it -u root ros2_explorer_container bash -c 'cd /app && source install/setup.bash && clear && exec bash'" C-m
tmux send-keys -t $SESSION:0.3 "docker exec -it -u root ros2_exploitation_container bash -c 'cd /app && source install/setup.bash && clear && exec bash'" C-m

# Attach to session
tmux attach -t $SESSION