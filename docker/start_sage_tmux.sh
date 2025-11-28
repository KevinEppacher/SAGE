#!/bin/bash

SESSION=sage

echo "Starting necessary Docker containers..."
docker compose up -d detection_container explorer_container lavis_container isaac_sim exploitation_container
echo "Docker containers started."

# Load a minimal dark tmux theme dynamically
tmux set-option -g status-bg colour236       # dark gray background
tmux set-option -g status-fg colour250       # light gray text
tmux set-option -g status-attr dim
tmux set-option -g pane-border-style fg=colour238
tmux set-option -g pane-active-border-style fg=colour39   # bright blue border
tmux set-option -g message-style bg=colour39,fg=colour231
tmux set-option -g status-left-length 40
tmux set-option -g status-right-length 100
tmux set-option -g status-left '#[fg=colour39]#[bg=colour39,fg=black] SAGE #[fg=colour39,bg=colour236,nobold,nounderscore,noitalics]'
tmux set-option -g status-right '#[fg=colour250]#H #[fg=colour239]| #[fg=colour39]%Y-%m-%d %H:%M'
tmux set-option -g mouse on
tmux set-option -g status-left '#[fg=colour33]#[bg=colour33,fg=black] 🧠 SAGE #[fg=colour33,bg=colour236,nobold,nounderscore,noitalics]'
tmux set -g status-right '#[fg=colour39]⚙  SAGE │  #(nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits | head -1)% │  %H:%M │ %Y-%m-%d '


echo "Starting new tmux session '${SESSION}'..."
tmux kill-session -t $SESSION 2>/dev/null

###########################################################
# MAIN WINDOW (Window 0)
# Pane mapping:
#   0.0 → LAVIS
#   0.1 → EXPLOITATION
#   0.2 → EXPLORER
#   0.3 → DETECTION
#   0.4 → NVTOP
###########################################################

tmux new-session -d -s $SESSION -n main

tmux split-window -h   -t $SESSION:0        # Pane 1
tmux split-window -v   -t $SESSION:0.0      # Pane 2
tmux split-window -v   -t $SESSION:0.1      # Pane 3

tmux select-layout -t $SESSION:0 tiled

# Pane 0 — LAVIS
tmux send-keys -t $SESSION:0.0 "docker exec -it ros2_lavis_container bash" C-m
tmux send-keys -t $SESSION:0.0 "clear" C-m
tmux send-keys -t $SESSION:0.0 "echo -e '\033[1;36m[Starting LAVIS...]\033[0m'" C-m
tmux send-keys -t $SESSION:0.0 "ros2 launch blip2_ros blip2_ros.launch.py"

# Pane 1 — EXPLOITATION
tmux send-keys -t $SESSION:0.1 "docker exec -it ros2_exploitation_container bash" C-m
tmux send-keys -t $SESSION:0.1 "clear" C-m
tmux send-keys -t $SESSION:0.1 "echo -e '\033[1;36m[Starting Memory...]\033[0m'" C-m
tmux send-keys -t $SESSION:0.1 "ros2 launch sage_commander exploitation_ws.launch.py"

# Pane 2 — EXPLORER
tmux send-keys -t $SESSION:0.2 "docker exec -it ros2_explorer_container bash" C-m
tmux send-keys -t $SESSION:0.2 "clear" C-m
tmux send-keys -t $SESSION:0.2 "echo -e '\033[1;36m[Starting Explorer...]\033[0m'" C-m
tmux send-keys -t $SESSION:0.2 "ros2 launch sage_commander explorer_ws.launch.py"

# Pane 3 — DETECTION
tmux send-keys -t $SESSION:0.3 "docker exec -it ros2_detection_container bash" C-m
tmux send-keys -t $SESSION:0.3 "clear" C-m
tmux send-keys -t $SESSION:0.3 "echo -e '\033[1;36m[Starting Detection...]\033[0m'" C-m
tmux send-keys -t $SESSION:0.3 "ros2 launch sage_commander detection_ws.launch.py"

###########################################################
# Window 1: Development
###########################################################

tmux new-window -t $SESSION -n development

# Split window into four equal panes (2x2 grid)
tmux split-window -h -t $SESSION:development.0      # Split into left/right
tmux split-window -v -t $SESSION:development.0      # Split left side vertically
tmux split-window -v -t $SESSION:development.1      # Split right side vertically

# Pane 0 (isaac window) — enter isaac_sim_container
tmux send-keys -t $SESSION:development.0 "docker exec -it ros2_explorer_container bash" C-m
tmux send-keys -t $SESSION:development.0 "clear" C-m
tmux send-keys -t $SESSION:development.0 "echo -e '\033[1;36m[Starting Behaviour Tree...]\033[0m'" C-m
tmux send-keys -t $SESSION:development.0 "ros2 launch sage_behaviour_tree sage_bt_action_server.launch.py"

tmux send-keys -t $SESSION:development.1 "docker exec -it ros2_exploitation_container bash" C-m
tmux send-keys -t $SESSION:development.1 "clear" C-m
tmux send-keys -t $SESSION:development.1 "echo -e '\033[1;36m[Starting Evaluator...]\033[0m'" C-m
tmux send-keys -t $SESSION:development.1 "ros2 launch sage_evaluator evaluate.launch.py"
tmux send-keys -t $SESSION:development.2 "nvtop" C-m
tmux send-keys -t $SESSION:development.3 "htop" C-m

tmux select-layout -t $SESSION:development tiled

###########################################################
# Window 2: isaac
###########################################################

tmux new-window -t $SESSION -n isaac
tmux split-window -h -t $SESSION:isaac.0
tmux split-window -v -t $SESSION:isaac.1

# Pane 0 (isaac window) — enter isaac_sim_container
tmux send-keys -t $SESSION:isaac.0 "docker exec -it isaac_sim_container bash" C-m
tmux send-keys -t $SESSION:isaac.0 "./runheadless.sh"

# Pane 1 (isaac window) — local webrtc client
tmux send-keys -t $SESSION:isaac.1 "cd /home/kevin/Documents/IsaacOmniverse && ./isaacsim-webrtc-streaming-client-1.0.6-linux-x64.AppImage" C-m

tmux send-keys -t $SESSION:isaac.2 "cd /home/kevin/Groot2/bin && ./groot2" C-m

###########################################################
# Attach to session (focus Window 0, Pane 0)
###########################################################
tmux select-window -t $SESSION:main
tmux select-pane -t $SESSION:main.0
tmux attach -t $SESSION
