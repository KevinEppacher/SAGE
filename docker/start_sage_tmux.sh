#!/bin/bash

SESSION=sage

echo "Starting new tmux session '${SESSION}'..."

# Kill existing session if it exists to ensure a fresh start
tmux kill-session -t $SESSION 2>/dev/null

###########################################################
# explorer window
###########################################################
tmux new-session -d -s $SESSION -n explorer

tmux split-window -h -t $SESSION:explorer.0
tmux split-window -v -t $SESSION:explorer.0
tmux split-window -v -t $SESSION:explorer.1

tmux select-layout -t $SESSION:explorer tiled

tmux send-keys -t $SESSION:explorer.0 "docker exec -it -u root -w /app ros2_lavis_container bash --init-file /app/.initrc" C-m
tmux send-keys -t $SESSION:explorer.1 "docker exec -it -u root -w /app ros2_detection_container bash --init-file /app/.initrc" C-m
tmux send-keys -t $SESSION:explorer.2 "docker exec -it -u root -w /app ros2_explorer_container bash --init-file /app/.initrc" C-m
tmux send-keys -t $SESSION:explorer.3 "docker exec -it -u root -w /app ros2_detection_container bash --init-file /app/.initrc" C-m

###########################################################
# exploitation window
###########################################################
tmux new-window -t $SESSION -n exploitation

tmux split-window -h -t $SESSION:exploitation.0
tmux split-window -v -t $SESSION:exploitation.0
tmux split-window -v -t $SESSION:exploitation.1

tmux select-layout -t $SESSION:exploitation tiled

tmux send-keys -t $SESSION:exploitation.0 "docker exec -it -u root -w /app ros2_exploitation_container bash --init-file /app/.initrc" C-m
tmux send-keys -t $SESSION:exploitation.1 "docker exec -it -u root -w /app ros2_exploitation_container bash --init-file /app/.initrc" C-m
tmux send-keys -t $SESSION:exploitation.2 "docker exec -it -u root -w /app ros2_exploitation_container bash --init-file /app/.initrc" C-m
tmux send-keys -t $SESSION:exploitation.3 "docker exec -it -u root -w /app ros2_exploitation_container bash --init-file /app/.initrc" C-m

###########################################################
# detection window
###########################################################
tmux new-window -t $SESSION -n detection

tmux split-window -h -t $SESSION:detection.0
tmux split-window -v -t $SESSION:detection.0
tmux split-window -v -t $SESSION:detection.1

tmux select-layout -t $SESSION:detection tiled

tmux send-keys -t $SESSION:detection.0 "docker exec -it -u root -w /app ros2_detection_container bash --init-file /app/.initrc" C-m
tmux send-keys -t $SESSION:detection.1 "docker exec -it -u root -w /app ros2_detection_container bash --init-file /app/.initrc" C-m
tmux send-keys -t $SESSION:detection.2 "docker exec -it -u root -w /app ros2_detection_container bash --init-file /app/.initrc" C-m
tmux send-keys -t $SESSION:detection.3 "docker exec -it -u root -w /app ros2_detection_container bash --init-file /app/.initrc" C-m

###########################################################
# lavis window
###########################################################
tmux new-window -t $SESSION -n lavis

tmux split-window -h -t $SESSION:lavis.0
tmux split-window -v -t $SESSION:lavis.0
tmux split-window -v -t $SESSION:lavis.1

tmux select-layout -t $SESSION:lavis tiled

tmux send-keys -t $SESSION:lavis.0 "docker exec -it -u root -w /app ros2_lavis_container bash --init-file /app/.initrc" C-m
tmux send-keys -t $SESSION:lavis.1 "docker exec -it -u root -w /app ros2_lavis_container bash --init-file /app/.initrc" C-m
tmux send-keys -t $SESSION:lavis.2 "docker exec -it -u root -w /app ros2_lavis_container bash --init-file /app/.initrc" C-m
tmux send-keys -t $SESSION:lavis.3 "docker exec -it -u root -w /app ros2_lavis_container bash --init-file /app/.initrc" C-m

###########################################################
# isaac-lab window
###########################################################
tmux new-window -t $SESSION -n isaac-lab-ros2

tmux select-layout -t $SESSION:isaac-lab-ros2 tiled

tmux send-keys -t $SESSION:isaac-lab-ros2.0 \
"docker exec -it isaac-lab-ros2 bash -c 'export PS1=\"\\[\\e[01;32m\\]\\u@\\h:\\[\\e[01;34m\\]\\w\\[\\e[00m\\]\\\\$ \"; exec bash --norc'" C-m


###########################################################
# monitoring window
###########################################################
tmux new-window -t $SESSION -n monitoring
tmux rename-window -t $SESSION:monitoring "MONITORING"

# Pane 0: CPU/RAM mit htop
tmux send-keys -t $SESSION:monitoring.0 "htop" C-m

# Split horizontal und GPU-Anzeige starten
tmux split-window -h -t $SESSION:monitoring.0
tmux send-keys -t $SESSION:monitoring.1 "watch -n 1 nvidia-smi" C-m

tmux select-layout -t $SESSION:monitoring tiled

###########################################################
# Focus the first window and attach the session
###########################################################
tmux select-window -t $SESSION:explorer
tmux attach -t $SESSION
