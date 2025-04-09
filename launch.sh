#!/bin/bash

SESSION="jetson_ssh"

# Kill old session if it exists
tmux kill-session -t $SESSION 2>/dev/null

# Start new session (top-left pane)
tmux new-session -d -s $SESSION
sleep 0.2

# === TOP ROW ===

tmux split-window -h -t $SESSION:0.0  # top-middle
sleep 0.2

tmux split-window -h -t $SESSION:0.1  # top-right
sleep 0.2

# === BOTTOM ROW ===

tmux select-pane -t $SESSION:0.0
tmux split-window -v                  # bottom-left
sleep 0.2

tmux select-pane -t $SESSION:0.1
tmux split-window -v                  # bottom-middle
sleep 0.2

tmux select-pane -t $SESSION:0.2
tmux split-window -v                  # bottom-right
sleep 0.2

# Apply clean layout
tmux select-layout -t $SESSION tiled

# === 1. SSH into each pane ===
SSH_CMD="ssh -X jetson@10.42.0.201"

for i in {0..5}; do
    sleep 2
    tmux send-keys -t $SESSION:0.$i "$SSH_CMD" C-m
done

# Wait for SSH to settle
sleep 5

# === 2. cd into workspace ===
for i in {0..5}; do
    tmux send-keys -t $SESSION:0.$i "cd flyrs_ws" C-m
done

# Wait a bit before sourcing
sleep 1

# === 3. run src_bash ===
for i in {0..5}; do
    tmux send-keys -t $SESSION:0.$i "src_bash" C-m
done

# Wait for environment to be sourced
sleep 1

# === 4. Run specific commands in designated panes ===

# Top-left (pane 0): main
tmux send-keys -t $SESSION:0.0 "ros2 run cv_basics main" C-m

# Top-middle (pane 1): camera
tmux send-keys -t $SESSION:0.1 "ros2 run cv_basics camera" C-m

# Bottom-left (pane 3): mavros
tmux send-keys -t $SESSION:0.2 "launch_mavros" C-m

# Bottom-middle (pane 4): drone gui
tmux send-keys -t $SESSION:0.4 "ros2 run drone_gui gui" C-m

# Focus top-left pane
tmux select-pane -t $SESSION:0.0

# Attach to session
tmux attach -t $SESSION
