#!/bin/sh

# modified from https://unix.stackexchange.com/questions/80473/how-do-i-run-a-shell-command-from-tmux-conf


tmux has-session -t tmux_run
if [ $? != 0 ]; then
    # create a new session
    tmux new-session -s tmux_run -n tmux_run -d
    
    # use mouse in Ubuntu /tmux 2.1
    tmux set -g mouse on

    # Highlight active window
    tmux set-window-option -g window-status-current-bg green

    # history limit
    tmux set -g history-limit 10000

    # Set status bar
    tmux set -g status-bg black
    tmux set -g status-fg white 

    tmux setw -g mode-keys vi
    tmux bind-key -t vi-copy 'v' begin-selection
    tmux bind-key -t vi-copy 'y' copy-pipe "xclip -sel clip -i"
    

    ## MAIN WINDOW

    tmux send-keys -t tmux_run 'roscore' C-m
    tmux select-layout tiled

    tmux split-window -v -t tmux_run
    tmux send-keys -t tmux_run 'sleep 2; roslaunch me5413_world world.launch' C-m
    tmux select-layout tiled
    
    tmux split-window -h -t tmux_run
    tmux send-keys -t tmux_run 'sleep 4; roslaunch me5413_world manual.launch' C-m
    tmux select-layout tiled

    tmux split-window -h -t tmux_run
    tmux send-keys -t tmux_run 'sleep 6; roslaunch hdl_localization hdl_localization.launch ' C-m
    #Gazebo Ground Truth
    # tmux send-keys -t tmux_run 'sleep 6; roslaunch gazebo_localization ground_robot_localization.launch ' C-m
    tmux select-layout tiled

    tmux split-window -h -t tmux_run
    tmux send-keys -t tmux_run 'sleep 8; rosrun rqt_robot_steering rqt_robot_steering '
    tmux select-layout tiled

    tmux split-window -h -t tmux_run
    tmux send-keys -t tmux_run 'roskill; gazebokill'
    tmux select-layout tiled

    tmux split-window -h -t tmux_run
    tmux send-keys -t tmux_run 'tmux kill-session'
    tmux select-layout tiled


fi
tmux attach -t tmux_run

