#!/bin/bash

set -e

source "/opt/ros/noetic/setup.bash"
source "/catkin_ws/devel/setup.bash"

# Create a data directory to download a rosbag to and store results
datadir="/catkin_ws/src/halepensis/data"
if [ -d $datadir ]; then
    echo "Directory already exists" ;
else
    `mkdir -p $datadir`;
    echo "$datadir directory is created"
fi

# Download Example Bag
if [ -f "${datadir}/example.bag" ]; then
    echo "Rosbag already exists."
else
    echo "Downloading rosbag"
    wget https://owncloud.tuwien.ac.at/index.php/s/pRXtI0g4KSvyUqO/download -O ${datadir}/example.bag
fi


tmux new-session -d -s  "halepensis"
tmux splitw -h -p 50
tmux selectp -t 0

echo "Launchin Halepensis hsrb.launch"
tmux send-keys -t 0 "roslaunch halepensis_app1 hsrb.launch" Enter

sleep 2
echo "Starting the example rosbag"
tmux send-keys -t 1 "rosbag play ${datadir}/example.bag" Enter

echo "attaching to tmux session..."
tmux attach-session -t "halepensis"

exec "$@"
