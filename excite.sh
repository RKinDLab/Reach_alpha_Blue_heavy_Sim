#!/bin/bash

function cleanup { 
    echo "Cleaning up before exit..." 
    ros2 topic pub /forward_current_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0]}" --once
    exit 1 
} 

# Function to adjust position based on thresholds
adjust_position() {
    local position=$1
    local upper_threshold=$2
    local lower_threshold=$3

    if [ "$(echo "$position > $upper_threshold" | bc)" -eq 1 ]; then
        echo "negative"
        position=$(seq -300.0 -10.0 0.0 | shuf | head -n1)
    fi

    if [ "$(echo "$position < $lower_threshold" | bc)" -eq 1 ]; then
        echo "positive"
        position=$(seq 0 10 300.0 | shuf | head -n1)
    fi

    echo $position
}

trap cleanup SIGINT

ros2 topic pub /forward_current_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0]}" --once
sleep 5


# Define the filename
filename='excitations.txt'


endtime=$(date -ud "1 hours" +%s)
while [ $(date -u +%s) -le $endtime ]
do
    # Adjust position0
    position0=$(adjust_position "$position0" 0.0137 0.0004)

    # Adjust position1
    position1=$(adjust_position "$position1" 5.70 0.1)

    # Adjust position2
    position2=$(adjust_position "$position2" 3.400 0.1)

    # Adjust position3
    position3=$(adjust_position "$position3" 3.400 0.1)

    # Adjust position4
    position4=$(adjust_position "$position4" 6.00 0.1)

    echo $position0
    echo $position1
    echo $position2
    echo $position3
    echo $position4

    printf "%s %s %s %s %s\n" "$position0 $position1 $position2 $position3 $position4" >> $filename
    ros2 topic pub /forward_current_controller/commands std_msgs/msg/Float64MultiArray "{data: [$position0, $position1, $position2, $position3, $position4]}" --once
    sleep 4

done