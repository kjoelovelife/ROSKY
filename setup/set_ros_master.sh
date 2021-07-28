#!/usr/bin/env bash
current_ip=$(hostname -I | awk '{print $1}')
hardware=$(uname -r)
echo "======== Setup ROS ========"
echo "Setting ROS_MASTER_URI..."
if [ $# -gt 0 ]; then
    # provided a hostname, use it as ROS_MASTER_URI
    export ROS_MASTER_URI=http://$1.local:11311/
else
    echo "No hostname provided. Using $HOSTNAME."
    export ROS_MASTER_URI=http://$HOSTNAME.local:11311/
fi

echo "ROS_MASTER_URI set to $ROS_MASTER_URI"
echo ""

# set ROS_HOSTNAME
echo "Setting ROS_HOSTNAME..."
export ROS_HOSTNAME=$current_ip
echo "ROS_HOSTNAME set to $current_ip"

echo "==== End ===="
