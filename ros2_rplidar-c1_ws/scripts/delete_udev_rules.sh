#!/bin/bash

# Expert version of the RPLIDAR udev rules removal script

# Function to print status messages
function status_msg {
    echo -e "\e[32m[INFO]\e[0m $1"
}

# Function to print error messages
function error_msg {
    echo -e "\e[31m[ERROR]\e[0m $1"
}

# Check if running as root
if [[ $EUID -ne 0 ]]; then
    error_msg "This script must be run as root. Please use sudo."
    exit 1
fi

# Define constants
UDEV_RULE="/etc/udev/rules.d/rplidar.rules"

status_msg "Deleting the remap rule for RPLIDAR device serial port (ttyUSBX)"
status_msg "Removing $UDEV_RULE"

# Check if the udev rule exists before attempting to delete it
if [[ -f $UDEV_RULE ]]; then
    sudo rm $UDEV_RULE
    status_msg "Successfully deleted $UDEV_RULE"
else
    error_msg "$UDEV_RULE does not exist. No action needed."
fi

# Restart udev service
status_msg "Reloading and restarting udev service"
sudo service udev reload
sudo service udev restart

status_msg "RPLIDAR udev rule deletion completed."
