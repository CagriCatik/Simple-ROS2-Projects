#!/bin/bash

# Expert version of the RPLIDAR setup script

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
RULES_FILE="scripts/rplidar.rules"
UDEV_RULES_DIR="/etc/udev/rules.d/"
TARGET_RULE="$UDEV_RULES_DIR/rplidar.rules"

status_msg "Remapping the device serial port (ttyUSBX) to /dev/rplidar"
status_msg "RPLIDAR USB connection will be available as /dev/rplidar. You can check it using the command: ls -l /dev | grep ttyUSB"

# Change directory to rplidar_ros2 package
if ! colcon_cd rplidar_ros2; then
    error_msg "Failed to locate rplidar_ros2 package. Please ensure the package is correctly built."
    exit 1
fi

# Copy udev rules to /etc/udev/rules.d
status_msg "Copying $RULES_FILE to $UDEV_RULES_DIR"
if [[ -f $RULES_FILE ]]; then
    cp $RULES_FILE $TARGET_RULE
else
    error_msg "The rules file $RULES_FILE does not exist. Aborting."
    exit 1
fi

# Restart udev service
status_msg "Reloading and restarting udev service"
sudo service udev reload
sudo service udev restart

status_msg "RPLIDAR setup completed successfully. Verify the connection by running: ls -l /dev | grep rplidar"
