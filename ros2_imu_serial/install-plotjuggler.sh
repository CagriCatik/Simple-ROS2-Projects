#!/bin/bash

# Check if PlotJuggler is installed
if ! command -v plotjuggler &> /dev/null
then
    echo "PlotJuggler is not installed. Please install it first."
    exit
fi

# Optional: Define a path to a configuration file or data file
CONFIG_FILE="path_to_your_config.xml"
DATA_FILE="path_to_your_data.bag" # Replace with your actual data file, e.g., ROS bag file

# Check if a configuration file exists
if [ -f "$CONFIG_FILE" ]; then
    echo "Launching PlotJuggler with configuration file: $CONFIG_FILE"
    plotjuggler -c "$CONFIG_FILE"
elif [ -f "$DATA_FILE" ]; then
    echo "Launching PlotJuggler with data file: $DATA_FILE"
    plotjuggler -l "$DATA_FILE"
else
    echo "No configuration or data file found. Launching default PlotJuggler."
    plotjuggler
fi
