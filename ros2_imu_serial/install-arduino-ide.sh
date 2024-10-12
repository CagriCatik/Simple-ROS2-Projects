#!/bin/bash

# Update and upgrade the system
echo "Updating and upgrading system..."
sudo apt update && sudo apt upgrade -y

# Install dependencies
echo "Installing dependencies..."
sudo apt install -y wget

# Define the download URL for Arduino IDE
DOWNLOAD_URL="https://downloads.arduino.cc/arduino-ide/arduino-ide_2.3.3_Linux_64bit.AppImage?_gl=1*pjp51x*_ga*MjgzNTg2ODEyLjE3Mjg3NTg4Njk.*_ga_NEXN8H46L5*MTcyODc1ODg2OC4xLjAuMTcyODc1ODg2OS4wLjAuMTMxNjYwMzEyOA..*_fplc*aFlvZ1NEaHJ2UXM4JTJCNyUyQjRiem8lMkZ0JTJCYkRFdlhwQXAzZUVMYW5yMnJ6RGhPaTVpck1LUWlJVk0xanhjYVhDZGFyUjZUdlJ0ZSUyRllQWDBZSnRKeFFLRnFGJTJCaFNaJTJCVXhxUHBXQ3N4c0ZpRDE2N2tWJTJCN0ZVcTNLT0hadE5BNXA3ZyUzRCUzRA..*_gcl_au*MTg5NjA4MDEwMy4xNzI4NzU4ODcw"

# Download Arduino IDE AppImage
echo "Downloading Arduino IDE..."
wget "$DOWNLOAD_URL" -O arduino-ide.AppImage

# Make the AppImage executable
echo "Making the AppImage executable..."
chmod +x arduino-ide.AppImage

# Move the AppImage to /opt
echo "Moving Arduino IDE to /opt..."
sudo mv arduino-ide.AppImage /opt/arduino-ide.AppImage

# Create a symbolic link to make Arduino IDE accessible from anywhere
echo "Creating symbolic link..."
sudo ln -s /opt/arduino-ide.AppImage /usr/local/bin/arduino-ide

# Add user to dialout group for permission to access serial ports
echo "Adding current user to dialout group for serial port access..."
sudo usermod -aG dialout $USER

# Print completion message
echo "Installation complete! You can launch Arduino IDE by typing 'arduino-ide'."
