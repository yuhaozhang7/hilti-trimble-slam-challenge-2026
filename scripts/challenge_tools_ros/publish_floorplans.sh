#!/bin/bash
# Script to publish all the floor plans stacked on top of one another
# This is only for demonstration purposes
#
# Run this script in the same directory as the floor plan images
# the last number is the height offset

ros2 run challenge_tools_ros map_server.py BuchsIT_EG_mask_windows.png 0

ros2 run challenge_tools_ros map_server.py BuchsIT_1OG_mask_windows.png 10
ros2 run challenge_tools_ros map_server.py BuchsIT_2OG_mask_windows.png 20
ros2 run challenge_tools_ros map_server.py BuchsIT_3OG_mask_windows.png 30
ros2 run challenge_tools_ros map_server.py BuchsIT_4OG_mask_windows.png 40

ros2 run challenge_tools_ros map_server.py BuchsIT_5OG_mask_windows.png 50
ros2 run challenge_tools_ros map_server.py BuchsIT_6OG_mask_windows.png 60
ros2 run challenge_tools_ros map_server.py BuchsIT_7OG_mask_windows.png 70

# special logic added in code to offset this image to -10 metres and to translate it to the same x,y frame
ros2 run challenge_tools_ros map_server.py BuchsIT_1UG_mask_windows.png 80
