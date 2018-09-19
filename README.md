# Udacity Self-Driving Car Engineer Capstone Project
*E.N.*
---

This project integrates path following with traffic light detection to produce a self-driving car system that follows a pre-defined trajectory and obeys traffic light rules.

The path following mechanism reads the nearest set of n-waypoints, and applies a tuned PID to control steering, thus maintaining the vehicle along the path.

Traffic-light detection is done using HSV color space, where red, green, and yellow lights are detected via custom bandpass filtering.

Once traffic lights are detected, and their state is determined through color classification, the waypoint trajectory speed is updated to either maintain vehicle speed, or gradually reduce the speed to a stopping point in front of the light.

During testing it was noted that a correct ROS refresh frequency is required to avoid lagging and instability issues.
