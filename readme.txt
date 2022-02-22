This code aim for 'how to integrate depth camera(D435i), Tracking camera(T265) and autopilot(pixhwak1).

* Overview
Depth camera: intelRealSense D435i
Tracking camera: intelRealSense T265
Autopilot: pixhawk1, Firmware: Hover 4.1.2
Computer process board: Intel Atom X7-E3950, Operation system: Ubuntu 18.04 LTS Bionic Beaver
code language: Python 3.9.1, used libraries: NumPy, Pyrealsense2, OpenCV, Time, NumPy, matplotlib, PLY, JSON, Pandas 
protocol: SSH


* Saved data
     1. Numeric Data (JSON file)
          Vehicle information: position coordinates, velocity, acceleration, heading
          Object information: object center coordinates, object edge coordinates
          Waypoint coordinates, Time 

     2. Images(PNG file)
          Depth camera images with detected objects, Point cloud images, 
          Tracking camera images

     3. SLAM Data (PLY file)
          Include SLAM and point cloud data
          * Load using Program. ( such as OctoMap, RealSense-viewer, etc.)
          
     4. Option (ROS. bag file)
          It takes 10 GB/min 


* Experimental Plan
Calculate the distance from the current position to the target waypoint at the end of the route with/without objects.

4 Independent variables, with 2 levels, a total of 16 factors
     Leg length: 3 or 5 meters, 
     Circuits: One or Triple repetitions of the path,
     Route shape: Triangle or Box, 
     Objects: with or without

Area: conducted in the same area, a building lobby, to remove a possible confounding variable associated with varying frictional coefficients of the floor
Tests conducted in Bane auditorium lobby


capywright by Hongseok Kim
