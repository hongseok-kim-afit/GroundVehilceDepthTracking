This code aim for 'how to integrate depth camera(D435i), Tracking camera(T265) and autopilot(pixhwak1).

* this code wrote with Python 3.6.9, and without ROS
     Saved data
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

