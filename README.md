# CATfromStereo
Real-time oriented CAT (Computed-Aided Teleoperation) system for robot with stereo-camera rigidly mounted on the last joint.

System tracks position of an object, which model is pre-loaded as a STL file. Sparce descriptive 3D points of an object are also uploaded. 
Then, for each frame 
- Simple stereo-matching is performed and some sparse reliable 3D points are extracted from the depth map.
- Points are further filtered and aligned using ICP method 
- Using pre-calibrated HandEye Transform as well as point of interest, required robot movement is re-calculated
