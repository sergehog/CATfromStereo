# CATfromStereo
Real-time oriented CAT (Computed-Aided Teleoperation) system for robot with stereo-camera rigidly mounted on the last joint.

System tracks position of an object, which model is pre-loaded as a STL file. Sparse descriptive 3D points of an object are also uploaded. 
Then, for each frame 
- Simple stereo-matching is performed and some sparse reliable 3D points are extracted from the depth map.
- Points are further filtered and aligned using ICP method 
- Using pre-calibrated HandEye Transform as well as point of interest, required robot movement is re-calculated

*ITERStereoEstimator* Main Project, used for CAT. Implements stereo-matching with OpenGL, ICP, etc. 

*OpenGLStereoMatcher* simple plane-sweeping stereo-matcher based on OpenGL.
It was extracted from *ITERStereoEstimator* project and mainly used for debugging
Whole processing, except image acquisition is implemented in OpenGL. It uses Vimba API to work with cameras and actual setup works with Allied Vision Prosilica GigE Cameras.
