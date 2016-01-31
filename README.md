# calibDuo

A stereo calibration utilty for the Duo cameras ( [Duo3D.com] (https://duo3d.com) ).

#### To Use:
1. Install OpenCV 3.x and the DuoSDK
2. Set your OPENCV_ROOT and DUO_ROOT environment variables appropriately
3. Adjust .pro as needed
4. Run qmake
5. Build
6. Print resources/DuoCalibrationGrid.png
7. Attach the printed chessboard to a rigid surface
8. Plug-in your Duo camera
9. Run the application
 * Press any key to capture an image set
 * Press _ESC_ to end capture and perform stereo calibration
 * Visualize the calibration results
 * Press _ESC_ to terminate the program
10. Retreive the .yml calibration files from cameraFiles/

**Note:** For best results capture ~20+ image sets and ensure these fully cover the cameras' FOVs.
