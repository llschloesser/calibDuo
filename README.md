# calibDuo

A stereo calibration utilty for Duo cameras ( [Duo3D.com] (https://duo3d.com) ).

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
   <img src="https://cloud.githubusercontent.com/assets/10792438/12801390/5f81660c-caaa-11e5-9979-55722a0b15bd.png" width="640" />
 * Press _ESC_ to end capture and perform stereo calibration
 * Visualize the calibration results
   <img src="https://cloud.githubusercontent.com/assets/10792438/12801389/5d8bc0d6-caaa-11e5-8518-56567a026268.png" width="640" />
   <img src= https://cloud.githubusercontent.com/assets/10792438/12801391/616d9062-caaa-11e5-9553-b205476de881.png width="320" />
 * Press _ESC_ to terminate the program
10. Retreive the .yml calibration files from cameraFiles/

**Note:** For best results capture ~20+ image sets and ensure these fully cover the cameras' FOVs. Closer is better.


