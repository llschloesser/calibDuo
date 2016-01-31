#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "DuoCalibrator.h"
#include "DuoUtility.h"


static const cv::Scalar WHITE( 255, 255, 255 );
static const cv::Scalar BLUE ( 255,   0,   0 );
static const cv::Scalar GREEN(   0, 255,   0 );
static const cv::Scalar RED  (   0,   0, 255 );

static const std::string WINDOW_NAME = "Duo Calibration";


// Gain, Exposure, and LED valid 0-100
int       EXPOSURE     =  70;
const int EXPOSURE_MAX = 100;
int       GAIN         =   0;
const int GAIN_MAX     = 100;
int       LED          =  20;
const int LED_MAX      = 100;


static void callbackExposure( int, void* ) { SetExposure(EXPOSURE); }
static void callbackGain    ( int, void* ) { SetGain(GAIN);         }
static void callbackLED     ( int, void* ) { SetLED(LED);           }


static void createTrackbars()
{
  // Create trackbars and atttch them to a window
  // 3 parameters are:
  // - the address of the variable that is changing
  //   when the trackbar is moved(eg.H_LOW),
  // - the max value the trackbar can move (eg. H_HIGH), and
  // - the function that is called whenever the trackbar is
  //   moved(eg. on_trackbar_callback)
  cv::createTrackbar( "Exposure", WINDOW_NAME,
                      &EXPOSURE, EXPOSURE_MAX, callbackExposure );
  cv::createTrackbar( "Gain", WINDOW_NAME,
                      &GAIN, GAIN_MAX, callbackGain );
  cv::createTrackbar( "LED", WINDOW_NAME,
                      &LED, LED_MAX, callbackLED );
}


static void drawLines( cv::Mat& display )
{
  const int rows = display.rows;
  const int cols = display.cols;

  const int numLines = 24;

  const int step = rows/numLines;

  for( int i = 1; i < numLines; ++i )
  {
    cv::line(	display,
              cv::Point(    0, i*step ),
              cv::Point( cols, i*step ),
              GREEN );
  }
}


int main()
{
  printf( "DUOLib Version:       v%s\n", GetLibVersion() );

  // Open DUO camera and start capturing
  if( !OpenDUOCamera( WIDTH_VGA, HEIGHT_VGA, FPS ) )
  {
    printf( "Could not open DUO camera\n" );
    return 0;
  }

  // Set exposure and gain
  SetExposure( EXPOSURE );
  SetGain( GAIN );
  SetLED( LED );

  cv::namedWindow( WINDOW_NAME, CV_GUI_NORMAL | CV_WINDOW_NORMAL );

  createTrackbars();

  cv::Mat left  = cv::Mat( VGA, CV_8U );
  cv::Mat right = cv::Mat( VGA, CV_8U );

  cv::Mat display = cv::Mat::zeros( VGA.height, 2*VGA.width, CV_8UC3 );

  cv::Mat leftDisplay  = display.colRange( 0, VGA.width );
  cv::Mat rightDisplay = display.colRange( VGA.width, 2*VGA.width );

  const cv::Size boardSize( 9, 6 );

  DuoCalibrator calibDuo( boardSize );

  std::cout << "Press a key to begin taking calibration images.\n";

  bool isActive = true;

  while( isActive )
  {
    while( true )
    {
      //
      // Capture DUO frame
      //
      PDUOFrame pFrameData = GetDUOFrame();
      if( pFrameData == nullptr ) continue;

      //
      // Set the image data
      //
      left.data  = (uint8_t*) pFrameData->leftData;
      right.data = (uint8_t*) pFrameData->rightData;

      static std::vector<cv::Point2f> leftPts;
      static std::vector<cv::Point2f> rightPts;

      calibDuo.sampleFrame( left, right, leftPts, rightPts );

      cv::cvtColor( left,  leftDisplay,  cv::COLOR_GRAY2BGR );
      cv::cvtColor( right, rightDisplay, cv::COLOR_GRAY2BGR );

      const bool foundL = leftPts.size()  == boardSize.area();
      const bool foundR = rightPts.size() == boardSize.area();

      cv::drawChessboardCorners( leftDisplay,  boardSize, leftPts,  foundL );
      cv::drawChessboardCorners( rightDisplay, boardSize, rightPts, foundR );

      std::stringstream ss;
      ss << "Press any key to capture a frame, ESC to begin calibration";

      cv::putText( display,
                   ss.str(),
                   cv::Point( 10, 30 ),
                   cv::FONT_HERSHEY_SIMPLEX,
                   0.75,
                   WHITE );

      ss.str("");
      ss << "# Image Sets = " << calibDuo.getNumImageSets();

      cv::putText( display,
                   ss.str(),
                   cv::Point( 10, 60 ),
                   cv::FONT_HERSHEY_SIMPLEX,
                   0.75,
                   WHITE );

      cv::imshow( WINDOW_NAME, display );

      const int key = cv::waitKey(10);

      if( key >= 0 )
      {
        if( key == 27 )
        {
          // Quit the loop with ESC
          isActive = false;
        }

        // Any key press is a command to keep a calibration set
        calibDuo.keepMostRecent();

        break;
      }
    }
  }

  std::cout << "Finished taking calibration images.\n";

  calibDuo.calibrate();

  std::cout << "Stereo-calibration completed.\n";

  const std::string DISP_WINDOW_NAME( "Disparity" );

  cv::namedWindow( DISP_WINDOW_NAME, CV_GUI_NORMAL | CV_WINDOW_NORMAL );

  while( true )
  {
    //
    // Capture DUO frame
    //
    PDUOFrame pFrameData = GetDUOFrame();
    if( pFrameData == nullptr ) continue;

    //
    // Set the image data
    //
    left.data  = (uint8_t*) pFrameData->leftData;
    right.data = (uint8_t*) pFrameData->rightData;

    const cv::Mat& newLeft  = calibDuo.undistortAndRectifyLeft( left );
    const cv::Mat& newRight = calibDuo.undistortAndRectifyRight( right );

    cv::cvtColor( newLeft,  leftDisplay,  cv::COLOR_GRAY2BGR );
    cv::cvtColor( newRight, rightDisplay, cv::COLOR_GRAY2BGR );

    drawLines( display );

    std::stringstream ss;
    ss << "Press ESC to terminate";

    cv::putText( display,
                 ss.str(),
                 cv::Point( 10, 30 ),
                 cv::FONT_HERSHEY_SIMPLEX,
                 0.75,
                 WHITE );

    cv::imshow( WINDOW_NAME, display );

    const cv::Mat& disp = calibDuo.getDisparity( newLeft, newRight );

    cv::imshow( DISP_WINDOW_NAME, disp );

    if( cv::waitKey( 5 ) == 27 )
    {
      // Terminate the program with ESC
      break;
    }
  }

  return 0;
}
