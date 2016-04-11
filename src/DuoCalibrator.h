#ifndef DUO_CALIBRATOR_H
#define DUO_CALIBRATOR_H

#include <opencv2/core.hpp>

#include "DuoUtility.h"


const cv::Size QVGA     = cv::Size( WIDTH_QVGA, HEIGHT_QVGA );
const cv::Size VGA      = cv::Size( WIDTH_VGA,  HEIGHT_VGA  );
const cv::Size DUO_FULL = cv::Size( WIDTH_FULL, HEIGHT_FULL );


class DuoCalibrator
{
public:

  DuoCalibrator( const cv::Size& boardSize )
    : m_squareLength( 2.533 )  // in cm, but this could be changed to m or mm
    , m_boardSize( boardSize ) // inner corners (this project's "chessboard" is 9x6)
    , m_imageSize( VGA )       // default resolution (could go as high as 752x480)
  {}

  void processFrame( const cv::Mat& left,
                     const cv::Mat& right,
                     std::vector<cv::Point2f>& leftPtsOut,
                     std::vector<cv::Point2f>& rightPtsOut );

  void keepMostRecent();

  size_t getNumImageSets() { return m_objectPts.size(); }

  void calibrate();

  const cv::Mat& undistortAndRectifyLeft( const cv::Mat& left ) const;

  const cv::Mat& undistortAndRectifyRight( const cv::Mat& right ) const;

  const cv::Mat& getDisparity( const cv::Mat& left, const cv::Mat& right ) const;

private:

  void detectChessboardPoints( const cv::Mat& left,
                               const cv::Mat& right,
                               std::vector<cv::Point2f>& leftPtsOut,
                               std::vector<cv::Point2f>& rightPtsOut );
  //
  // TODO: Implement this, should give better results
  //
  //void detectCircleGridPoints( const cv::Mat& left,
  //                             const cv::Mat& right,
  //                             std::vector<cv::Point2f>& leftPtsOut,
  //                             std::vector<cv::Point2f>& rightPtsOut );

private:

  const float                           m_squareLength;

  const cv::Size                        m_boardSize;
  const cv::Size                        m_imageSize;

  //
  // Object points in world coordinates
  //
  std::vector<std::vector<cv::Point3f>> m_objectPts;
  std::vector<cv::Point3f>              m_lastObjectPts;

  //
  // Image points in pixel coordinates
  //
  std::vector<std::vector<cv::Point2f>> m_imagePtsL;
  std::vector<std::vector<cv::Point2f>> m_imagePtsR;

  std::vector<cv::Point2f>              m_lastImagePtsL;
  std::vector<cv::Point2f>              m_lastImagePtsR;

  //
  // Camera Intrinsics
  //
  cv::Mat                               m_M1;
  cv::Mat                               m_D1;
  cv::Mat                               m_M2;
  cv::Mat                               m_D2;

  //
  // Camera Extrinsics
  //
  cv::Mat                               m_R;
  cv::Mat                               m_T;
  cv::Mat                               m_E;
  cv::Mat                               m_F;
  cv::Mat                               m_R1;
  cv::Mat                               m_R2;
  cv::Mat                               m_P1;
  cv::Mat                               m_P2;
  cv::Mat                               m_Q;

  //
  // Undistort Maps
  //
  cv::Mat                               m_mapL1;
  cv::Mat                               m_mapL2;
  cv::Mat                               m_mapR1;
  cv::Mat                               m_mapR2;
};

#endif // DUO_CALIBRATOR_H
