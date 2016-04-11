#ifndef DUO_UTILITY_H
#define DUO_UTILITY_H

#include <condition_variable>
#include <cstdlib>
#include <mutex>
#include <stdio.h>

#include "DUOLib.h"

const int32_t FPS         =  30;
const int32_t WIDTH_FULL  = 752;
const int32_t HEIGHT_FULL = 480;
const int32_t WIDTH_VGA   = 640;
const int32_t HEIGHT_VGA  = 480;
const int32_t WIDTH_QVGA  = 320;
const int32_t HEIGHT_QVGA = 240;


static DUOInstance _duo        = nullptr;
static PDUOFrame   _pFrameData = nullptr;

static std::mutex              _frameMutex;
static std::condition_variable _frameCV;

static bool _ready = false;

//
// One and only duo callback function
// It sets the current frame data and signals that the new frame data is ready
//
static void CALLBACK DUOCallback( const PDUOFrame pFrameData, void* pUserData )
{
  std::unique_lock<std::mutex> lk( _frameMutex );
  _pFrameData = pFrameData;
  _ready = true;
  _frameCV.notify_one();
}

//
// Opens, sets current image format and fps and starts capturing
//
static bool OpenDUOCamera( const int width, const int height, const float fps )
{
  if( _duo != nullptr )
  {
    StopDUO( _duo );
    CloseDUO( _duo );
    _duo = nullptr;
  }

  //
  // Find optimal binning parameters for given (width, height)
  // This maximizes sensor imaging area for given resolution
  //
  int binning = DUO_BIN_NONE;
  if( width <= 752/2 )
    binning += DUO_BIN_HORIZONTAL2;
  if( height <= 480/4 )
    binning += DUO_BIN_VERTICAL4;
  else if( height <= 480/2 )
    binning += DUO_BIN_VERTICAL2;

  //
  // Check if we support given resolution (width, height, binning, fps)
  //
  DUOResolutionInfo ri;
  if( !EnumerateResolutions( &ri, 1, width, height, binning, fps ) )
    return 0;

  if( !OpenDUO( &_duo ) )
    return 0;

  char tmp[260];
  // Get and print some DUO parameter values
  GetDUODeviceName( _duo,tmp );
  printf( "DUO Device Name:      '%s'\n", tmp );

  GetDUOSerialNumber( _duo, tmp );
  printf( "DUO Serial Number:    %s\n", tmp );

  GetDUOFirmwareVersion( _duo, tmp );
  printf( "DUO Firmware Version: v%s\n", tmp );

  GetDUOFirmwareBuild( _duo, tmp );
  printf( "DUO Firmware Build:   %s\n", tmp );

  SetDUOResolutionInfo( _duo, ri );

  if( !StartDUO( _duo, DUOCallback, nullptr ) )
    return false;

  SetDUOCameraSwap( _duo, true );
  SetDUOHFlip( _duo, true );
  SetDUOVFlip( _duo, true );

  return true;
}

//
// Waits until the new DUO frame is ready and returns it
//
static PDUOFrame GetDUOFrame()
{
  if( _duo == nullptr ) return nullptr;

  std::unique_lock<std::mutex> lk( _frameMutex );
  _frameCV.wait( lk, [] { return _ready; } );
  _ready = false;

  return _pFrameData;
}

//
// Stop capture and close the camera
//
static void CloseDUOCamera()
{
  if( _duo == nullptr ) return;

  StopDUO( _duo );
  CloseDUO( _duo );
  _duo = nullptr;
}


static void SetExposure( const float value )
{
  if( _duo == nullptr ) return;
  SetDUOExposure( _duo, value );
}


static void SetGain( const float value )
{
  if( _duo == nullptr ) return;
  SetDUOGain( _duo, value );
}


static void SetLED( const float value )
{
  if( _duo == nullptr ) return;
  SetDUOLedPWM( _duo, value );
}


#endif // DUO_UTILITY_H
