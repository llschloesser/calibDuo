#
# calibDuo.pro
#

CONFIG += warn_off c++11 console

TARGET   = calibDuo
TEMPLATE = app

HEADERS += \
    src/DuoCalibrator.h   \
    src/DuoUtility.h      \

INCLUDEPATH += src/

SOURCES += \
    src/calibDuo.cpp      \
    src/DuoCalibrator.cpp \

#
# OpenCV 3+
#
message( OPENCV_ROOT is $$(OPENCV_ROOT) )
INCLUDEPATH += $$(OPENCV_ROOT)/include
LIBS += -L$$(OPENCV_ROOT)/lib
LIBS += \
        -lopencv_calib3d     \
        -lopencv_core        \
        -lopencv_features2d  \
        -lopencv_highgui     \
        -lopencv_imgproc     \
        -lopencv_imgcodecs   \

#
# List of all available libraries:
#
# -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired
# -lopencv_calib3d -lopencv_ccalib -lopencv_core -lopencv_cudaarithm
# -lopencv_cudabgsegm -lopencv_cudafeatures2d -lopencv_cudafilters
# -lopencv_cudaimproc -lopencv_cudalegacy -lopencv_cudaobjdetect
# -lopencv_cudaoptflow -lopencv_cudastereo -lopencv_cudawarping
# -lopencv_cudev -lopencv_cvv -lopencv_datasets -lopencv_dnn -lopencv_dpm
# -lopencv_face -lopencv_features2d -lopencv_flann -lopencv_fuzzy
# -lopencv_hdf -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc
# -lopencv_line_descriptor -lopencv_ml -lopencv_objdetect -lopencv_optflow
# -lopencv_photo -lopencv_plot -lopencv_reg -lopencv_rgbd -lopencv_saliency
# -lopencv_shape -lopencv_stereo -lopencv_stitching -lopencv_structured_light
# -lopencv_superres -lopencv_surface_matching -lopencv_text -lopencv_tracking
# -lopencv_video -lopencv_videoio -lopencv_videostab -lopencv_viz
# -lopencv_xfeatures2d -lopencv_ximgproc -lopencv_xobjdetect -lopencv_xphoto


#
# DUO 3D SDK
#
message( DUO_ROOT is $$(DUO_ROOT) )
INCLUDEPATH += $$(DUO_ROOT)/include
LIBS += -L$$(DUO_ROOT)/osx/x64
LIBS += -lDUO

