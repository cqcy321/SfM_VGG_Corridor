QT += core
QT -= gui

TARGET = SfM
CONFIG += console ldconfig opencv
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    video_line_matching.cpp \
    point_matching.cpp \
    sift_matching.cpp \
    sfm.cpp

INCLUDEPATH += /usr/local/include/opencv \
/usr/local/include/opencv2 \
/usr/local/include \
/usr/include/eigen3/ \
/usr/local/cuda-8.0/include \
/home/cy/Documents/sw/opencv/opencv_contrib-3.1.0/modules/sfm/include/opencv2/sfm

LIBS += -L/usr/local/lib -lopencv_cudabgsegm -lopencv_cudaobjdetect -lopencv_cudastereo -lopencv_stitching -lopencv_cudafeatures2d -lopencv_superres -lopencv_cudacodec -lopencv_videostab -lopencv_cudaoptflow -lopencv_cudalegacy -lopencv_cudawarping -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_cvv -lopencv_datasets -lopencv_dnn -lopencv_dpm -lopencv_face -lopencv_fuzzy -lopencv_hdf -lopencv_line_descriptor -lopencv_optflow -lopencv_plot -lopencv_reg -lopencv_saliency -lopencv_sfm -lopencv_stereo -lopencv_structured_light -lopencv_rgbd -lopencv_viz -lopencv_surface_matching -lopencv_text -lopencv_xfeatures2d -lopencv_shape -lopencv_video -lopencv_ximgproc -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_xobjdetect -lopencv_objdetect -lopencv_ml -lopencv_xphoto -lopencv_highgui -lopencv_videoio -lopencv_photo -lopencv_imgcodecs -lopencv_cudaimgproc -lopencv_cudafilters -lopencv_imgproc -lopencv_cudaarithm -lopencv_core -lopencv_cudev

HEADERS += \
    def.hpp
