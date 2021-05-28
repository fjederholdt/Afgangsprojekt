QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    analyse.cpp \
    dhparams.cpp \
    fsclass.cpp \
    kalibrering.cpp \
    main.cpp \
    mainwindow.cpp \
    nykalibrering.cpp

HEADERS += \
    analyse.h \
    dhparams.h \
    fsclass.h \
    kalibrering.h \
    mainwindow.h \
    nykalibrering.h

FORMS += \
    analyse.ui \
    kalibrering.ui \
    mainwindow.ui \
    nykalibrering.ui

TRANSLATIONS += \
    KameraKalibrering_da_DK.ts
CONFIG += lrelease
CONFIG += embed_translations

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

INCLUDEPATH += -I/usr/local/include/opencv2 opt/pylon/include -I/usr/include/pcl-1.0
LIBS += -L/usr/local/lib -L/opt/pylon/lib -L/usr/lib/x86_64-linux-gnu \
-lpylonbase \
-lpcl_common \
-lpcl_visualization \
-lvtksys-7.1 \
-lvtkRenderingCore-7.1 \
-lvtkCommonDataModel-7.1 \
-lvtkCommonMath-7.1 \
-lvtkCommonCore-7.1 \
-lGenApi_gcc_v3_1_Basler_pylon \
-lGCBase_gcc_v3_1_Basler_pylon \
-lLog_gcc_v3_1_Basler_pylon \
-lMathParser_gcc_v3_1_Basler_pylon \
-lXmlParser_gcc_v3_1_Basler_pylon \
-lNodeMapData_gcc_v3_1_Basler_pylon \
-lboost_filesystem \
-lopencv_core \
-lopencv_aruco \
-lopencv_highgui \
-lopencv_video \
-lopencv_ml \
-lopencv_features2d \
-lopencv_calib3d \
-lopencv_objdetect \
-lopencv_videoio \
-lopencv_imgcodecs \
-lopencv_imgproc \
-lrtde \
-lpthread

unix:!macx: LIBS += -L$$PWD/../../../../../../opt/pylon/lib/ -lpylonbase \
-lpylonutility \
-lGenApi_gcc_v3_1_Basler_pylon \
-lGCBase_gcc_v3_1_Basler_pylon \
-lLog_gcc_v3_1_Basler_pylon \
-lMathParser_gcc_v3_1_Basler_pylon \
-lXmlParser_gcc_v3_1_Basler_pylon \
-lNodeMapData_gcc_v3_1_Basler_pylon \

INCLUDEPATH += $$PWD/../../../../../../opt/pylon/include
DEPENDPATH += $$PWD/../../../../../../opt/pylon/include

unix:!macx: LIBS += -L$$PWD/../../../../../../usr/lib/x86_64-linux-gnu/ -lpcl_common \
-lpcl_visualization \

INCLUDEPATH += $$PWD/../../../../../../usr/include/pcl-1.10
DEPENDPATH += $$PWD/../../../../../../usr/include/pcl-1.10

unix:!macx: LIBS += -L$$PWD/../../../../../../usr/lib/x86_64-linux-gnu/ -lvtksys-7.1 \
-lvtkRenderingCore-7.1 \
-lvtkCommonDataModel-7.1 \
-lvtkCommonMath-7.1 \
-lvtkCommonCore-7.1 \

INCLUDEPATH += $$PWD/../../../../../../usr/include/vtk-7.1 $$PWD/../../../../../../usr/include/eigen3
DEPENDPATH += $$PWD/../../../../../../usr/include/vtk-7.1 $$PWD/../../../../../../usr/include/eigen3
