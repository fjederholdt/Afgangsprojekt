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

INCLUDEPATH += -I/usr/local/include/opencv2
LIBS += -L/usr/local/lib -lboost_filesystem -lopencv_core -lopencv_aruco -lopencv_highgui -lopencv_video -lopencv_ml -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_videoio -lopencv_imgcodecs -lopencv_imgproc -lrtde

