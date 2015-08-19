#-------------------------------------------------
#
# Project created by QtCreator 2014-04-23T14:38:22
#
#-------------------------------------------------

QT       += core gui

TARGET = src
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    closest.cpp \
    iterativeclosestpoint.cpp \
    csvrow.cpp

HEADERS  += mainwindow.h \
    closest.h \
    iterativeclosestpoint.h \
    csvrow.h

FORMS    += mainwindow.ui

INCLUDEPATH +=   "/usr/include/eigen3/" \
                 "/usr/include/vtk-5.8/" \
                 "/usr/include/boost/" \
                 "/usr/include/pcl-1.7" \
                 "/usr/include/flann" \
                 "/usr/lib/"

LIBS += -L/usr/lib \
        -lQVTK \
        -lvtkRendering \
        -lvtkCommon \
        -lvtkFiltering \
        -lpcl_visualization \
        -lboost_system \
        -lboost_filesystem \
        -lpcl_filters \
        -lpcl_sample_consensus \
        -lpcl_registration \
        -lpcl_common \
        -lpcl_kdtree \
        -lpcl_io \
        -lpcl_search \


INCLUDEPATH += $$PWD/../../../../../usr/include
DEPENDPATH += $$PWD/../../../../../usr/include

RESOURCES += \
    resources.qrc
