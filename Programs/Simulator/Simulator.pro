#-------------------------------------------------
#
# Project created by QtCreator 2020-12-27T00:38:49
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Simulator
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

include(../../external/IPM_Shared/external/p3t/IPM_SCPI++/IPM_SCPI++.pri)
include(../../external/IPM_Shared/ModulesChain/Modules.pri)
include(../../external/IPM_Shared/external/p3t/IPM_Plot/IPM_Plot.pri)
include(../../external/IPM_Shared/external/p3t/PTS_BinaryIO/PTS_BinaryIO.pri)
include(../../external/IPM_Shared/external/p3t/PTS_Stream/IPM_TCPSocket.pri)
include(../../external/IPM_Shared/external/p3t/IPM_Parameter/IPM_Parameter.pri) # includes all for IPM Params

DEFINES += PT_USE_CONSOLE

VPATH    =  ../ \
            ../../external \
            ../../external/IPM_Shared/external/p3t \
            ../../external/IPM_Shared \
            /usr/include/eigen3 \
            ../../ \


INCLUDEPATH += $$VPATH
DEPENDPATH += $$INCLUDEPATH



SOURCES += \
        main.cpp \
        mainwindow.cpp \
    ../../Shared/enut.cpp \
    ../../external/IPM_Shared/external/p3t/IPM_SCPI/Remote/IPM_SCPI_Remote.cpp \
    ../../Shared/enut_simulator.cpp \
    ../../external/IPM_Shared/IPM_Calibration/ipm_ceres_geometry_fit.cpp \
    ../../Shared/enut_controller.cpp \
    ../../Shared/enut_models.cpp \
    ../../external/p3t/IPM_SCPI++/ipm_scpippserver/simple_server.cpp \
    ../../external/p3t/IPM_SCPI++/ipm_scpippserver/simple_server_connection.cpp \
    ../../external/p3t/IPM_SCPI++/ipm_scpippserver/simple_server_connection_manager.cpp \
    ../../external/p3t/IPM_SCPI++/ipm_scpipp_server2.cpp \

HEADERS += \
        mainwindow.h \
    ../../Shared/enut.h \
    ../../external/IPM_Shared/external/p3t/IPM_SCPI/Remote/IPM_SCPI_Remote.h \
    ../../Shared/enut_simulator.h \
    ../../Shared/enut_models.h \
    ../../external/IPM_Shared/IPM_Calibration/ipm_ceres_geometry_fit.h \
    ../../Shared/enut_ifaces.h \
    ../../Shared/enut_controller.h

SOURCES *= \
        PTS_BinaryIO/PTS_BinaryIO.cpp \
        PTS_Stream/IPM_Stream_Marker.cpp \
        PTS_Stream/IPM_Stream_Footer.cpp \

HEADERS *= \
        PTS_BinaryIO/PTS_BinaryIO.h \
        PTS_BinaryIO/PTS_BinConversion.h \
        PTS_Stream/IPM_Stream_Marker.h

unix {
    LIBS *= -lglog
    #LIBS *= -lgflags
    LIBS *= -lceres
    LIBS *= -fopenmp
    LIBS *= -lcholmod
    LIBS *= -lblas
    LIBS *= -llapack
    LIBS *= -lcxsparse
}

FORMS += \
        mainwindow.ui
