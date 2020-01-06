TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += /home/pi/my_samba/voxelsdk/Voxel
INCLUDEPATH += /home/pi/my_samba/voxelsdk/build/Voxel
INCLUDEPATH += /home/pi/my_samba/voxelsdk/TI3DToF

LIBS += -L/home/pi/my_samba/voxelsdk/build/lib \
-lvoxel -lvoxelpcl -lti3dtof

LIBS += -L/home/pi/my_samba/voxelsdk/build/lib/Voxel \


SOURCES += main.cpp
