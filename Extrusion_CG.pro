TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    edit.cpp \
    extrusion.cpp \
    point.cpp \
    edge.cpp \
    render.cpp \
    plandata.cpp

LIBS += -lgmp -lmpfr



mac: LIBS += -framework GLUT
mac: LIBS += -framework OpenGL


macx: LIBS += -L$$PWD/../../../../../opt/local/lib/ -lCGAL

INCLUDEPATH += $$PWD/../../../../../opt/local/include
DEPENDPATH += $$PWD/../../../../../opt/local/include

HEADERS += \
    edit.h \
    extrusion.h \
    point.h \
    edge.h \
    main.h \
    plandata.h




win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../Library/Frameworks/release/ -lGLUI
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../Library/Frameworks/debug/ -lGLUI
else:mac: LIBS += -F$$PWD/../../../../../Library/Frameworks/ -framework GLUI
else:unix: LIBS += -L$$PWD/../../../../../Library/Frameworks/ -lGLUI

INCLUDEPATH += $$PWD/../../../../../Library/Frameworks/GLUI.framework/Versions/A/Headers
DEPENDPATH += $$PWD/../../../../../Library/Frameworks/GLUI.framework/Versions/A/Headers
