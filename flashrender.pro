TARGET = flashrender
TEMPLATE = lib
CONFIG += shared
# completely turn off optimization for desktop version to avoid debug variable optimization
linux-g++ {
  QMAKE_CXXFLAGS += -O0
}

CONFIG += c++2a

ANDROID_ABIS=arm64-v8a

SOURCES += ../base/flashbase.cpp \
 ../base/flashclass.cpp \
 ../base/flashclassmanager.cpp \
 ../base/flashdatetime.cpp \
 ../base/flashimport.cpp \
 ../base/flashlocker.cpp \
 ../base/flashmap.cpp \
 ../base/flashobject.cpp \
 ../flashrender/flashrender.cpp

INCLUDEPATH += $$PWD/../base
INCLUDEPATH += $$PWD/../flashrender

HEADERS += ../base/flashbase.h \
 ../base/flashclass.h \
 ../base/flashclassmanager.h \
 ../base/flashdatetime.h \
 ../base/flashimport.h \
 ../base/flashlocker.h \
 ../base/flashmap.h \
 ../base/flashobject.h \
 ../base/flashserialize.h \
 ../flashrender/flashrender.h
