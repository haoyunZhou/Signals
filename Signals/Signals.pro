TEMPLATE = app
CONFIG += console
CONFIG += c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    models/fspl.cpp \
    models/ericsson.cpp \
    models/ecc33.cpp \
    models/cost.cpp \
    models/hata.cpp \
    models/itm.cpp \
    models/itwom3.0.cpp \
    models/los.cpp \
    models/pel.cpp \
    models/sui.cpp \
    outputs.cpp \
    inputs.cpp \
    main.cpp \
    Signals.cpp

OTHER_FILES += \
    test.sh \
    signalserverLIDAR \
    signalserverHD \
    Signals.pro.user \
    Signals.pro \
    README.md \
    Makefile \
    LICENSE \
    CHANGELOG \
    .gitignore \
    models/README

HEADERS += \
    outputs.h \
    inputs.h \
    common.h \
    models/sui.h \
    models/pel.h \
    models/los.h \
    models/itwom3.0.h \
    models/hata.h \
    models/fspl.h \
    models/ericsson.h \
    models/ecc33.h \
    models/cost.h \
    Signals.h

