TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    hdmap/hdmap_util.cpp \
    pnc_map/pnc_map.cpp \
    hdmap.cpp \
    json/json_writer.cpp \
    json/json_valueiterator.inl \
    json/json_value.cpp \
    json/json_reader.cpp \
    json/json_internalmap.inl \
    json/json_internalarray.inl \
    map_interface.cpp \
    config/hdmap_config_param_initial.cpp \
    hdmap/hdmap_common.cpp

HEADERS += \
    hdmap/hdmap_util.h \
    pnc_map/pnc_map.h \
    hdmap.h \
    map_struct.h \
    json/writer.h \
    json/value.h \
    json/reader.h \
    json/json_batchallocator.h \
    json/json.h \
    json/forwards.h \
    json/features.h \
    json/config.h \
    json/autolink.h \
    map_interface.h \
    config/hdmap_config_param_initial.h \
    MSFLPublish.h \
    hdmap/hdmap_common.h

DISTFILES += \
    json/sconscript
