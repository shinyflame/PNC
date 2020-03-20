#-------------------------------------------------
#
# Project created by QtCreator 2018-12-10T15:41:52
#
#-------------------------------------------------

QT       += core gui printsupport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Planning
TEMPLATE = app
//TEMPLATE = lib

#INCLUDEPATH += /usr/local/lib
LIBS += /usr/local/lib/*.so
# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
    main.cpp \
    mainwindow.cpp \
    json/json_internalarray.inl \
    json/json_internalmap.inl \
    json/json_reader.cpp \
    json/json_value.cpp \
    json/json_valueiterator.inl \
    json/json_writer.cpp \
    lattice/trajectory1d/lattice_trajectory1d.cpp \
    lattice/trajectory_generation/trajectory1d_generator.cpp \
    lattice/trajectory_generation/trajectory_combiner.cpp \
    common/trajectory/discretized_trajectory.cpp \
    lattice/trajectory_generation/piecewise_braking_trajectory_generator.cpp \
    lattice/trajectory1d/piecewise_acceleration_trajectory1d.cpp \
    lattice/trajectory_generation/backup_trajectory_generator.cpp \
    lattice/trajectory_generation/trajectory_evaluator.cpp \
    constraint_checker/collision_checker.cpp \
    constraint_checker/constraint_checker.cpp \
    constraint_checker/constraint_checker1d.cpp \
    lattice/trajectory1d/constant_deceleration_trajectory1d.cpp \
    common/trajectory/trajectory_stitcher.cpp \
    common/trajectory/publishable_trajectory.cpp \
    common/ego_info.cpp \
    config/planning_config_param_initial.cpp \
    lattice/behavior/feasible_region.cpp \
    lattice/behavior/path_time_graph.cpp \
    lattice/behavior/prediction_querier.cpp \
    common/obstacle.cpp \
    common/reference_line_info.cpp \
    lattice/trajectory_generation/end_condition_sampler.cpp \
    common/frame.cpp \
    reference_line/reference_line.cpp \
    reference_line/reference_line_provider.cpp \
    #common/change_lane_decider.cpp \
    map/route_segment.cpp \
    common/lag_prediction.cpp \
    common/get_data.cpp \
    planner/lattice/lattice_planner.cpp \
    common/get_now_time.cpp \
    planning_base.cpp \
    std_planning.cpp \
    common/vehicle_state/vehicle_state_provider.cpp \
    planning.cpp \
    toolkits/deciders/traffic_decider.cpp \
    common/speed/st_boundary.cpp \
    common/speed/st_point.cpp \
    common/path_obstacle.cpp \
    common/path_decision.cpp \
    toolkits/deciders/utill.cpp \
    toolkits/deciders/signal_light.cpp \
    common/speed/speed_data.cpp \
    common/get_traffic_rule.cpp \
    common/g_test.cpp \
    math/curve1d/quartic_polynomial_curve1d.cpp \
    math/aabox2d.cpp \
    math/angle.cpp \
    math/box2d.cpp \
    math/cartesian_frenet_conversion.cpp \
    math/line_segment2d.cpp \
    math/linear_interpolation.cpp \
    math/math_utils.cpp \
    math/path_matcher.cpp \
    math/polygon2d.cpp \
    math/search.cpp \
    math/sin_table.cpp \
    math/util.cpp \
    math/vec2d.cpp \
    math/curve1d/quintic_polynomial_curve1d.cpp \
    common/frame_test.cpp \
    common/write_read_file.cpp \
    common/coordinate_convert.cpp \
    math/curve_math.cpp \
    qcustomplot.cpp \
    toolkits/deciders/stop_sign.cpp \
    common/garbage_deal.cpp \
    toolkits/deciders/destination.cpp \
    PlanPublish.cpp \
    ../map/hdmap.cpp \
    ../map/config/hdmap_config_param_initial.cpp \
    ../map/hdmap/hdmap_common.cpp \
    ../map/hdmap/hdmap_util.cpp \
    ../map/pnc_map/pnc_map.cpp \
    ../map/map_interface.cpp\
    ../map/common/pnc_information.cpp \
    ../map/common/pnc_util.cpp




HEADERS += \
    mainwindow.h \
    json/autolink.h \
    json/config.h \
    json/features.h \
    json/forwards.h \
    json/json.h \
    json/json_batchallocator.h \
    json/reader.h \
    json/value.h \
    json/writer.h \
    lattice/trajectory1d/lattice_trajectory1d.h \
    lattice/trajectory_generation/trajectory1d_generator.h \
    lattice/trajectory_generation/trajectory_combiner.h \
    common/trajectory/discretized_trajectory.h \
    lattice/trajectory_generation/piecewise_braking_trajectory_generator.h \
    lattice/trajectory1d/piecewise_acceleration_trajectory1d.h \
    lattice/trajectory_generation/trajectory_evaluator.h \
    lattice/trajectory_generation/backup_trajectory_generator.h \
    constraint_checker/collision_checker.h \
    constraint_checker/constraint_checker.h \
    constraint_checker/constraint_checker1d.h \
    lattice/trajectory1d/constant_deceleration_trajectory1d.h \
    common/trajectory/trajectory.h \
    common/trajectory/trajectory_stitcher.h \
    common/trajectory/publishable_trajectory.h \
    common/ego_info.h \
    common/util/factory.h \
    toolkits/deciders/deciders/traffic_rule.h \
    toolkits/deciders/traffic_decider.h \
    toolkits/deciders/traffic_rule.h \
    config/planning_config_param_initial.h \
    lattice/behavior/feasible_region.h \
    lattice/behavior/path_time_graph.h \
    lattice/behavior/prediction_querier.h \
    common/obstacle.h \
    common/reference_line_info.h \
    common/struct.h \
    lattice/trajectory_generation/end_condition_sampler.h \
    common/frame.h \
    reference_line/reference_line.h \
    reference_line/reference_line_provider.h \
    #common/change_lane_decider.h \
    map/route_segment.h \
    common/indexed_list.h \
    common/lag_prediction.h \
    common/get_data.h \
    planner/planner.h \
    common/util/factory.h \
    common/macro.h \
    planner/lattice/lattice_planner.h \
    common/get_now_time.h \
    planning_base.h \
    std_planning.h \
    common/vehicle_state/vehicle_state_provider.h \
    planning.h \
    common/speed/st_boundary.h \
    common/speed/st_point.h \
    common/path_obstacle.h \
    common/path_decision.h \
    math/aaboxkdtree2d.h \
    toolkits/deciders/utill.h \
    toolkits/deciders/signal_light.h \
    common/speed/speed_data.h \
    common/get_traffic_rule.h \
    common/g_test.h \
    math/curve1d/curve1d.h \
    math/curve1d/polynomial_curve1d.h \
    math/curve1d/quartic_polynomial_curve1d.h \
    math/curve1d/quintic_polynomial_curve1d.h \
    math/aabox2d.h \
    math/aaboxkdtree2d.h \
    math/angle.h \
    math/box2d.h \
    math/cartesian_frenet_conversion.h \
    math/euler_angles_zxy.h \
    math/line_segment2d.h \
    math/linear_interpolation.h \
    math/math_utils.h \
    math/path_matcher.h \
    math/polygon2d.h \
    math/quaternion.h \
    math/search.h \
    math/sin_table.h \
    math/util.h \
    math/vec2d.h \
    common/write_read_file.h \
    common/coordinate_convert.h \
    map/map_struct.h \
    math/curve_math.h \
    qcustomplot.h \
    toolkits/deciders/stop_sign.h \
    common/garbage_deal.h \
    toolkits/deciders/destination.h \
    PlanPublish.h \
    common/MSFLPublish.h \
    ../map/hdmap.h \
    ../map/config/hdmap_config_param_initial.h \
    ../map/hdmap/hdmap_common.h \
    ../map/hdmap/hdmap_util.h \
    ../map/pnc_map/pnc_map.h \
    map/HdMapPublish.h \
    ../map/common/serialize_data.h \
    MSFLPublish.h \
    ../map/map_interface.h \
    ../map/HdMapPublish.h \
    ../map/common/pnc_macro.h\
    ../map/common/pnc_information.h\
    ../map/common/factory.h \
    ../map/common/pnc_util.h



FORMS += \
        mainwindow.ui

DISTFILES += \
    json/sconscript \
    ../map/json/sconscript
