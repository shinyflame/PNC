#ifndef REFERENCE_LINE_PROVIDER_H
#define REFERENCE_LINE_PROVIDER_H

#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <Eigen/Eigen>
#include "../map/route_segment.h"
#include "../reference_line/reference_line.h"
#include "../math/util.h"
#include "../common/coordinate_convert.h"

namespace planning {


class ReferenceLineProvider
{
public:
  ReferenceLineProvider() = default ;
  ReferenceLineProvider(const hdmap::HDMap* base_map);
  explicit ReferenceLineProvider(const hdmap::PncRoutes* pnc_routes);

    bool GetReferenceLines( std::list<ReferenceLine> *reference_lines,
                            std::list<hdmap::RouteSegment> *routes,
                            std::vector<hdmap::ReferenLineWithParam> *ref_line_with_param);
    bool CreateReferenceLine( std::list<ReferenceLine> *reference_lines,
                              std::list<hdmap::RouteSegment> *routes,
                              std::vector<hdmap::ReferenLineWithParam> *ref_line_with_param);
    bool SmoothReferenceLine(hdmap::RouteSegment* Route,
                             hdmap::ReferenLineWithParam* reference_line_with_param);

    void MatrixSolve(const vector<double> &s,
                     const vector<double> &x,
                     const double & d_s0,
                     Eigen::MatrixXd &A );
    void MatrixSolve(const vector<double> &s,
                     const vector<double> &x,
                     Eigen::MatrixXd &A );
    PathPoint SolveXY(const double &s, const hdmap::Curve_Coefficient &curve_param);
    void ToDiscretizedReferenceLine(const hdmap::ReferenLineWithParam &referen_line_with_param,
                                    vector<PathPoint> &DiscretizedPoint);
    PathPoint PlanningCoordinateTransformation( const hdmap::MapPoint &original_point,
                                                PathPoint TransPoint  );
    Point3D CoordinateTransformation(hdmap::MapPoint OriginPoint,
                                     hdmap::MapPoint TransTargetPoint);
    void SolveDerivative(double s0, Eigen::MatrixXd &A,
                         double *ds,double *ds0,double *dds0,double *ddds0);
    void GetAnchorPoints(hdmap::RouteSegment * route,
                         std::vector<hdmap::MapPoint>* anchor_points);
    void GetAnchorPoint( hdmap::RouteSegment * route,
                     std::vector<hdmap::MapPoint>* anchor_points);
    void UpdateVehicleState(const VehicleState& vehicle_state);

    bool CreateRoutes( std::list<hdmap::RouteSegment> *routes );

    void MatrixSolveWithDerivative(const vector<double> &s_value,
                                   const vector<double> &y_value,
                                   double ds0,
                                   double dds0,
                                   Eigen::MatrixXd &A );
    void MatrixSolveWithDerivative(const vector<double> &s_value,
                                   const vector<double> &y_value,
                                   double d_s0,
                                   Eigen::MatrixXd &A );


    hdmap::MapPoint GetNextStartPoint( const hdmap::MapPoint &first_start_point,
                                       const hdmap::Curve_Coefficient &curve_param,
                                       const double & s);
    void MatrixSolveWithDDerivative(const vector<double> &s_value,
                                   const vector<double> &y_value,
                                   double w,
                                   double d_s0,
                                   double dd_s0,
                                   Eigen::MatrixXd &A );
    unsigned long long LastTimeDelay();
    void CaculateYaw(vector<PathPoint> &points);
    double CaculateKappa(PathPoint pre_p,PathPoint suc_p);
    double CaculateKappa(PathPoint p_a,PathPoint p_b,PathPoint p_c);

    void Stop();
    bool Start();
    SLPoint GetAdcSLPoint() {
        Vec2d adc_xy(vehicle_state_.x,vehicle_state_.y);
        SLPoint adc_sl;
        routes_.front().GetProjection(adc_xy,adc_sl.s,adc_sl.l);
        return adc_sl;
    }
    PathPoint AnalyticallyFindNearestRefPoint(const double x, const double y,
                                              const ReferenceLine * reference_line,
                                              double tol, int max_iter);
    vector<double> DistanceToThePointOnReferenceLine(const double x, const double y, const double s,
                                                     hdmap::ReferenLineWithParam ref_with_param,
                                                     int idx, PathPoint &matched_point );
    void FromLocalToGlobal(PathPoint &path_point, hdmap::MapPoint start_point);
    vector<double> ComputeDerivative(const double x0, const double y0,
    PathPoint &Point, Point3D &dPoint, Point3D &ddPoint, const hdmap::MapPoint start_point);
    PathPoint GetMatchPointFromS(const double S,
         const hdmap::ReferenLineWithParam &referen_line_with_param);
private:

    bool is_stop_ = false;
    bool is_initialized_ = false;

    std::mutex pnc_map_mutex_;
    //std::unique_ptr<hdmap::PncMap> pnc_map_;

    std::mutex vehicle_state_mutex_;
    VehicleState vehicle_state_;
    hdmap::MapPoint map_original_point_;
    uint64_t last_calculation_time_ = 0.0;
//    static int update_ ;
    list<hdmap::RouteSegment> routes_;
    static std::list<ReferenceLine> reference_lines_;
    static vector<hdmap::ReferenLineWithParam> ref_line_with_params_;
    static map<int,int> update_flag_;
//    static std::vector<hdmap::EndSegmentCondition> end_conditions_;
};
double FiveCurveFuction(Eigen::MatrixXd &A, double x);
}//end namespace planning
#endif // REFERENCE_LINE_PROVIDER_H
