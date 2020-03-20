
#include "../math/curve_math.h"
#include "./reference_line_provider.h"
#include "../common/get_now_time.h"
#include <cmath>
extern ConfigParam g_config_param;
extern VehicleParam g_vehicle_config;

namespace planning {

//std::list<hdmap::RouteSegment> ReferenceLineProvider::routes_ ;
std::list<ReferenceLine>   ReferenceLineProvider::reference_lines_;
vector<hdmap::ReferenLineWithParam> ReferenceLineProvider::ref_line_with_params_;
map<int,int> ReferenceLineProvider::update_flag_ = { {0,0},{1,0},{2,0},{3,0},{4,0},
                                                     {5,0},{6,0},{7,0},{8,0},{9,0}
                                                   };

ReferenceLineProvider::ReferenceLineProvider(const hdmap::HDMap *base_map) {

  //pnc_map_.reset(new hdmap::PncMap(base_map));
  is_initialized_ = true;

}  // namespace planning

ReferenceLineProvider::
ReferenceLineProvider(const hdmap::PncRoutes* pnc_routes){
  for(hdmap::PncRoute pnc_route : *pnc_routes){
     for(int i = 0; i<pnc_route.route.reference_points.size();i++){

       pnc_route.route.reference_points.at(i).euler_angles.yaw =
          pnc_route.route.reference_points.at(i).euler_angles.yaw /180.0*M_PI;
       }
     bool is_update = false;
     if(update_flag_.size() > 10){
        update_flag_.clear();
     }
     if(update_flag_.find(pnc_route.route.id) == update_flag_.end()){
        update_flag_[pnc_route.route.id] = pnc_route.is_update;
        is_update = true;
     }else{
        if(update_flag_[pnc_route.route.id] != pnc_route.is_update){
            update_flag_[pnc_route.route.id] = pnc_route.is_update;
            is_update = true;
        }
     }
       routes_.emplace_back(pnc_route.route,    pnc_route.route.id,
                            pnc_route.on_route, is_update,pnc_route.side_slip);
     }
   is_initialized_ = true;
 }




void ReferenceLineProvider::UpdateVehicleState(const VehicleState& vehicle_state){
  vehicle_state_ = vehicle_state;
}

unsigned long long ReferenceLineProvider::LastTimeDelay(){

  return last_calculation_time_;
}

void ReferenceLineProvider::Stop() {
  is_stop_ = true;

}

bool ReferenceLineProvider::Start(){
  return is_initialized_ ;
}

bool ReferenceLineProvider::
 GetReferenceLines( std::list<ReferenceLine> *reference_lines,
                    std::list<hdmap::RouteSegment> *routes,
                    std::vector<hdmap::ReferenLineWithParam> *ref_line_with_params)
{
  double start_time = Clock::NowInMs();
  if (!CreateReferenceLine(reference_lines, routes, ref_line_with_params)) {
        cout << "Failed to create reference line"<<endl;
        return false;
      }
   //UpdateReferenceLine(*reference_lines, *segments);
   double end_time = Clock::NowInMs();
   last_calculation_time_ = end_time - start_time;
   return true;
}

bool ReferenceLineProvider::
CreateReferenceLine( std::list<ReferenceLine> *reference_lines,
                     std::list<hdmap::RouteSegment> *routes,
                     std::vector<hdmap::ReferenLineWithParam> *ref_line_with_params)
{
    double  t0 = Clock::NowInMs();
    if (!CreateRoutes(routes)) {
        cout << "Failed to create reference line from routing"<<endl;
        return false;
      }

    std::list<ReferenceLine> temp_reference_lines;
    std::vector<hdmap::ReferenLineWithParam> temp_ref_line_with_params;
    for(hdmap::RouteSegment route:*routes) //start for circle
    {
       hdmap::ReferenLineWithParam temp_ref_line_with_param;
      int route_id =  route.GetRouteId();
      cout<<"route id: "<<route_id<<endl;
      if(route.GetUpdate())
        {
          if(!SmoothReferenceLine(&route, &temp_ref_line_with_param))
          {
            cout << "Smooth Reference Line Error" << endl;
            return false;
          }
          vector<PathPoint> DiscretizedPoint;
          ToDiscretizedReferenceLine(temp_ref_line_with_param,DiscretizedPoint);
          ReferenceLine reference_line(DiscretizedPoint,route,temp_ref_line_with_param);
          temp_reference_lines.push_back(reference_line);
          temp_ref_line_with_params.push_back(temp_ref_line_with_param);

        }else{//start else

          if(!reference_lines_.empty()){//estimata last routes_ empty or not?
             int i = 0;
             for(auto &last_reference_line : reference_lines_){//start for
               if( last_reference_line.GetReferenceLineId() != route_id)
                   continue;
               temp_reference_lines.push_back(last_reference_line);
               temp_ref_line_with_params.push_back(ref_line_with_params_[i]);
               i++;
               break;
             }//end for
          }else{
             cout<<"Error last save reference_lines_ is empty,"
                   " Update again"<<endl;
             if(!SmoothReferenceLine(&route, &temp_ref_line_with_param))
             {
               cout << "Smooth Reference Line Error" << endl;
               return false;
             }
             vector<PathPoint> DiscretizedPoint;
             ToDiscretizedReferenceLine(temp_ref_line_with_param,DiscretizedPoint);
             ReferenceLine reference_line(DiscretizedPoint,route,temp_ref_line_with_param);
             temp_reference_lines.push_back(reference_line);
             temp_ref_line_with_params.push_back(temp_ref_line_with_param);

          }

        }//end else

    }//end for circle

    *reference_lines = temp_reference_lines;
    *ref_line_with_params = temp_ref_line_with_params;
    //reference_lines_.clear();
    reference_lines_ = temp_reference_lines;
    cout<<"routes size = "<<routes->size()<<endl;
    //ref_line_with_params_.clear();
    ref_line_with_params_ = temp_ref_line_with_params;
    return true;

}

void ReferenceLineProvider::CaculateYaw( vector<PathPoint> &points){
  for(int i = 1 ;i < points.size() ; i++)
    {
      double dx = points.at(i).x - points.at(i-1).x;
      double dy = points.at(i).y - points.at(i-1).y;
      double theta = 0;

      if( dx >= 0 )
       {
          if( dy >= 0)
            { theta = -(M_PI/2.0 - abs(atan(dy/dx))); }
          else
            { theta = -(M_PI/2.0 + abs(atan(dy/dx))); }
       } else {

          if( dy >= 0)
            { theta = M_PI/2.0 - abs(atan(dy/dx));}
          else
            { theta = M_PI/2.0 + abs(atan(dy/dx));}
       }

      //theta -=M_PI_2;
      if(theta > M_PI){
          theta = theta - 2.0*M_PI;
        }
      if(theta < -M_PI){
         theta  = theta + 2.0*M_PI;
        }
      points.at(i).theta = theta ;
    }
  points.at(0).theta = points.at(1).theta;
  return;
}
bool ReferenceLineProvider::CreateRoutes( std::list<hdmap::RouteSegment> *routes )
{

  for(auto route : routes_){

      routes->push_back(route);

    }
  return !routes->empty();
}

bool ReferenceLineProvider::SmoothReferenceLine(hdmap::RouteSegment* route,
                          hdmap::ReferenLineWithParam* referen_line_with_param )
{
  //1. generate anchor points discretize 25m each section
  std::vector<hdmap::MapPoint> anchor_points;

  GetAnchorPoints(route, &anchor_points);

  //2. divide 25m section to 5m part ,add Smooth function

  hdmap::MapPoint start_point ;
  double x,dx,d2x,d3x,y,dy,d2y,d3y;
  double ndx,nd2x,ndy,nd2y;

  for(int i = 1; i < anchor_points.size(); i++)
  {
    double ploy_line_length = anchor_points.at(i).s - anchor_points.at(i-1).s;
    int num_of_anchors =std::max(2, static_cast<int>(ploy_line_length /
                                    g_config_param.interval_length));//5.0m
    num_of_anchors += 5/g_config_param.interval_length; //add one more point
    std::vector<double> anchor_s;
    math::util::uniform_slice(anchor_points.at(i-1).s, anchor_points.at(i).s+5,
                                              num_of_anchors, &anchor_s);
    // 2.1 get fitting value point every 5m
    std::vector<hdmap::MapPoint> poly_points;
    for (const double s : anchor_s)
    {
      auto anchor = route->GetMatchPoint(s);
      poly_points.push_back(anchor);
    }
    // 2.2 get each point value xyz and s
    // get start point of 25m section (xyz and s) then
    // set Original Point of 25m section (xyz and theta)
    vector<double>route_S;
    vector<double>route_X;
    vector<double>route_Y;

    for(int j = 0; j < poly_points.size(); j++)
    { 

      if(j==0)
       {
         if(i == 1){//get first segment first Point
           start_point = poly_points.at(0);
         }else{//other segments replace first point
          poly_points.at(0) = start_point;
          }
         //save segment start point as segment orignal point
         referen_line_with_param->start_point.push_back(start_point);
       }
       //convert target point to start_point coordinate and return result

       Point3D Point = CoordinateTransformation(start_point,poly_points.at(j));
       route_S.push_back(j * g_config_param.interval_length);
       route_X.push_back(Point.x);
       route_Y.push_back(Point.y);

    }
    double last_start_yaw=start_point.euler_angles.yaw;

    //2.3 solve coefficient
    Eigen::MatrixXd A(6,1);
    Eigen::MatrixXd B(6,1);
    double wa = 100.0;
    double wb = 100.0;
    if( i == 1 ){
        MatrixSolveWithDerivative(route_S, route_Y,1,B );
        MatrixSolveWithDerivative(route_S, route_X,0,A );
      }else{
//        MatrixSolveWithDDerivative(route_S, route_Y,wb,ndy,nd2y, B );
//        MatrixSolveWithDDerivative(route_S, route_X,wa,ndx,nd2x, A );
        MatrixSolveWithDerivative(route_S, route_Y,1,B );
        MatrixSolveWithDerivative(route_S, route_X,0,A );
      }

    hdmap::Curve_Coefficient curve_param;
    for(int j = 0; j < 6; j++){
      curve_param.A[j] = A(j,0);
      curve_param.B[j] = B(j,0);
    }
    //get new start point
    start_point = GetNextStartPoint(start_point,curve_param,
                                         g_config_param.segment_length);

    referen_line_with_param->curve_param.push_back(curve_param);
    // transform dx ddx,dy ddy of the last curve to current coordinate
    SolveDerivative(g_config_param.segment_length,A,&x,&dx,&d2x,&d3x);
    SolveDerivative(g_config_param.segment_length,B,&y,&dy,&d2y,&d3y);

    double theta_yaw = (start_point.euler_angles.yaw-last_start_yaw);

    ndx =  cos(theta_yaw) * dx + sin(theta_yaw) * dy;
    ndy = -sin(theta_yaw) * dx + cos(theta_yaw) * dy;

    nd2x =  cos(theta_yaw) * d2x + sin(theta_yaw) * d2y;
    nd2y = -sin(theta_yaw) * d2x + cos(theta_yaw) * d2y;

  }


  return true;
}


void ReferenceLineProvider::
ToDiscretizedReferenceLine(const hdmap::ReferenLineWithParam &reference_line,
                           vector<PathPoint> &DiscretizedPoint )
{

   for(int i = 0; i < reference_line.curve_param.size(); i++)
   {
     double length = g_config_param.segment_length;

     for(double s = 0;
                s <=  length - g_config_param.discretion_length;
                s += g_config_param.discretion_length)
     {

       PathPoint path_point = SolveXY( s, reference_line.curve_param.at(i) );

       Point3D temp_point;
       temp_point.x = path_point.x ;
       temp_point.y = path_point.y ;
       Point3D trans_value = {0} ;
       trans_value.x = reference_line.start_point.at(i).point_enu.x;
       trans_value.y = reference_line.start_point.at(i).point_enu.y;
       //map_original_point_.euler_angles.yaw
        double rotate_value = reference_line.start_point.at(i).euler_angles.yaw;
       //convert position to world coordinate
       CoordinateConvert::
       PointRelativeTranslationAndRotaion(temp_point,trans_value,-rotate_value);
       path_point.x = temp_point.x;
       path_point.y = temp_point.y;
       path_point.theta += reference_line.start_point.at(i).euler_angles.yaw;
       //theta ( -M_PI ~ M_PI )
       if(path_point.theta > M_PI){
           path_point.theta = path_point.theta - 2.0*M_PI;
         }
       if(path_point.theta < -M_PI){
           path_point.theta = path_point.theta + 2.0*M_PI;
         }

       DiscretizedPoint.push_back(path_point);
     }
   }

   double dis = 0;
   for(int i = 0; i < DiscretizedPoint.size() ; i++){
     if(i > 0){
     dis = sqrt(pow(DiscretizedPoint.at(i).x - DiscretizedPoint.at(i-1).x,2) +
                pow(DiscretizedPoint.at(i).y - DiscretizedPoint.at(i-1).y,2) );
     DiscretizedPoint.at(i).s = dis + DiscretizedPoint.at(i-1).s;
    }

   }

   return;
}


PathPoint ReferenceLineProvider::GetMatchPointFromS(const double S,
     const hdmap::ReferenLineWithParam &referen_line_with_param)
{
    int match_index = S / g_config_param.segment_length;
    double s = S - match_index *  g_config_param.segment_length;

    PathPoint path_point =
    SolveXY( s, referen_line_with_param.curve_param.at(match_index));

    Point3D temp_point;
    temp_point.x = path_point.x ;
    temp_point.y = path_point.y ;
    Point3D trans_value = {0} ;
    trans_value.x = referen_line_with_param.start_point.at(match_index).point_enu.x;
    trans_value.y = referen_line_with_param.start_point.at(match_index).point_enu.y;
    //map_original_point_.euler_angles.yaw
     double rotate_value =
                    referen_line_with_param.start_point.at(match_index).euler_angles.yaw;
    //convert position to world coordinate
    CoordinateConvert::
    PointRelativeTranslationAndRotaion(temp_point,trans_value,-rotate_value);
    path_point.x = temp_point.x;
    path_point.y = temp_point.y;
    path_point.theta +=
                   referen_line_with_param.start_point.at(match_index).euler_angles.yaw;
    //theta ( -M_PI ~ M_PI )
    if(path_point.theta > M_PI){
        path_point.theta = path_point.theta - 2.0*M_PI;
      }
    if(path_point.theta < -M_PI){
        path_point.theta = path_point.theta + 2.0*M_PI;
      }
    path_point.s = S;
    return path_point;

}

void ReferenceLineProvider::
GetAnchorPoints( hdmap::RouteSegment * route,
                 std::vector<hdmap::MapPoint>* anchor_points)
{
  double reference_line_length = g_config_param.segment_length * 4;//route->Length();
  int num_of_anchors =std::max(2, static_cast<int>(reference_line_length /
                                  g_config_param.segment_length + 0.5));
  if(num_of_anchors != 4){
      cout<<"Error num of anchors is = "<<num_of_anchors<<endl;
      return;
    }
  std::vector<double> anchor_s;
  //0,25,50,75,100,125,150
  math::util::uniform_slice(0.0,reference_line_length,num_of_anchors,&anchor_s);
  //get hdmap match point from s
  for (const double s : anchor_s){
    hdmap::MapPoint anchor = route->GetMatchPoint(s);
    anchor_points->emplace_back(anchor);
  }
}

void ReferenceLineProvider::
GetAnchorPoint( hdmap::RouteSegment * route,
                 std::vector<hdmap::MapPoint>* anchor_points)
{
  double reference_line_length = route->Length();

  std::vector<double> anchor_s;
  //125,150
  math::util::uniform_slice(125.0,reference_line_length,1,&anchor_s);
  //get hdmap match point from s
  for (const double s : anchor_s){
    hdmap::MapPoint anchor = route->GetMatchPoint(s);
    anchor_points->emplace_back(anchor);
  }
}

Point3D ReferenceLineProvider::
CoordinateTransformation( hdmap::MapPoint OriginPoint,
                          hdmap::MapPoint TransTargetPoint )
{

    Point3D TmpPoint;
    TmpPoint.x = TransTargetPoint.point_enu.x - OriginPoint.point_enu.x;
    TmpPoint.y = TransTargetPoint.point_enu.y - OriginPoint.point_enu.y;
    TmpPoint.z = TransTargetPoint.point_enu.z - OriginPoint.point_enu.z;

    double yaw = OriginPoint.euler_angles.yaw;
    Point3D Point;
    Point.x =  cos(yaw) * TmpPoint.x + sin(yaw) * TmpPoint.y;
    Point.y = -sin(yaw) * TmpPoint.x + cos(yaw) * TmpPoint.y;
    Point.z = TmpPoint.z;

    return Point;
}

void ReferenceLineProvider::
MatrixSolve(const  vector<double> &s_value,
            const  vector<double> &y_value,
            Eigen::MatrixXd &A ){

//  for(int i = 0;i < 6;i++)
//  {
//    cout<<"S"<<i<<" = "<<s_value[i]<<endl;
//    cout<<"Y"<<i<<" = "<<y_value[i]<<endl;
//  }

  double s[6][6] ={0};
  for(int i = 0;i<6;i++)
      for(int j = 0;j<6;j++)
         s[i][j] = pow(s_value[i],j);

  Eigen::MatrixXd S(6,6);
  S<<s[0][0], s[0][1], s[0][2], s[0][3], s[0][4], s[0][5],
     s[1][0], s[1][1], s[1][2], s[1][3], s[1][4], s[1][5],
     s[2][0], s[2][1], s[2][2], s[2][3], s[2][4], s[2][5],
     s[3][0], s[3][1], s[3][2], s[3][3], s[3][4], s[3][5],
     s[4][0], s[4][1], s[4][2], s[4][3], s[4][4], s[4][5],
     s[5][0], s[5][1], s[5][2], s[5][3], s[5][4], s[5][5];

//  for(int i = 0;i<6;i++){
//      for(int j = 0;j<6;j++)
//       cout<<"S(" <<i<< "," <<j<<  ") = " <<S(i,j)<<endl;
//    }

  Eigen::MatrixXd Y(6,1);
  Y<<y_value[0],y_value[1],y_value[2],y_value[3],y_value[4],y_value[5];
  //S * A = Y
  //S*A=Y，那么A = S^(-1) * Y
  S = S.inverse();//逆矩阵
  A = S * Y;
  return;
}


void ReferenceLineProvider::
MatrixSolveWithDerivative(const vector<double> &s_value,
                          const vector<double> &y_value,
                          double d_s0,
                          double dd_s0,
                          Eigen::MatrixXd &A ){

//  for(int i = 0;i < s_value.size();i++)
//  {
//    cout<<"S"<<i<<" = "<<s_value[i]<<endl;
//    cout<<"Y"<<i<<" = "<<y_value[i]<<endl;
//  }

  double s[11] = {0};
  s[0] = s_value.size(); // -1 was abandoned by glk
  for(int i = 0; i < s_value.size(); i++){
      s[1]  += pow(s_value.at(i),1) ;
      s[2]  += pow(s_value.at(i),2) ;
      s[3]  += pow(s_value.at(i),3) ;
      s[4]  += pow(s_value.at(i),4) ;
      s[5]  += pow(s_value.at(i),5) ;
      s[6]  += pow(s_value.at(i),6) ;
      s[7]  += pow(s_value.at(i),7) ;
      s[8]  += pow(s_value.at(i),8) ;
      s[9]  += pow(s_value.at(i),9) ;
      s[10] += pow(s_value.at(i),10) ;
    }
  double s0[6] = {0};
  for(int i = 0;i < s_value.size();i++)
       s0[i] = pow(s_value.at(0),i) ;

  //Eigen::MatrixXd S(9,9);
  Eigen::Matrix<double, 9, 9> S;
  S<<s[0], s[1], s[2], s[3], s[4], s[5],  0.5*s0[0], 0.0,       0.0,
     s[1], s[2], s[3], s[4], s[5], s[6],  0.5*s0[1], 0.5,       0.0,
     s[2], s[3], s[4], s[5], s[6], s[7],  0.5*s0[2], s0[1] ,    1.0,
     s[3], s[4], s[5], s[6], s[7], s[8],  0.5*s0[3], 1.5*s0[2], 3.0 *s0[1],
     s[4], s[5], s[6], s[7], s[8], s[9],  0.5*s0[4], 2.0*s0[3], 6.0 *s0[2],
     s[5], s[6], s[7], s[8], s[9], s[10], 0.5*s0[5], 2.5*s0[4], 10.0*s0[3],
     s0[0], s0[1],   s0[2],      s0[3],      s0[4],      s0[5], 0.0, 0.0, 0.0,
     0.0,   s0[0], 2*s0[1],    3*s0[2],  4.0*s0[3],  5.0*s0[4], 0.0, 0.0, 0.0,
     0.0,     0.0,     2.0,  6.0*s0[1], 12.0*s0[2], 20.0*s0[3], 0.0, 0.0, 0.0;


//  for(int i = 0;i<6;i++){
//      for(int j = 0;j<6;j++)
//       cout<<"S(" <<i<< "," <<j<<  ") = " <<S(i,j)<<endl;
//    }

  double y[6] = {0};
  for(int i = 0;i < y_value.size();i++){
      for(int j = 0; j < y_value.size(); j++)
         y[i] += y_value[j] * pow(s_value[j],i);
    }

  //Eigen::MatrixXd Y(9,1);
  Eigen::Matrix<double, 9, 1> Y;
  Y<<y[0],y[1],y[2],y[3],y[4],y[5],s_value[0],d_s0,dd_s0;
  //S * A = Y
  //S*A=Y，那么A = S^(-1) * Y
  Eigen::Matrix<double, 9, 1> AA;
  S = S.inverse();//逆矩阵
  AA = S * Y;
  for(int i = 0 ; i < 6 ; i++){
      A(i,0) = AA(i,0);
    }
  return;
}

void ReferenceLineProvider::
MatrixSolveWithDDerivative(const vector<double> &s_value,
                               const vector<double> &y_value,
                               double w,
                               double d_s0,
                               double dd_s0,
                               Eigen::MatrixXd &A ){
    double s[11] = {0};
    s[0] = s_value.size(); // -1 was abandoned by glk
    double ss[10]={0};
    ss[0]=0.0;

    for(int i = 0; i < s_value.size(); i++){
        s[1]  += pow(s_value.at(i),1) ;
        s[2]  += pow(s_value.at(i),2) ;
        s[3]  += pow(s_value.at(i),3) ;
        s[4]  += pow(s_value.at(i),4) ;
        s[5]  += pow(s_value.at(i),5) ;
        s[6]  += pow(s_value.at(i),6) ;
        s[7]  += pow(s_value.at(i),7) ;
        s[8]  += pow(s_value.at(i),8) ;
        s[9]  += pow(s_value.at(i),9) ;
        s[10] += pow(s_value.at(i),10) ;
        ss[1] += pow(s_value.at(i),6) + 36.0  * w;
        ss[2] += pow(s_value.at(i),7) + 144.0 * w * s_value.at(i) ;
        ss[3] += pow(s_value.at(i),8) + 360.0 * w * pow(s_value.at(i),2) ;
        ss[4] += pow(s_value.at(i),7) + 144.0 * w * s_value.at(i);
        ss[5] += pow(s_value.at(i),8) + 576.0 * w * pow(s_value.at(i),2) ;
        ss[6] += pow(s_value.at(i),9) + 1440.0* w * pow(s_value.at(i),3) ;
        ss[7] += pow(s_value.at(i),8) + 360.0 * w * pow(s_value.at(i),2) ;
        ss[8] += pow(s_value.at(i),9) + 1440.0* w * pow(s_value.at(i),3) ;
        ss[9] += pow(s_value.at(i),10) + 3600.0*w * pow(s_value.at(i),4) ;

      }
    double s0[6] = {0};
    for(int i = 0;i < s_value.size();i++)
         s0[i] = pow(s_value.at(0),i) ;

    //Eigen::MatrixXd S(9,9);
    Eigen::Matrix<double, 9, 9> S;
    S<<s[0],  s[1],  s[2],    s[3],     s[4],      s[5],      0.5,       0.0,       0.0,
       s[1],  s[2],  s[3],    s[4],     s[5],      s[6],      0.5*s0[1], 0.5,       0.0,
       s[2],  s[3],  s[4],    s[5],     s[6],      s[7],      0.5*s0[2], s0[1] ,    1.0,
       s[3],  s[4],  s[5],    ss[1],    ss[2],     ss[3],     0.5*s0[3], 1.5*s0[2], 3.0 *s0[1],
       s[4],  s[5],  s[6],    ss[4],    ss[5],     ss[6],     0.5*s0[4], 2.0*s0[3], 6.0 *s0[2],
       s[5],  s[6],  s[7],    ss[7],    ss[8],     ss[9],     0.5*s0[5], 2.5*s0[4], 10.0*s0[3],
       s0[0], s0[1], s0[2],   s0[3],    s0[4],     s0[5],     0.0,       0.0,       0.0,
       0.0,   s0[0], s0[1]*2, s0[2]*3,  s0[3]*4,   s0[4]*5.0, 0.0,       0.0,       0.0,
       0.0,   0.0,   2.0,     s0[1]*6,  s0[2]*12,  s0[3]*20,  0.0,       0.0,       0.0;


  //  for(int i = 0;i<6;i++){
  //      for(int j = 0;j<6;j++)
  //       cout<<"S(" <<i<< "," <<j<<  ") = " <<S(i,j)<<endl;
  //    }

    double y[6] = {0};
    for(int i = 0;i < y_value.size();i++){
        for(int j = 0; j < y_value.size(); j++)
           y[i] += y_value[j] * pow(s_value[j],i);
      }

    //Eigen::MatrixXd Y(9,1);
    Eigen::Matrix<double, 9, 1> Y;
    Y<<y[0],y[1],y[2],y[3],y[4],y[5],0,d_s0,dd_s0;
    //S * A = Y
    //S*A=Y，那么A = S^(-1) * Y
    Eigen::Matrix<double, 9, 1> AA;
    S = S.inverse();//逆矩阵
    AA = S * Y;
    for(int i = 0 ; i < 6 ; i++){
        A(i,0) = AA(i,0);
      }
    return;

}

void ReferenceLineProvider::
MatrixSolveWithDerivative( const vector<double> &s_value,
                           const vector<double> &y_value,
                           double d_s0,
                           Eigen::MatrixXd &A ){

//  for(int i = 0;i < s_value.size();i++)
//  {
//    cout<<"S"<<i<<" = "<<s_value[i]<<endl;
//    cout<<"Y"<<i<<" = "<<y_value[i]<<endl;
//  }
//  cout<<"*******************************"<<endl;
  double s0[6] = {0};
  for(int i = 0;i < s_value.size();i++)
       s0[i] = pow(s_value.at(0),i) ;

  double s[11] = {0};
  s[0] = s_value.size();
  for(int i = 0; i < s_value.size(); i++){
      s[1]  += pow(s_value.at(i),1) ;
      s[2]  += pow(s_value.at(i),2) ;
      s[3]  += pow(s_value.at(i),3) ;
      s[4]  += pow(s_value.at(i),4) ;
      s[5]  += pow(s_value.at(i),5) ;
      s[6]  += pow(s_value.at(i),6) ;
      s[7]  += pow(s_value.at(i),7) ;
      s[8]  += pow(s_value.at(i),8) ;
      s[9]  += pow(s_value.at(i),9) ;
      s[10] += pow(s_value.at(i),10) ;
    }

  Eigen::Matrix<double, 8, 8> S;
  S<<s[0], s[1], s[2], s[3], s[4], s[5],  0.5*s0[0], 0.0,
     s[1], s[2], s[3], s[4], s[5], s[6],  0.5*s0[1], 0.5,
     s[2], s[3], s[4], s[5], s[6], s[7],  0.5*s0[2], s0[1] ,
     s[3], s[4], s[5], s[6], s[7], s[8],  0.5*s0[3], 1.5*s0[2],
     s[4], s[5], s[6], s[7], s[8], s[9],  0.5*s0[4], 2.0*s0[3],
     s[5], s[6], s[7], s[8], s[9], s[10], 0.5*s0[5], 2.5*s0[4],
     s0[0], s0[1],   s0[2],      s0[3],      s0[4],      s0[5], 0.0, 0.0,
     0.0,   s0[0], 2*s0[1],    3*s0[2],  4.0*s0[3],  5.0*s0[4], 0.0, 0.0;



//  for(int i = 0;i<6;i++){
//      for(int j = 0;j<6;j++)
//       cout<<"S(" <<i<< "," <<j<<  ") = " <<S(i,j)<<"  ";
//      cout<<endl;
//    }

  double y[6] = {0};
  for(int i = 0;i < y_value.size();i++){
      for(int j = 0; j < y_value.size(); j++)
         y[i] += y_value[j] * pow(s_value[j],i);
    }

//  for(int i = 0;i < y_value.size();i++){
//      cout<<"y["<<i<<"] = "<<y[i]<<endl;
//    }

  Eigen::MatrixXd Y(8,1);

  Y<<y[0],y[1],y[2],y[3],y[4],y[5],s_value.at(0),d_s0;
  //S * A = Y
  //S*A=Y，那么A = S^(-1) * Y
  S = S.inverse();//逆矩阵
  Eigen::MatrixXd AA(8,1);
  AA = S * Y;
  for(int i = 0 ; i < 6 ; i++){
      A(i,0) = AA(i,0);
    }

  return;
}


PathPoint ReferenceLineProvider::
SolveXY( const double & s,const  hdmap::Curve_Coefficient &curve_param)
{
    Eigen::MatrixXd A(6,1);
    Eigen::MatrixXd B(6,1);
    for(int i = 0; i < 6; i++) {
      A(i,0) = curve_param.A[i];
      B(i,0) = curve_param.B[i];
    }

    double x,dx,d2x,d3x;
    double y,dy,d2y,d3y;
    //slove dx / dy
    SolveDerivative(s,A,&x,&dx,&d2x,&d3x);
    SolveDerivative(s,B,&y,&dy,&d2y,&d3y);
    PathPoint path_point;
    path_point.s = s;
    path_point.x = x;
    path_point.y = y;
    path_point.z = 0.0;
    double theta;//theta is on local coordinate system angle with y-axis right is(-)
    //left is (+)
    theta = -atan2(dx,dy);
    /*
    if( dx >= 0 )
     {
        if( dy >= 0)
          { theta = -(M_PI/2.0 - abs(atan(dy/dx))); }
        else
          { theta = -(M_PI/2.0 + abs(atan(dy/dx))); }
     } else {

        if( dy >= 0)
          { theta = M_PI/2.0 - abs(atan(dy/dx));}
        else
          { theta = M_PI/2.0 + abs(atan(dy/dx));}//
      }
    */
    if(s == 0.0 )
    path_point.theta=0.0;
    else
    path_point.theta  = theta  ;
    static int index_i = 0;
    if(index_i < 10)
     cout<<"theta "<<index_i++ <<" = "<<path_point.theta / M_PI *180.0<<endl;
    path_point.kappa  = CurveMath::ComputeCurvature(dx, d2x, dy, d2y);
    path_point.dkappa = CurveMath::ComputeCurvatureDerivative( dx, d2x, d3x,
                                                               dy, d2y, d3y );
    return path_point;
}




void ReferenceLineProvider::
SolveDerivative(double s0, Eigen::MatrixXd &A,
                double *f_s,double *ds0,double *dds0,double *ddds0)
{
  *f_s =  A(0,0) +
          A(1,0) * s0 +
          A(2,0) * s0 * s0 +
          A(3,0) * s0 * s0 * s0 +
          A(4,0) * s0 * s0 * s0 * s0 +
          A(5,0) * s0 * s0 * s0 * s0 * s0;

  *ds0 = A(1,0) +
         2 * A(2,0) * s0 +
         3 * A(3,0) * s0 *s0 +
         4 * A(4,0) * s0 * s0 * s0 +
         5 * A(5,0) * s0 * s0* s0 * s0;

  *dds0 =  2 * A(2,0) +
           6 * A(3,0) * s0 +
          12 * A(4,0) * s0 * s0 +
          20 * A(5,0) * s0 * s0 * s0;

  *ddds0 =  6 * A(3,0) +
           24 * A(4,0) * s0 +
           60 * A(5,0) * s0 * s0;
  return;
}

double FiveCurveFuction(Eigen::MatrixXd &A, double x){

  return A(0,0) +
         A(1,0) * x +
         A(2,0) * x * x +
         A(3,0) * x * x * x +
         A(4,0) * x * x * x * x+
         A(5,0) * x * x * x * x * x;
}

hdmap::MapPoint ReferenceLineProvider::
GetNextStartPoint(  const hdmap::MapPoint &first_start_point,
                    const hdmap::Curve_Coefficient &curve_param,
                    const double & s
                    ){

  PathPoint path_point = SolveXY( s,curve_param );

  Point3D temp_point;
  temp_point.x = path_point.x ;
  temp_point.y = path_point.y ;
  Point3D trans_value = {0} ;
  //map_original_point_.euler_angles.yaw
   double rotate_value = first_start_point.euler_angles.yaw;
  //convert position to world coordinate
   trans_value.x = first_start_point.point_enu.x;
   trans_value.y = first_start_point.point_enu.y;
  CoordinateConvert::
  PointRelativeTranslationAndRotaion(temp_point,trans_value,-rotate_value);

  path_point.theta += first_start_point.euler_angles.yaw;
  //theta ( -M_PI ~ M_PI )
  if(path_point.theta > M_PI){
      path_point.theta = path_point.theta - 2.0*M_PI;
    }
  if(path_point.theta < -M_PI){
      path_point.theta = path_point.theta + 2.0*M_PI;
    }

  hdmap::MapPoint map_point;
  map_point.point_enu.x = temp_point.x;
  map_point.point_enu.y = temp_point.y;
  map_point.euler_angles.yaw = path_point.theta;
  map_point.s = path_point.kappa;

  return map_point;

}

double ReferenceLineProvider::CaculateKappa(PathPoint pre_p,PathPoint suc_p){

  double theta_pre = pre_p.theta;
  double theta_suc = suc_p.theta;
  double s_pre = pre_p.s;
  double s_suc = suc_p.s;
  double s_dif = s_suc - s_pre;

  if(theta_pre * theta_suc < -9.0){

     if(theta_pre > 0){
         theta_suc += M_PI * 2.0;
      } else {
         theta_suc -= M_PI * 2.0;
      }

   }

  return (theta_suc - theta_pre)/(s_dif + g_config_param.lattice_epsilon);

 }

double ReferenceLineProvider::CaculateKappa(PathPoint p_a,PathPoint p_b,PathPoint p_c){


  Vec2d ab(p_b.x - p_a.x,p_b.y - p_a.y);
  Vec2d ac(p_c.x - p_a.x,p_c.y - p_a.y);
  Vec2d bc(p_c.x - p_b.x,p_c.y - p_b.y);

  double Sabc = ab.CrossProd(ac);

  return Sabc * 4.0 / ab.Length() *ac.Length() *bc.Length();
 }


PathPoint ReferenceLineProvider::
AnalyticallyFindNearestRefPoint(const double x, const double y,
                                const ReferenceLine * reference_line,
                                double tol, int max_iter) {
    cout<<"input (x,y) = ("<<x<<", "<<y<<")"<<endl;
    auto ref_with_param = reference_line->GetReferenceLineWithParam();
    double s = 5;

    PathPoint matched_point;

    int num_iter = 0;
    double diff = 100;
    int idx = 0;
    vector<double> f_vals;
    while (num_iter ++ < max_iter && diff > tol){
       /// compute f', f'', f
       f_vals = DistanceToThePointOnReferenceLine(x, y, s,
                                                  ref_with_param, idx, matched_point);
       s = s - f_vals[0] / f_vals[1];
       diff = abs(f_vals[0]);

    }
    f_vals = DistanceToThePointOnReferenceLine(x, y, s,
                                               ref_with_param, idx, matched_point);
    matched_point.s = s + idx * g_config_param.segment_length;
    cout << "AnalyticallyFindNearestRefPoint:(x, y, theta, s) "
         << matched_point.x << "," << matched_point.y << ","
         << matched_point.theta << "," << matched_point.s << endl;
    return matched_point;
}

vector<double> ReferenceLineProvider::
DistanceToThePointOnReferenceLine(const double x, const double y,const double s,
                                  hdmap::ReferenLineWithParam ref_with_param,
                                  int idx, PathPoint &matched_point){
    vector<double> res;
    double real_s = s;

    for(int i = 0; i < ref_with_param.curve_param.size();i++){
        if(s >= i * g_config_param.segment_length &&
                s < (i + 1) * g_config_param.segment_length){
            idx = i;
            real_s = s - i * g_config_param.segment_length;
            break;
        }
    }

    auto curve_param = ref_with_param.curve_param[idx];
    auto start_point = ref_with_param.start_point[idx];
    /// solve x, dx, d2x, y, dy, d2y
    Eigen::MatrixXd A(6,1);
    Eigen::MatrixXd B(6,1);
    for(int i = 0; i < 6; i++) {
      A(i,0) = curve_param.A[i];
      B(i,0) = curve_param.B[i];
    }

    double rx,rdx,rd2x,rd3x;
    double ry,rdy,rd2y,rd3y;
    //slove dx / dy
    SolveDerivative(real_s,A,&rx,&rdx,&rd2x,&rd3x);
    SolveDerivative(real_s,B,&ry,&rdy,&rd2y,&rd3y);
    Point3D rdPoint;
    Point3D rddPoint;
    rdPoint.x = rdx;
    rdPoint.y = rdy;
    rddPoint.x = rd2x;
    rddPoint.y = rd2y;
    /// find matched reference point

    PathPoint path_point = SolveXY(real_s, curve_param);

    FromLocalToGlobal(path_point, start_point);
    matched_point = path_point;

    res = ComputeDerivative(x, y, matched_point, rdPoint, rddPoint, start_point);

    double dx = path_point.x - x;
    double dy = path_point.y - y;
    double dist =  dx * dx + dy * dy;
    res.push_back(dist);

    return res;
}


void ReferenceLineProvider::
FromLocalToGlobal(PathPoint &path_point, hdmap::MapPoint start_point){
    Point3D temp_point;
    temp_point.x = path_point.x ;
    temp_point.y = path_point.y ;
    Point3D trans_value = {0} ;
    trans_value.x = start_point.point_enu.x;
    trans_value.y = start_point.point_enu.y;
    //map_original_point_.euler_angles.yaw
     double rotate_value = start_point.euler_angles.yaw;
    //convert position to world coordinate
    CoordinateConvert::
    PointRelativeTranslationAndRotaion(temp_point,trans_value,-rotate_value);
    path_point.x = temp_point.x;
    path_point.y = temp_point.y;
    path_point.theta += start_point.euler_angles.yaw;
    //theta ( -M_PI ~ M_PI )
    if(path_point.theta > M_PI){
        path_point.theta = path_point.theta - 2.0*M_PI;
      }
    if(path_point.theta < -M_PI){
        path_point.theta = path_point.theta + 2.0*M_PI;
      }
}


vector<double> ReferenceLineProvider::
ComputeDerivative(const double x0, const double y0,
                  PathPoint &Point, Point3D &dPoint,
                  Point3D &ddPoint, const hdmap::MapPoint start_point) {
  using namespace Eigen;

  double df, ddf;
  vector<double> res;

  Point3D trans_value = {0} ;
  trans_value.x = start_point.point_enu.x;
  trans_value.y = start_point.point_enu.y;
  double rot_val = start_point.euler_angles.yaw;

  auto gx = Point.x;
  auto gy = Point.y;
  auto dx_loc = dPoint.x;
  auto dy_loc = dPoint.y;
  auto ddx_loc = ddPoint.x;
  auto ddy_loc = ddPoint.y;
  auto sin_val = sin(rot_val);
  auto cos_val = cos(rot_val);
  auto dx_global = dx_loc * cos_val - dy_loc * sin_val;
  auto dy_global = dx_loc * sin_val + dy_loc * cos_val;

  auto ddx_global = ddx_loc * cos_val - ddy_loc * sin_val;
  auto ddy_global = ddx_loc * sin_val + ddy_loc * cos_val;


  df = 2 * (gx - x0) * dx_global + 2 * (gy - y0) * dy_global;
      ddf = 2 * dx_global * dx_global + 2 * (gx - x0) * ddx_global +
            2 * dy_global * dy_global + 2 * (gy - y0) * ddy_global;
  res.push_back(df);
  res.push_back(ddf);

  return res;

}



} //end namespace planning

