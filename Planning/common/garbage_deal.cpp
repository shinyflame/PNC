
#include "garbage_deal.h"
extern ConfigParam g_config_param;
namespace planning {

map<int,Garbage> GarbageDeal::garbages_pair_;

GarbageDeal::GarbageDeal(ReferenceLineInfo * ref_line_info,const vector<Garbage> &Garbages)
           :ref_line_info_(ref_line_info),garbages_(Garbages) {  Init();  }

void GarbageDeal::Init(){

  auto &reference_line = ref_line_info_->reference_line();
  auto adc_sl = ref_line_info_->GetVechicleSLPoint();
  cout<<"adc_sl s = "<<adc_sl.s<<endl;
  //1. update garbages
  for(auto garbage: garbages_){
     garbages_pair_[garbage.id] = garbage;
  }

  //2. computer garbage sl remove behind clean vehicle garbage and add sl_queue
  map<int,Garbage> temp_pair;
  double clean_max_dis   = g_config_param.clean_max_dis;
  double clean_max_width = g_config_param.clean_max_width;
  double clean_min_width = g_config_param.clean_min_width;
  for(pair<int,Garbage> garbage_pair:garbages_pair_){
      SLPoint sl;
      auto pos = garbage_pair.second.position;
      reference_line.XYToSL({pos.x,pos.y},&sl);
      if( sl.s >adc_sl.s &&sl.s < adc_sl.s + clean_max_dis &&
          sl.l > clean_min_width && sl.l < clean_max_width){
          temp_pair.emplace(garbage_pair);
          //sl_queue_.emplace(sl);
          garbages_sl_.push_back(sl);
        }
    }
  //3. save
  cout<<"before update garbages num = "<<garbages_pair_.size()<<endl;
  garbages_pair_ = temp_pair;
  cout<<"after  update garbages num = "<<garbages_pair_.size()<<endl;

}


void GarbageDeal::SetGarbagesSL(){

 while( sl_queue_.size() > 0){

      garbages_sl_.push_back(sl_queue_.top());
      sl_queue_.pop();
   }

 }


}// end namespace planning
