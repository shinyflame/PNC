#ifndef GARBAGE_DEAL_H
#define GARBAGE_DEAL_H
#include "reference_line_info.h"
#include <map>
#include <functional>
#include <utility>
#include <memory>
#include <queue>

using namespace std;
namespace planning {
class GarbageDeal
{
public:
  GarbageDeal( ReferenceLineInfo * ref_line_info,const vector<Garbage> &Garbages);
  void SetGarbagesSL();
  vector<SLPoint> GetGarbagesSL() { return garbages_sl_;}
  void Init();
  struct SLComparator
      : public std::binary_function<const SLPoint&, const SLPoint&, bool> {
    bool operator()(const SLPoint& left, const SLPoint& right) const {
      return left.s > right.s;
    }
  };

  std::priority_queue<SLPoint, std::vector<SLPoint>, SLComparator> sl_queue_;

private:
  ReferenceLineInfo * ref_line_info_ = nullptr;
  vector<Garbage> garbages_;
  static map<int,Garbage> garbages_pair_;
  vector<SLPoint> garbages_sl_;
};


}//namespace planning
#endif // GARBAGE_DEAL_H
