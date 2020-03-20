#ifndef G_TEST_H
#define G_TEST_H
#include "stdio.h"
#include "iostream"
#include "../config/planning_config_param_initial.h"
using namespace std;

namespace planning {



template < typename T>
void ExpectEQ(T a,T b)
{
  if(a == b)
   {
      cout<< "Success! Expect EQ result Get it"<<endl;
   }
  else{
      double aa = (double)a;
      double bb = (double)b;
      cout<< "Error!!! Expect EQ result But is not... ... ..."<<"first value: "
          <<aa<<"; second value: "<<bb<<endl;
    }
}

void ExpectTrue(bool value);

void ExpectFalse(bool value);

template < typename T0>
void ExpectNear(T0 a,T0 b,T0 c){

  if((a - b <= c) || (a - b >= -c))
   {
      cout<< "Success! Expect value near result Get it"<<endl;
   }else{
      cout<< "Error!!! Expect value near result But is not ... ... ..."<<endl;
   }

}

template <typename T>
void ExpectLE(T a,T b){
  if(a <= b){
      cout<< "Success! ExpectLE value near result Get it"<<endl;
    }else{
      cout<< "Error!!! ExpectLE value near result But is not ... ... ..."<<endl;
    }
}

template <typename T>
void ExpectGE(T a,T b){
  if(a >= b){
      cout<< "Success! ExpectGE value near result Get it"<<endl;
    }else{
      cout<< "Error!!! ExpectGE value near result But is not ... ... ..."<<endl;
    }
}


} //end namespace planning


#endif // G_TEST_H
