
#include "../common/g_test.h"

namespace planning {




void ExpectTrue(bool value)
{
  if(value)
   {
      cout<< "Success! Expect True result Get it"<<endl;
   }
  else{
      cout<< "Error!!! Expect True but result is False... ... ... "<<endl;
    }
}
void ExpectFalse(bool value)
{
  if(!value)
   {
      cout<< "Success! Expect False result Get it"<<endl;
   }
  else{
      cout<<"value:"<<value<<endl;
      cout<< "Error!!! Expect Fasle but result is Ture... ... ..."<<endl;
    }
}


} //end namespace planning
