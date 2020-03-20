#ifndef PNC_MACRO_H
#define PNC_MACRO_H

#include<mutex>
#include<utility>
#include <iomanip>
#include <iostream>

//init DISALLOW_COPY_AND_ASSIGN 2020/02/18

#undef DISALLOW_COPY_AND_ASSIGN

#define DISALLOW_COPY_AND_ASSIGN(classtype)                               \
    classtype(const classtype&)=delete;                                                          \
    classtype &operator=(const classtype&)=delete;


#define DECLARE_SINGLETON(classtype)                                                  \
    public:                                                                                                                   \
      static classtype* Instance(){                                                                        \
                                                                                                                                    \
            if(instance==nullptr){                                                                               \
                static std::once_flag   o_flag;                                                             \
                std::call_once(o_flag,[&](){   instance=new classtype();  });      \
          }                                                                                                                          \
                                                                                                                                      \
          return instance;                                                                                             \
     }                                                                                                                                \
                                                                                                                                       \
      static void  FreeInstance(){                                                                             \
            if(instance!=nullptr){                                                                                   \
               delete instance;                                                                                         \
               instance=nullptr;                                                                                       \
             }                                                                                                                         \
     }                                                                                                                                 \
     private:                                                                                                                    \
            static classtype* instance;                                                                         \
        classtype();                                                                                                         \
        DISALLOW_COPY_AND_ASSIGN(classtype);

#define  BUILD_SHARE_VAR(classtype)                                                                     \
    classtype* classtype::instance=nullptr;

#endif // PNC_MACRO_H
