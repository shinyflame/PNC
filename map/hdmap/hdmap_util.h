#ifndef HDMAP_UTIL_H
#define HDMAP_UTIL_H


#include <memory>
#include <string>
#include "../HdMapPublish.h"

namespace hdmap {

class HDMapUtil {
   public:

      static string version_alg(){return "2.22.019";}  // version of algorithm

      static string version_bin(){return base_map_->version;}  // version of binary map file

      /**
       * @brief  base map pointer
       * @return  pointer of base map loaded from the file; nullptr if failed to load
       */
      static const HDMap* BaseMapPtr();

      /**
       * @brief get the original point of map coordinate system
       * @return original point of the hdmap
       */
      static MapPoint getOriginalPoint();

      /**
       * @brief load map from file
       * @param map_filename
       * @return true if it's successful; false if it's not
       */
      static bool loadMapFromFile (const std::string& map_filename); // assign base_map_

      /**
       * @brief transform coordintates from global to map system
       * @param vehicle_global_coordinates
       * @return
       */
      static Point3D mapCoordintateTransform(PointLLH &vehicle_global_coordinates);

      static HDMap* base_map_;     // changed in 2019-11-15


   private:
      ~HDMapUtil() = delete;

      // static unique_ptr<HDMap> base_map_; ///// Consider using normal pointer so that
                                            //  you don't automatically delete the old pointer and the content it points to



 };


}

#endif
