
/**
 * @file
 * @brief Defines the Factory class.
 */

#ifndef COMMON_FACTORY_H_
#define COMMON_FACTORY_H_

#include <map>
#include <memory>
#include <utility>

namespace hdmap {

template <typename IdentifierType, class AbstractProduct,
          class ProductCreator = AbstractProduct *(*)(),
          class MapContainer = std::map<IdentifierType, ProductCreator>>
class Factory {
 public:

  bool Register(const IdentifierType &id, ProductCreator creator) {
    return producers_.insert(std::make_pair(id, creator)).second;
  }


  bool Unregister(const IdentifierType &id) {
    return producers_.erase(id) == 1;
  }

  bool Empty() const { return producers_.empty(); }

  template <typename... Args>
  std::unique_ptr<AbstractProduct> CreateObjectOrNull(const IdentifierType &id,
                                                      Args &&... args) {
    auto id_iter = producers_.find(id);
    if (id_iter != producers_.end()) {
      return std::unique_ptr<AbstractProduct>(
          (id_iter->second)(std::forward<Args>(args)...));
    }
    return nullptr;
  }

  template <typename... Args>
  std::unique_ptr<AbstractProduct> CreateObject(const IdentifierType &id,
                                                Args &&... args) {
    auto result = CreateObjectOrNull(id, std::forward<Args>(args)...);
    //AERROR_IF(!result) << "Factory could not create Object of type : " << id;
    return result;
  }

 private:
  MapContainer producers_;
};



} // namespace hdmap

#endif  // COMMON_FACTORY_H_
