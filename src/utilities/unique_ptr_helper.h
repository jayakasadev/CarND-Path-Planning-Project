#ifndef PATH_PLANNING_UNIQUE_PTR_HELPER_H
#define PATH_PLANNING_UNIQUE_PTR_HELPER_H

#include <memory>

// note: this implementation does not disable this overload for array types
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args){
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

#endif //PATH_PLANNING_UNIQUE_PTR_HELPER_H
