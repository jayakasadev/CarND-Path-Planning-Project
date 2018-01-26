#ifndef PATH_PLANNING_OBJECT_POOL_H
#define PATH_PLANNING_OBJECT_POOL_H

#include <deque>
#include <iostream>

template <class T>
class object_pool{
protected:
    std::deque<T> pool;
public:
    object_pool(){
        // std::cout << "object_pool constructor" << std::endl;
    }

    ~object_pool(){
        std::cout << "object_pool destructor" << std::endl;
    }

    T acquire(){
        if(this->pool[index] == NULL){
            throw std::invalid_argument("there is an object at index in pool");
        }
        T ptr = this->pool.front();
        this->pool.pop_front();
        return ptr;
    }

    T acquire(short index){
        if(this->pool.size() == 0){
            throw std::invalid_argument("there are no objects in pool");
        }
        else if(this->pool[index] == NULL){
            throw std::invalid_argument("there is no object at index in pool");
        }
        T ptr = this->pool[index];
        this->pool.erase(pool.begin() + index); // delete element at index
        return ptr;
    }

    inline void release(T obj, short index){
        if(this->pool[index] != NULL){
            throw std::invalid_argument("there is an object at index");
        }
        this->pool.insert(pool.begin() + index, obj);
    }

    inline void release(T obj){
        this->pool.push_back(obj);
    }

    inline bool isEmpty(){
        return pool.empty();
    }

    inline void clear(){
        pool.clear();
    }

    inline short size(){
        return pool.size();
    }

    void print() {
        for(short a = 0; a < pool.size(); a++){
            std::cout << this->pool[a] << std::endl;
        }
    }
};

#endif //PATH_PLANNING_OBJECT_POOL_H
