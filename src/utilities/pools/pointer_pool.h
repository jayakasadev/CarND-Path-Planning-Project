#ifndef PATH_PLANNING_POINTER_POOL_H
#define PATH_PLANNING_POINTER_POOL_H

#include <iostream>
#include <memory>
#include <deque>
#include "../unique_ptr_helper.h"

template <class T>
class pointer_pool{
public:
    pointer_pool(){
        // std::cout << "pointer_pool constructor" << std::endl;
    }

    ~pointer_pool(){
        std::cout << "pointer_pool destructor" << std::endl;
    }

    pointer_pool(const pointer_pool &pointer_pool){
        std::cout << "pointer_pool copy constructor" << std::endl;
    }

    virtual void add(T * obj) = 0;

    virtual T* acquire() = 0;

    virtual T* acquire(short index) = 0;

    virtual void release(T* obj, short index) = 0;

    virtual void release(T* obj) = 0;

    virtual bool isEmpty() = 0;

    virtual void clear() = 0;

    virtual short size() = 0;

    virtual void print() = 0;
};

template <class T>
class shared_pool : public pointer_pool<T>{
protected:
    short index;
    std::deque<std::shared_ptr<T>> pool; // internally stored in shared_ptr
public:
    shared_pool() : pointer_pool<T>(){
        // std::cout << "shared_pool constructor" << std::endl;
        index = 0;
    }

    ~shared_pool(){
        std::cout << "shared_pool destructor" << std::endl;
    }

    shared_pool(const shared_pool &shared_pool){
        std::cout << "shared_pool copy constructor" << std::endl;
        for(short a = 0; a < shared_pool.pool.size(); a++){
            pool.push_back(shared_pool.pool[a]);
        }
        this->index = shared_pool.index;
    }

    inline void add(T * obj){
        // std::cout << *obj << std::endl;
        this->pool.push_back(std::make_shared<T>(*obj));
    }

    inline T* acquire(){
        if(pool.size() == 0){
            throw std::logic_error("shared_pool is empty. add some elements first");
        }
        return acquire(index++);
    }

    inline T* acquire(short index){
        if(pool.size() == 0){
            throw std::logic_error("shared_pool is empty. add some elements first");
        }
        return this->pool[index].get();
    }

    inline void release(T* obj, short index){
        if(pool.size() == 0){
            throw std::logic_error("shared_pool is empty. add some elements first");
        }
        // do nothing
    }

    inline void release(T* obj){
        if(pool.size() == 0){
            throw std::logic_error("shared_pool is empty. add some elements first");
        }
        // do nothing
    }

    inline bool isEmpty(){
        return this->pool.empty();
    }

    inline void clear(){
        this->pool.clear();
    }

    inline short size(){
        return this->pool.size();
    }

    void print(){
        for(short a = 0; a < this->pool.size(); a++){
            std::cout << pool[a] << std::endl;
        }
    }
};

template <class T>
class unique_pool : public pointer_pool<T>{
protected:
    short index;
    std::deque<std::unique_ptr<T>> pool; // internally stored in shared_ptr
public:
    unique_pool() : pointer_pool<T>(){
        // std::cout << "unique_pool constructor" << std::endl;
        index = 0;
    }

    ~unique_pool(){
        std::cout << "unique_pool destructor" << std::endl;
    }

    unique_pool(const unique_pool &unique_pool){
        std::cout << "shared_pool copy constructor" << std::endl;
        for(short a = 0; a < unique_pool.pool.size(); a++){
            pool.push_back(std::move(unique_pool.pool[a]));
        }
        this->index = unique_pool.index;
    }

    inline void add(T * obj){
        this->pool.push_back(make_unique<T>(*obj));
    }

    inline T* acquire(){
        if(pool.size() == 0){
            throw std::logic_error("shared_pool is empty. add some elements first");
        }
        return acquire(this->index++);
    }

    inline T* acquire(short index){
        if(pool.size() == 0){
            throw std::logic_error("shared_pool is empty. add some elements first");
        }

        T* ptr = this->pool[index].release();
        return ptr;
    }

    inline void release(T* obj, short index){
        if(pool.size() == 0){
            throw std::logic_error("shared_pool is empty. add some elements first");
        } else if(this->pool[index] != nullptr){
            throw std::invalid_argument("there is already a value at index");
        }
        this->pool.insert(this->pool.begin() + index, make_unique<T>(*obj));
    }

    inline void release(T* obj){
        if(pool.size() == 0){
            throw std::logic_error("shared_pool is empty. add some elements first");
        }
        this->pool.push_back(make_unique<T>(*obj));
    }

    inline bool isEmpty(){
        return this->pool.empty();
    }

    inline void clear(){
        this->pool.clear();
    }

    inline short size(){
        return this->pool.size();
    }

    void print(){
        for(short a = 0; a < this->pool.size(); a++){
            std::cout << *this->pool[a].get() << std::endl;
        }
    }
};

#endif //PATH_PLANNING_POINTER_POOL_H
