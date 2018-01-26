#ifndef PATH_PLANNING_SYNCH_POOL_H
#define PATH_PLANNING_SYNCH_POOL_H

#include <mutex>
#include <condition_variable>
#include <iostream>
#include "object_pool.h"
#include "pointer_pool.h"

class synch_pool{
protected:
    std::mutex mutex;
    std::condition_variable pool_permission;

    short live_get;
    short live_release;
    short waiting_release;
    short live_print;

public:
    synch_pool(){
        std::cout << "synch_pool constructor" << std::endl;
        live_release = 0;
        live_get = 0;
        live_print = 0;
        waiting_release = 0;
    }

    ~synch_pool(){
        std::cout << "synch_pool destructor" << std::endl;
    }
};

template <typename T>
class synch_object_pool : public synch_pool, public object_pool<T> {
public:
    synch_object_pool() : synch_pool(), object_pool<T>(){
        std::cout << "synch_object_pool constructor" << std::endl;
        live_release = 0;
        live_get = 0;
        live_print = 0;
        waiting_release = 0;
    }

    ~synch_object_pool(){
        std::cout << "synch_object_pool destructor" << std::endl;
    }

    void print() {
        std::unique_lock<std::mutex> lock(this->mutex);
        while(this->waiting_release > 0 || this->live_release > 0 || this->live_get > 0){
            this->pool_permission.wait(lock);
        }
        this->live_print++;

        object_pool<T>::print();

        this->live_print--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
    }

    T acquire(){
        std::unique_lock<std::mutex> lock(this->mutex);
        while(this->pool.empty() || this->waiting_release > 0 || this->live_release > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->live_get++;

        T ptr = object_pool<T>::acquire();

        this->live_get--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
        return ptr;
    }

    T acquire(short index){
        std::unique_lock<std::mutex> lock(this->mutex);
        while(this->pool.empty() || this->waiting_release > 0 || this->live_release > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->live_get++;

        T ptr = object_pool<T>::acquire(index);

        this->live_get--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
        return ptr;
    }

    void release(T obj, short index){
        std::unique_lock<std::mutex> lock(this->mutex);
        this->waiting_release++;
        while(this->live_get > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->live_release++;

        object_pool<T>::release(obj, index);

        this->live_release--;
        this->waiting_release--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
    }

    void release(T obj){
        std::unique_lock<std::mutex> lock(this->mutex);
        this->waiting_release++;
        while(this->live_get > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->live_release++;

        object_pool<T>::release(obj);

        this->live_release--;
        this->waiting_release--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
    }
};

template <typename T>
class synch_shared_pool : public synch_pool, public shared_pool<T> {
public:
    synch_shared_pool() : synch_pool(), shared_pool<T>(){
        std::cout << "synch_shared_pool constructor" << std::endl;
        live_release = 0;
        live_get = 0;
        live_print = 0;
        waiting_release = 0;
    }

    ~synch_shared_pool(){
        std::cout << "synch_shared_pool destructor" << std::endl;
    }

    void print() {
        std::unique_lock<std::mutex> lock(this->mutex);
        while(this->waiting_release > 0 || this->live_release > 0 || this->live_get > 0){
            this->pool_permission.wait(lock);
        }
        this->live_print++;

        shared_pool<T>::print();

        this->live_print--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
    }

    T* acquire(){
        std::unique_lock<std::mutex> lock(this->mutex);
        while(this->pool.empty() || this->waiting_release > 0 || this->live_release > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->live_get++;

        T* ptr = shared_pool<T>::acquire();

        this->live_get--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
        return ptr;
    }

    T* acquire(short index){
        std::unique_lock<std::mutex> lock(this->mutex);
        while(this->pool.empty() || this->waiting_release > 0 || this->live_release > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->live_get++;

        T* ptr = shared_pool<T>::acquire(index);

        this->live_get--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
        return ptr;
    }

    void release(T* obj, short index){
        std::unique_lock<std::mutex> lock(this->mutex);
        this->waiting_release++;
        while(this->live_get > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->live_release++;

        shared_pool<T>::release(obj, index);

        this->live_release--;
        this->waiting_release--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
    }

    void release(T* obj){
        std::unique_lock<std::mutex> lock(this->mutex);
        this->waiting_release++;
        while(this->live_get > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->live_release++;

        shared_pool<T>::release(obj);

        this->live_release--;
        this->waiting_release--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
    }
};

template <typename T>
class synch_unique_pool : public synch_pool, public unique_pool<T> {
public:
    synch_unique_pool() : synch_pool(), unique_pool<T>(){
        std::cout << "synch_unique_pool constructor" << std::endl;
        live_release = 0;
        live_get = 0;
        live_print = 0;
        waiting_release = 0;
    }

    ~synch_unique_pool(){
        std::cout << "synch_unique_pool destructor" << std::endl;
    }

    void print() {
        std::unique_lock<std::mutex> lock(this->mutex);
        while(this->waiting_release > 0 || this->live_release > 0 || this->live_get > 0){
            this->pool_permission.wait(lock);
        }
        this->live_print++;

        unique_pool<T>::print();

        this->live_print--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
    }

    T* acquire(){
        std::unique_lock<std::mutex> lock(this->mutex);
        while(this->pool.empty() || this->waiting_release > 0 || this->live_release > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->live_get++;

        T* ptr = unique_pool<T>::acquire();

        this->live_get--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
        return ptr;
    }

    T* acquire(short index){
        std::unique_lock<std::mutex> lock(this->mutex);
        while(this->pool.empty() || this->waiting_release > 0 || this->live_release > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->live_get++;

        T* ptr = unique_pool<T>::acquire(index);

        this->live_get--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
        return ptr;
    }

    void release(T* obj, short index){
        std::unique_lock<std::mutex> lock(this->mutex);
        this->waiting_release++;
        while(this->live_get > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->live_release++;

        unique_pool<T>::release(obj, index);

        this->live_release--;
        this->waiting_release--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
    }

    void release(T* obj){
        std::unique_lock<std::mutex> lock(this->mutex);
        this->waiting_release++;
        while(this->live_get > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->live_release++;

        unique_pool<T>::release(obj);

        this->live_release--;
        this->waiting_release--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
    }
};

#endif //PATH_PLANNING_SYNCH_POOL_H
