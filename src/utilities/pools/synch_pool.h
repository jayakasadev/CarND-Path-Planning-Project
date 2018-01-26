#ifndef PATH_PLANNING_SYNCH_POOL_H
#define PATH_PLANNING_SYNCH_POOL_H

#include <mutex>
#include <condition_variable>
#include <iostream>
#include "object_pool.h"
#include "pointer_pool.h"

template <class T>
class synch_pool{
protected:
    std::mutex mutex;
    std::condition_variable pool_permission;

    short live_get;
    short live_release;
    short live_print;
    short live_add;

    short waiting_release;
    short waiting_add;

public:
    synch_pool(){
        std::cout << "synch_pool constructor" << std::endl;
        live_release = 0;
        live_get = 0;
        live_print = 0;
        waiting_release = 0;
        waiting_add = 0;
        live_add = 0;
    }

    ~synch_pool(){
        std::cout << "synch_pool destructor" << std::endl;
    }
};

template <typename T>
class synch_object_pool : public synch_pool<T>, public object_pool<T>{
private:
    std::deque<T> pool;
public:
    synch_object_pool() : synch_pool<T>(), object_pool<T>(){
        std::cout << "synch_object_pool constructor" << std::endl;
    }

    ~synch_object_pool(){
        std::cout << "synch_object_pool destructor" << std::endl;
    }

    virtual std::ostream& print(std::ostream& os) {
        std::unique_lock<std::mutex> lock(this->mutex);
        if(this->waiting_add > 0 || this->live_add > 0 || this->waiting_release > 0  || this->live_release > 0 || this->live_get > 0){
            this->pool_permission.wait(lock);
        }
        this->live_print++;

        os << "synch_object_pool" << std::endl;
        for(short a = 0; a < pool.size(); a++){
            os << pool[a] << std::endl;
        }

        this->live_print--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of others
        }
        return os;
    }

    void add(T obj){
        std::unique_lock<std::mutex> lock(this->mutex);
        this->waiting_add++;
        while(this->live_add > 0 || this->live_release > 0 || this->live_get > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->waiting_add--;
        this->live_add++;

        this->pool.push_back(obj);

        this->live_add--;
        if(this->waiting_add > 0 || this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
    }

    T& acquire(){
        std::unique_lock<std::mutex> lock(this->mutex);
        while(this->waiting_add > 0 || this->live_add > 0 || this->waiting_release > 0 || this->live_release > 0
              || this->live_get > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->live_get++;

        if(this->pool.empty()){
            // rmr to release all locks
            this->live_get--;
            if(this->waiting_add > 0 || this->waiting_release > 0){
                this->pool_permission.notify_one();
            } else{
                this->pool_permission.notify_all(); // lot of others
            }
            lock.unlock();

            throw std::invalid_argument("there is an object at index in pool");
        }
        T ptr = this->pool.front();
        this->pool.pop_front();

        this->live_get--;
        if(this->waiting_add > 0 || this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
        return ptr;
    }

    T& acquire(short index){
        std::unique_lock<std::mutex> lock(this->mutex);
        while(this->waiting_add > 0 || this->live_add > 0 || this->waiting_release > 0 || this->live_release > 0
              || this->live_get > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->live_get++;

        if(this->pool.empty() || index > pool.size()){
            // rmr to release all locks
            this->live_get--;
            if(this->waiting_add > 0 || this->waiting_release > 0){
                this->pool_permission.notify_one();
            } else{
                this->pool_permission.notify_all(); // lot of getters
            }
            lock.unlock();

            throw std::invalid_argument("there is no objects or not enough objects in the pool");
        }

        T ptr = this->pool[index];
        this->pool.erase(pool.begin() + index); // delete element at index

        this->live_get--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
        return ptr;
    }

    void release(T& obj, short index){
        std::unique_lock<std::mutex> lock(this->mutex);
        this->waiting_release++;
        while(this->waiting_add > 0 || this->live_add > 0 || this->live_release > 0 || this->live_get > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->waiting_release--;
        this->live_release++;

        if(this->pool[index] != NULL){
            this->live_release--;
            if(this->waiting_release > 0){
                this->pool_permission.notify_one();
            } else{
                this->pool_permission.notify_all(); // lot of getters
            }
            lock.unlock();

            throw std::invalid_argument("there is an object at index");
        }
        this->pool.insert(pool.begin() + index, obj);

        this->live_release--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
    }

    void release(T& obj){
        std::unique_lock<std::mutex> lock(this->mutex);
        this->waiting_release++;
        while(this->waiting_add > 0 || this->live_add > 0 || this->live_release > 0 || this->live_get > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->waiting_release--;
        this->live_release++;

        this->pool.push_back(obj);

        this->live_release--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
    }
};

template <typename T>
class synch_shared_pool : public synch_pool<T>, public shared_pool<T>{
public:
    synch_shared_pool() : synch_pool<T>(), shared_pool<T>(){
        std::cout << "synch_shared_pool constructor" << std::endl;
    }

    ~synch_shared_pool(){
        std::cout << "synch_shared_pool destructor" << std::endl;
    }

    virtual std::ostream& print(std::ostream& os) {
        std::unique_lock<std::mutex> lock(this->mutex);
        if(this->waiting_add > 0 || this->live_add > 0 || this->waiting_release > 0  || this->live_release > 0 || this->live_get > 0){
            this->pool_permission.wait(lock);
        }
        this->live_print++;

        os << "synch_shared_pool"<< std::endl;
        for(short a = 0; a < this->pool.size(); a++){
            os << *(this->pool[a].get()) << std::endl;
        }

        this->live_print--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of others
        }
        return os;
    }

    void add(std::shared_ptr<T> obj){
        std::unique_lock<std::mutex> lock(this->mutex);
        this->waiting_add++;
        while(this->live_add > 0 || this->live_release > 0 || this->live_get > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->waiting_add--;
        this->live_add++;

        // this->pool.push_back(std::make_shared<T>(obj));
        shared_pool<T>::add(obj);

        this->live_add--;
        if(this->waiting_add > 0 || this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
    }

    std::shared_ptr<T> acquire(){
        std::unique_lock<std::mutex> lock(this->mutex);
        while(this->waiting_add > 0 || this->live_add > 0 || this->waiting_release > 0 || this->live_release > 0
              || this->live_get > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->live_get++;

        if(this->pool.empty()){
            // rmr to release all locks
            this->live_get--;
            if(this->waiting_add > 0 || this->waiting_release > 0){
                this->pool_permission.notify_one();
            } else{
                this->pool_permission.notify_all(); // lot of others
            }
            lock.unlock();

            throw std::invalid_argument("there is an object at index in pool");
        }
        // T * ptr = this->pool[this->index++].get();
        std::shared_ptr<T> ptr = shared_pool<T>::acquire();

        this->live_get--;
        if(this->waiting_add > 0 || this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
        return *ptr;
    }

    std::shared_ptr<T> acquire(short index){
        std::unique_lock<std::mutex> lock(this->mutex);
        while(this->waiting_add > 0 || this->live_add > 0 || this->waiting_release > 0 || this->live_release > 0
              || this->live_get > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->live_get++;

        if(this->pool.empty() || index > this->pool.size()){
            // rmr to release all locks
            this->live_get--;
            if(this->waiting_add > 0 || this->waiting_release > 0){
                this->pool_permission.notify_one();
            } else{
                this->pool_permission.notify_all(); // lot of getters
            }
            lock.unlock();

            throw std::invalid_argument("there is no objects or not enough objects in the pool");
        }

        // T * ptr = this->pool[index].get();
        std::shared_ptr<T> ptr = shared_pool<T>::acquire(index);

        this->live_get--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
        return *ptr;
    }

    void release(std::shared_ptr<T> obj, short index){}

    void release(std::shared_ptr<T> obj){}
};

template <typename T>
class synch_unique_pool : public synch_pool<T>, public unique_pool<T>{
public:
    synch_unique_pool() : synch_pool<T>(), unique_pool<T>(){
        std::cout << "synch_unique_pool constructor" << std::endl;
    }

    ~synch_unique_pool(){
        std::cout << "synch_unique_pool destructor" << std::endl;
    }

    virtual std::ostream& print(std::ostream& os) {
        std::unique_lock<std::mutex> lock(this->mutex);
        if(this->waiting_add > 0 || this->live_add > 0 || this->waiting_release > 0  || this->live_release > 0 || this->live_get > 0){
            this->pool_permission.wait(lock);
        }
        this->live_print++;

        os << "synch_unique_pool" << std::endl;
        for(short a = 0; a < this->pool.size(); a++){
            os << *(this->pool[a].get()) << std::endl;
        }

        this->live_print--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of others
        }
        return os;
    }

    void add(std::shared_ptr<T> obj){
        std::unique_lock<std::mutex> lock(this->mutex);
        this->waiting_add++;
        while(this->live_add > 0 || this->live_release > 0 || this->live_get > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->waiting_add--;
        this->live_add++;

        // this->pool.push_back(make_unique<T>(obj));
        unique_pool<T>::add(obj);

        this->live_add--;
        if(this->waiting_add > 0 || this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
    }

    std::shared_ptr<T> acquire(){
        std::unique_lock<std::mutex> lock(this->mutex);
        while(this->waiting_add > 0 || this->live_add > 0 || this->waiting_release > 0 || this->live_release > 0
              || this->live_get > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->live_get++;

        if(this->pool.empty()){
            // rmr to release all locks
            this->live_get--;
            if(this->waiting_add > 0 || this->waiting_release > 0){
                this->pool_permission.notify_one();
            } else{
                this->pool_permission.notify_all(); // lot of others
            }
            lock.unlock();

            throw std::invalid_argument("there is an object at index in pool");
        }

        std::shared_ptr<T> ptr = std::move(this->pool[this->index++]);
        this->pool.erase(this->pool.begin() + this->index);
        // T ptr = unique_pool<T>::acquire();

        this->live_get--;
        if(this->waiting_add > 0 || this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
        return ptr;
    }

    std::shared_ptr<T> acquire(short index){
        std::unique_lock<std::mutex> lock(this->mutex);
        while(this->waiting_add > 0 || this->live_add > 0 || this->waiting_release > 0 || this->live_release > 0
              || this->live_get > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->live_get++;

        if(this->pool.empty() || index > this->pool.size()){
            // rmr to release all locks
            this->live_get--;
            if(this->waiting_add > 0 || this->waiting_release > 0){
                this->pool_permission.notify_one();
            } else{
                this->pool_permission.notify_all(); // lot of getters
            }
            lock.unlock();

            throw std::invalid_argument("there is no objects or not enough objects in the pool");
        }

        std::shared_ptr<T> ptr = std::move(this->pool[index++]);
        this->pool.erase(this->pool.begin() + index);
        // T ptr= unique_pool<T>::acquire(index);

        this->live_get--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
        return ptr;
    }

    void release(std::shared_ptr<T> obj, short index){
        std::unique_lock<std::mutex> lock(this->mutex);
        this->waiting_release++;
        while(this->waiting_add > 0 || this->live_add > 0 || this->live_release > 0 || this->live_get > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->waiting_release--;
        this->live_release++;

        if(this->pool[index] != NULL){
            this->live_release--;
            if(this->waiting_release > 0){
                this->pool_permission.notify_one();
            } else{
                this->pool_permission.notify_all(); // lot of getters
            }
            lock.unlock();

            throw std::invalid_argument("there is an object at index");
        }
        // this->pool.insert(this->pool.begin() + index, make_unique(*obj));
        unique_pool<T>::release(obj, index);

        this->live_release--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
    }

    void release(std::shared_ptr<T> obj){
        std::unique_lock<std::mutex> lock(this->mutex);
        this->waiting_release++;
        while(this->waiting_add > 0 || this->live_add > 0 || this->live_release > 0 || this->live_get > 0 || this->live_print > 0){
            this->pool_permission.wait(lock);
        }
        this->waiting_release--;
        this->live_release++;

        // this->pool.push_back(make_unique(*obj));
        unique_pool<T>::release(obj);

        this->live_release--;
        if(this->waiting_release > 0){
            this->pool_permission.notify_one();
        } else{
            this->pool_permission.notify_all(); // lot of getters
        }
        lock.unlock();
    }
};

#endif //PATH_PLANNING_SYNCH_POOL_H
