#ifndef PATH_PLANNING_READ_WRITE_LOCK_H
#define PATH_PLANNING_READ_WRITE_LOCK_H

#include <mutex>
#include <condition_variable>

class read_write_lock{
private:
    std::mutex mutex;
    std::condition_variable write_permission;
    std::condition_variable read_permission;
    short live_readers;
    short live_writers;
    short waiting_writers;
public:
    read_write_lock(){
        live_writers = 0;
        live_readers = 0;
        waiting_writers = 0;
    }
    ~read_write_lock(){}

    void read_lock(){
        std::unique_lock<std::mutex> lock(mutex);
        while(waiting_writers > 0){ // writes given preference
            read_permission.wait(lock); // wait until there are no other writers waiting
        }
        ++live_readers;
        lock.unlock();
    }

    void read_unlock(){
        std::unique_lock<std::mutex> lock(mutex);
        --live_readers;
        lock.unlock();
        write_permission.notify_one(); // notify the first waiting thread
    }

    void write_lock(){
        std::unique_lock<std::mutex> lock(mutex);
        ++waiting_writers;
        while(live_readers > 0 || live_writers > 0){ // do not want to lock in the middle of another read or write operation
            write_permission.wait(lock); // wait until i can read
        }
        ++live_writers; // actively writing
        lock.unlock();
    }

    void write_unlock(){
        std::unique_lock<std::mutex> lock(mutex);
        --waiting_writers; // no longer waiting, done here to keep another writer thread from acquiring the write lock at the same time by accident
        --live_writers; // no longer writing
        if(waiting_writers > 0){
            write_permission.notify_one(); // notify the next writer
        }
        else{
            read_permission.notify_all(); // notify all readers
        }
        lock.unlock();
    }
};

#endif //PATH_PLANNING_READ_WRITE_LOCK_H
