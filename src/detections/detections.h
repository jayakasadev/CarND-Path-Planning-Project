#ifndef PATH_PLANNING_DETECTIONS_H
#define PATH_PLANNING_DETECTIONS_H

#include <vector>
#include "../vehicle/traffic.h"
#include "../utilities/read_write_lock.h"

class detections {
private:
    read_write_lock lock; // used to allow thread safe activity
    std::vector<traffic *> detected;
    short itr = 0;

public:

    detections(){}

    ~detections(){}

    /**
     * Method adds vehicle to storage
     *
     * @param vehicle
     */
    void add(traffic &vehicle);

    /**
     * Method to iterate all detections
     * @return traffic object
     */
    traffic * iterateDetections();

    /**
     * Method clears the collection
     */
    void reset();

    /**
     * Prints out the detections
     */
    void print();
};


#endif //PATH_PLANNING_DETECTIONS_H
