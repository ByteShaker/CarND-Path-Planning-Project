//
// Created by Friedrich Schweizer on 29.07.17.
//

#ifndef PATH_PLANNING_PATH_PLANNER_H
#define PATH_PLANNING_PATH_PLANNER_H

#include <iostream>
#include <math.h>
#include <vector>

#include "json.hpp"

// for convenience
using json = nlohmann::json;

using namespace std;

class PP {
public:

    /**
     * Constructor
     */
     PP();

    /**
     * Destructor
     */
     virtual ~PP();

    /** Predict behaviour of other cars */

    void predict(json sensor_fusion);
};


#endif //PATH_PLANNING_PATH_PLANNER_H
