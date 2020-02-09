//
// Created by Mleekko on 2020-02-09.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include "Car.h"
#include <vector>

using std::vector;


class PathPlanner {
public:
    PathPlanner();

    void calculatePath();

    void updateCar(double x, double y, double s, double d, double yaw, double speed);

    const vector<double> &getPathX() const;

    const vector<double> &getPathY() const;

private:
    Car *car;
    vector<double> pathX;
    vector<double> pathY;
};


#endif //PATH_PLANNING_PATHPLANNER_H
