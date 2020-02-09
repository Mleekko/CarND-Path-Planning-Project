//
// Created by Mleekko on 2020-02-09.
//

#include "../include/PathPlanner.h"
#include "../include/helpers.h"
#include <cmath>

void PathPlanner::calculatePath() {
    pathX.clear();
    pathY.clear();


    double dist_inc = 0.25;
    for (int i = 0; i < 50; ++i) {
        pathX.push_back(car->x + (dist_inc * i) * cos(deg2rad(car->yaw)));
        pathY.push_back(car->y + (dist_inc * i) * sin(deg2rad(car->yaw)));
    }

}

void PathPlanner::updateCar(double x, double y, double s, double d, double yaw, double speed) {
    car->x = x;
    car->y = y;
    car->s = s;
    car->d = d;
    car->yaw = yaw;
    car->speed = speed;
}

PathPlanner::PathPlanner() {
    car = new Car();
}

const vector<double> &PathPlanner::getPathX() const {
    return pathX;
}

const vector<double> &PathPlanner::getPathY() const {
    return pathY;
}
