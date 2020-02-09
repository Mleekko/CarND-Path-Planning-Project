//
// Created by Mleekko on 2020-02-09.
//

#include "../include/PathPlanner.h"
#include "../include/helpers.h"
#include <cmath>
#include <utility>
#include <iostream>

void PathPlanner::calculatePath() {
    pathX.clear();
    pathY.clear();

  // std::cout << "car: " << car->s << "," << car->d << std::endl;


    double dist_inc = 0.25;
    for (int i = 0; i < 200; ++i) {
        const vector<double> &xy = getXY(car->s + (dist_inc * i), car->d, mapS, mapX, mapY);

        // std::cout << "car`: " << xy[0] << "," << xy[1] << std::endl;

        pathX.push_back(xy[0]);
        pathY.push_back(xy[1]);

//        pathX.push_back(car->x + (dist_inc * i) * cos(deg2rad(car->yaw)));
//        pathY.push_back(car->y + (dist_inc * i) * sin(deg2rad(car->yaw)));
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

PathPlanner::PathPlanner(vector<double> mapX, vector<double> mapY, vector<double> mapS, vector<double> mapDx,
                         vector<double> mapDy) : mapX(std::move(mapX)), mapY(std::move(mapY)), mapS(std::move(mapS)),
                                                 mapDx(std::move(mapDx)), mapDy(std::move(mapDy)) {
    car = new Car();
}

const vector<double> &PathPlanner::getPathX() const {
    return pathX;
}

const vector<double> &PathPlanner::getPathY() const {
    return pathY;
}
