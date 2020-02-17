//
// Created by Mleekko on 2020-02-09.
//

#include "../include/PathPlanner.h"
#include "../include/helpers.h"
#include "../include/Coordinates.h"
#include <cmath>
#include <utility>
#include <iostream>
#include  "../include/spline/spline.h"


PathPlanner::PathPlanner(vector<double> mapX, vector<double> mapY, vector<double> mapS, vector<double> mapDx,
                         vector<double> mapDy) : mapX(std::move(mapX)), mapY(std::move(mapY)), mapS(std::move(mapS)),
                                                 mapDx(std::move(mapDx)), mapDy(std::move(mapDy)) {
    car = new Car();
}

inline void ToLocalCoordinates(
        vector<double> &x_points, vector<double> &y_points, double refX, double refY, double ref_yaw) {
    for (int i = 0; i < x_points.size(); i++) {
        double shift_x = x_points[i] - refX;
        double shift_y = y_points[i] - refY;

        x_points[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
        y_points[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
    }
}

void PathPlanner::calculatePath() {
    Coordinates c;

    // TODO: should this be desired lane?
    int currentLane = 1;
    // Reference velocity (mph)
    double ref_vel = 45.;

    unsigned int pathSize = prevPathX.size();
    double refS = pathSize > 0 ? prevS : car->s;
    double refD = pathSize > 0 ? prevD : car->d;
    double refX = car->x;
    double refY = car->y;
    double refYaw = deg2rad(car->yaw);

    // std::cout << "car: " << car->s << "," << car->d << std::endl;

    vector<double> splinePointsX;
    vector<double> splinePointsY;

//     std::cout << "pathSize: " << pathSize << std::endl;
    // decide the previous point
    if (pathSize < 2){ // infer the previous point based on car direction
        double prevCarX = refX - cos(refYaw);
        double prevCarY = refY - sin(refYaw);
        splinePointsX.push_back(prevCarX);
        splinePointsY.push_back(prevCarY);
    } else { // use last 2 points from previous path
        refX = prevPathX[pathSize - 1];
        refY = prevPathY[pathSize - 1];
        double prevCarX = prevPathX[pathSize - 2];
        double prevCarY = prevPathY[pathSize - 2];
        refYaw = atan2(refY - prevCarY, refX - prevCarX);

        splinePointsX.push_back(prevCarX);
        splinePointsY.push_back(prevCarY);

        vector<double> sd = getFrenet(refX, refY, refYaw, mapX, mapY);
        refS = sd[0];
        refD = sd[1];
    }

    // push the current car location
    splinePointsX.push_back(refX);
    splinePointsY.push_back(refY);

    // push projections in 3 points 23 meters apart, meters
    for (int i=1; i<4; i++) {
        vector<double> nextWayPoint = getXY(refS + 25 * i, (2 + 4 * currentLane), mapS, mapX, mapY);
        splinePointsX.push_back(nextWayPoint[0]);
        splinePointsY.push_back(nextWayPoint[1]);
    }

    c.toLocal(splinePointsX, splinePointsY, refX, refY, refYaw);

    tk::spline s;
    s.set_points(splinePointsX, splinePointsY);

    double targetX = 30.0;
    double targetY = s(targetX);
    double target_dist = sqrt(targetX * targetX + targetY * targetY);

    // convert mph into meters
    double N = target_dist / (0.02 * ref_vel / 2.24);

    vector<double> newPathX;
    vector<double> newPathY;

    double diffX = 0.;
    // fill the missing points
    for (int i = 0; i < PATH_POINTS - pathSize; i++)    {
        double x = diffX + targetX / N;
        double y = s(x);

        diffX = x;

        newPathX.push_back(x);
        newPathY.push_back(y);
    }

    c.toWorld(newPathX, newPathY, refX, refY, refYaw);

    // add the missing points to the path
    pathX.insert(pathX.end(), newPathX.begin(), newPathX.end());
    pathY.insert(pathY.end(), newPathY.begin(), newPathY.end());

}

void PathPlanner::updateCar(double x, double y, double s, double d, double yaw, double speed) {
    car->x = x;
    car->y = y;
    car->s = s;
    car->d = d;
    car->yaw = yaw;
    car->speed = speed;
}

void PathPlanner::updatePrevPath(vector<double> &prevPathX, vector<double> &prevPathY, double s, double d) {
    this->prevPathX = prevPathX;
    this->prevPathY = prevPathY;
    this->prevS = s;
    this->prevD = d;
}


const vector<double> &PathPlanner::getPathX() const {
    return pathX;
}

const vector<double> &PathPlanner::getPathY() const {
    return pathY;
}

void PathPlanner::reset() {
    pathX.clear();
    pathY.clear();
}

