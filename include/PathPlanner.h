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

    PathPlanner(vector<double> mapX, vector<double> mapY, vector<double> mapS,
                vector<double> mapDx, vector<double> mapDy);

    void calculatePath(const vector<Car> &traffic);

    void updateCar(double x, double y, double s, double d, double yaw, double speed);

    void updatePrevPath(vector<double> &prevPathX, vector<double> &prevPathY, double prevS, double prevD);

    void reset();

    const vector<double> &getPathX() const;

    const vector<double> &getPathY() const;

private:
    Car *car;
    int targetLane;

    vector<double> newPathX;
    vector<double> newPathY;

    vector<double> splinePointsX;
    vector<double> splinePointsY;

    vector<double> prevPathX;
    vector<double> prevPathY;
    double prevS;
    double prevD;

    vector<double> mapX;
    vector<double> mapY;
    vector<double> mapS;
    vector<double> mapDx;
    vector<double> mapDy;

};


#endif //PATH_PLANNING_PATHPLANNER_H
