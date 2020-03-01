//
// Created by Mleekko on 2020-02-09.
//

#include "../include/PathPlanner.h"
#include "../include/helpers.h"
#include "../include/Coordinates.h"
#include "../include/Constants.h"
#include <cmath>
#include <utility>
#include <cfloat>
#include  "../include/spline/spline.h"


PathPlanner::PathPlanner(vector<double> mapX, vector<double> mapY, vector<double> mapS, vector<double> mapDx,
                         vector<double> mapDy) : mapX(std::move(mapX)), mapY(std::move(mapY)), mapS(std::move(mapS)),
                                                 mapDx(std::move(mapDx)), mapDy(std::move(mapDy)) {
    car = new Car();
    this->prevS = 0.;
    this->prevD = 0.;
    this->targetLane = 1;
}

void PathPlanner::calculatePath(const vector<Car>& traffic) {
    Coordinates c;

    int currentLane = car->getCurrentLane();

    unsigned int pathSize = prevPathX.size();
    double refS = pathSize > 0 ? prevS : car->s;
    double refD = pathSize > 0 ? prevD : car->d;
    double refX = car->x;
    double refY = car->y;
    double refYaw = deg2rad(car->yaw);

    // just get the free lane or the lane with the farther collision distance
    double collisionTimes[3] = {DBL_MAX, DBL_MAX, DBL_MAX};
    double nextCarDistances[3] = {DBL_MAX, DBL_MAX, DBL_MAX};
    double nextCarSpeeds[3] = {DBL_MAX, DBL_MAX, DBL_MAX};

    for (auto other : traffic) {
        double distance = other.s - car->s;
        int lane = other.getCurrentLane();
        if (distance > 0 && nextCarDistances[lane] > distance) {
            nextCarDistances[lane] = distance;
            nextCarSpeeds[lane] = other.speed / MILES_PER_HOUR_2_METERS_PER_SECOND;
        }
        if (distance > -8 && distance < 10) {
            // can't use the lane - car is too close
            collisionTimes[lane] = 0;
        } else if (distance >= 10 && distance < 80) {
            double time = distance / (car->speed - other.speed);
            if (time > 0) {
                if (collisionTimes[lane] > time) {
                    collisionTimes[lane] = time;
                }
            }
        }
    }

    // keep the lane if we have nowhere to go.
    if (nextCarDistances[this->targetLane] < 65) {
        auto max = DBL_MIN;
        for (int i = 0; i < 3; i++) {
            if (std::abs(currentLane - i) > 1) { // change lanes one at a time
                continue;
            }
            if (collisionTimes[i] > max) {
                max = collisionTimes[i];
                if (max > 0) { // yes, this is correct
                    this->targetLane = i;
                }
            }
        }
    }

    bool brakeAllowed = false;
    double targetVelocity = MAX_SPEED;
    if (nextCarDistances[this->targetLane] < 18) {
        brakeAllowed = true;
        targetVelocity = nextCarSpeeds[this->targetLane];
    }
    if (this->targetLane != currentLane && nextCarDistances[currentLane] < 18) {
        brakeAllowed = true;
        targetVelocity = std::min(targetVelocity, nextCarSpeeds[currentLane]);
    }

    this->splinePointsX.clear();
    this->splinePointsY.clear();

    double velocity;
//     std::cout << "pathSize: " << pathSize << std::endl;
    // decide the previous point
    if (pathSize < 2) { // infer the previous point based on car direction
        double prevCarX = refX - cos(refYaw);
        double prevCarY = refY - sin(refYaw);
        splinePointsX.push_back(prevCarX);
        splinePointsY.push_back(prevCarY);
        // need to convert
        velocity = car->speed * MILES_PER_HOUR_2_METERS_PER_SECOND;
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

        velocity = distance(refX, refY, prevCarX, prevCarY) / MILES_PER_HOUR_2_METERS_PER_SECOND / TICK_INTERVAL;
    }

    bool keepLane = this->targetLane == currentLane;

    // push the current car location
    splinePointsX.push_back(refX);
    splinePointsY.push_back(refY);

    // push projections in 2 points 40 meters apart
    for (int i = 1; i < 3; i++) {
        vector<double> nextWayPoint = getXY(refS + 40 * i, (LANE_WIDTH / 2 + LANE_WIDTH * this->targetLane),
                                            mapS, mapX, mapY);
        splinePointsX.push_back(nextWayPoint[0]);
        splinePointsY.push_back(nextWayPoint[1]);
    }

    c.toLocal(splinePointsX, splinePointsY, refX, refY, refYaw);

    tk::spline s;
    s.set_points(splinePointsX, splinePointsY);

    double targetX = 30.0;
    double targetY = s(targetX);
    double target_dist = sqrt(targetX * targetX + targetY * targetY);

    newPathX.clear();
    newPathY.clear();

    double diffX = 0.;

    // fill the missing points
    for (int i = 0; i < PATH_POINTS - pathSize; i++) {
        // reduce jerk when changing lane
        double accelerationFactor = 1.;
        if (!keepLane) {
            double diff = std::abs(s(diffX) - targetY);
            accelerationFactor = (LANE_WIDTH - diff) / LANE_WIDTH * 0.65;
        }

        // fix the speed
        double step_v = MAX_ACCELERATION * accelerationFactor * TICK_INTERVAL ;
        if (velocity < targetVelocity) {
            velocity += step_v;
            velocity = std::min(targetVelocity, velocity);
        } else if (velocity > targetVelocity) {
            if (brakeAllowed) {
                step_v *= 1.5;
            }
            velocity -= step_v;
            velocity = std::max(targetVelocity, velocity);
        }

        double N = target_dist / (TICK_INTERVAL * velocity / 2.24);
        double x = diffX + targetX / N;
        double y = s(x);

        diffX = x;

        newPathX.push_back(x);
        newPathY.push_back(y);
    }
 //   std::cout << "velocity END: " << velocity << std::endl;


    c.toWorld(newPathX, newPathY, refX, refY, refYaw);

    // add the missing points to the path
    prevPathX.insert(prevPathX.end(), newPathX.begin(), newPathX.end());
    prevPathY.insert(prevPathY.end(), newPathY.begin(), newPathY.end());

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
    return prevPathX;
}

const vector<double> &PathPlanner::getPathY() const {
    return prevPathY;
}

void PathPlanner::reset() {
    prevPathX.clear();
    prevPathY.clear();
    car = new Car();
    this->targetLane = 1;
}

