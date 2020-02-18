//
// Created by Mleekko on 2020-02-09.
//

#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H


class Car {
public:
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;

    int getCurrentLane();

    Car(double x, double y, double s, double d, double speed);

    Car();
};


#endif //PATH_PLANNING_CAR_H
