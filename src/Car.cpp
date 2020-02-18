//
// Created by Mleekko on 2020-02-09.
//

#include "../include/Car.h"
#include "../include/Constants.h"

int Car::getCurrentLane() {
    return (int) (this->d / LANE_WIDTH);
}

Car::Car() {}

Car::Car(double x, double y, double s, double d, double speed) : x(x), y(y), s(s), d(d), speed(speed) {}
