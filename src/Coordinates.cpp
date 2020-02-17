//
// Created by Mleekko on 2020-02-16.
//

#include "../include/Coordinates.h"
#include <cmath>

void Coordinates::toLocal(vector<double> &pointsX, vector<double> &pointsY, double refX, double refY, double refYaw) {
    double cosShift = cos(0 - refYaw);
    double sinShift = sin(0 - refYaw);

    for (size_t i = 0; i < pointsX.size(); i++) {
        double shiftX = pointsX[i] - refX;
        double shiftY = pointsY[i] - refY;

        pointsX[i] = shiftX * cosShift - shiftY * sinShift;
        pointsY[i] = shiftX * sinShift + shiftY * cosShift;
    }
}

void Coordinates::toWorld(vector<double> &pointsX, vector<double> &pointsY, double refX, double refY, double refYaw) {
    double cosYaw = cos(refYaw);
    double sinYaw = sin(refYaw);

    for (size_t i = 0; i < pointsX.size(); i++) {
        double x = pointsX[i];
        double y = pointsY[i];

        pointsX[i] = refX + (x * cosYaw - y * sinYaw);
        pointsY[i] = refY + (x * sinYaw + y * cosYaw);
    }
}
