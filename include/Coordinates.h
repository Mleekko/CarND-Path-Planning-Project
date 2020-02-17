//
// Created by Mleekko on 2020-02-16.
//

#ifndef PATH_PLANNING_COORDINATES_H
#define PATH_PLANNING_COORDINATES_H

#include <vector>

using std::vector;

class Coordinates {
public:
    void toLocal(vector<double> &pointsX, vector<double> &pointsY, double refX, double refY, double refYaw);
    void toWorld(vector<double> &pointsX, vector<double> &pointsY, double refX, double refY, double refYaw);

};


#endif //PATH_PLANNING_COORDINATES_H
