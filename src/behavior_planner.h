#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"

using namespace std;

class BehaviorPlanner {
public:

	BehaviorPlanner();
	static int d_to_lane(double d);	
	static double lane_to_d(int lane);

	int lane;
	double v;
	void choose_next_state(double car_s, double car_d, double car_speed, vector<vector<double>> sensor_fusion);

private:

	Vehicle ego;
	map<int, Vehicle> vehicles;		
};

	


#endif
