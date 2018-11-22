#include <algorithm>
#include <iostream>
#include "behavior_planner.h"
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"

static const int SPEED_LIMIT = 48;
static const int NUM_LANES = 3;
static const int ROAD_LANE_WIDTH_METERS = 4;
static const int ROAD_WIDTH_METERS = NUM_LANES * ROAD_LANE_WIDTH_METERS;
static const int GOAL_LANE = 1;
static const int MAX_ACCEL = 2;
static const int EGO_KEY = -1;
static const float MAX_WAYPOINT_S = 6914.14941f;

BehaviorPlanner::BehaviorPlanner() {}

int BehaviorPlanner::d_to_lane(double d)
{
	return abs((int)(trunc(d / ROAD_LANE_WIDTH_METERS) - 2));
}

double BehaviorPlanner::lane_to_d(int lane)
{
	return ROAD_WIDTH_METERS - (lane * ROAD_LANE_WIDTH_METERS + ROAD_LANE_WIDTH_METERS / 2);
}

void BehaviorPlanner::choose_next_state(double car_s, double car_d, double car_speed, vector<vector<double>> sensor_fusion)
{

	// calculate the goal s value
	const double goal_s = car_s + 30.0;

	//cout << "car_s=" << car_s << "\n";

	const int ego_lane = d_to_lane(car_d);

	if (this->ego.initialized())
	{
		this->ego.update_state(ego_lane, car_s, car_speed);
		this->ego.goal_s = goal_s;
	}
	else
	{
		vector<int> ego_config = { SPEED_LIMIT,NUM_LANES, MAX_ACCEL };
		this->ego = Vehicle(EGO_KEY, ego_lane, car_s, car_speed, 0, VehicleState::KeepLane);
		this->ego.configure(ego_config);
		this->ego.goal_s = goal_s;
	}

	vector<int> vehicles_updated;

	// find the ref_v to use.
	for (auto detected_vehicle : sensor_fusion)
	{
		int veh_id = (int)detected_vehicle[0];
		double veh_d = detected_vehicle[6];
		double veh_s = detected_vehicle[5];
		double veh_vx = detected_vehicle[3];
		double veh_vy = detected_vehicle[4];
		double veh_v = sqrt(veh_vx * veh_vx + veh_vy * veh_vy);


		//cout << "veh_s=" << veh_s << "\n";
		if (veh_d < ROAD_WIDTH_METERS)
		{
			int veh_lane = abs((int) trunc(veh_d / ROAD_LANE_WIDTH_METERS) - 2);
			//cout << "d=" << veh_d << "; lane=" << veh_lane << "\n";

			auto veh_it = vehicles.find(veh_id);
			if (veh_it == vehicles.end())
			{
				Vehicle vehicle = Vehicle(veh_id, veh_lane, veh_s, veh_v, 0);
				vehicles.insert(std::pair<int, Vehicle>(veh_id, vehicle));
				vehicles_updated.push_back(veh_id);
			}
			else
			{
				veh_it->second.update_state(veh_lane, veh_s, veh_v);;
				vehicles_updated.push_back(veh_id);
			}
		}
	}

	map<int, vector<Vehicle>> predictions;
	if (this->vehicles.size() > 0)
	{
		for (auto &kv : this->vehicles)
		{
			int target_id = kv.first;

			int sensor_data_included_vehicle = false;
			for (auto veh_id : vehicles_updated)
			{
				if (veh_id == target_id)
				{
					sensor_data_included_vehicle = true;
					vector<Vehicle> preds = kv.second.generate_predictions();
					predictions[target_id] = preds;
					break;
				}
			}

			if (!sensor_data_included_vehicle)
			{
				this->vehicles.erase(target_id);
			}
		}
	}

	auto trajectory = ego.choose_next_state(predictions);


	this->ego.state = trajectory[1].state;
	this->ego.goal_lane = trajectory[1].lane;
	if (trajectory[1].s > MAX_WAYPOINT_S)
		this->ego.goal_s = trajectory[1].s - MAX_WAYPOINT_S;
	else
		this->ego.goal_s = trajectory[1].s;

	//cout << "lane=" << trajectory[1].lane << "; state[1]=" << trajectory[1].state << "; vel=" << trajectory[1].v << "\n";

	this->lane = trajectory[1].lane;
	this->v = trajectory[1].v;
}