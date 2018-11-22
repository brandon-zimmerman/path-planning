#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

enum VehicleState
{
	ConstantSpeed,
	KeepLane,
	PrepareLaneChangeLeft,
	PrepareLaneChangeRight,
	LaneChangeLeft,
	LaneChangeRight,
};

std::ostream & operator << (std::ostream &os, const VehicleState &state);

class Vehicle {
public:	

	map<VehicleState, int> lane_direction = { {VehicleState::PrepareLaneChangeLeft, 1}, {VehicleState::LaneChangeLeft, 1}, {VehicleState::LaneChangeRight, -1}, {VehicleState::PrepareLaneChangeRight, -1} };

	int id;
	int lane;
	double s;
	double v;
	double a;
	int goal_lane;
	double goal_s;
	double target_speed;
	VehicleState state;

	/**
	* Constructor
	*/
	Vehicle();
	Vehicle(int id, int lane, double s, double v, double a, VehicleState state = VehicleState::ConstantSpeed);

	/**
	* Destructor
	*/
	virtual ~Vehicle();

	void update_state(int lane, double s, double v);

	vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);

	vector<VehicleState> successor_states();

	vector<Vehicle> generate_trajectory(VehicleState state, map<int, vector<Vehicle>> predictions);

	vector<double> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);

	vector<Vehicle> constant_speed_trajectory();

	vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

	vector<Vehicle> lane_change_trajectory(VehicleState state, map<int, vector<Vehicle>> predictions);

	vector<Vehicle> prep_lane_change_trajectory(VehicleState state, map<int, vector<Vehicle>> predictions);	

	double position_at(float t);

	bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

	bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

	vector<Vehicle> generate_predictions(int horizon = 2);

	void realize_next_state(vector<Vehicle> trajectory);

	void configure(vector<int> road_data);

	bool initialized();

private:	
	bool is_initialized;
	float t = 0.2f;
	int preferred_buffer = 10; // impacts "keep lane" behavior.
	int change_lanes_buffer = 20; 	
	int lanes_available;
	double max_acceleration = 0.2f;

};

#endif