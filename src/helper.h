#ifndef HELPER_H
#define HELPER_H



class Helper {
public:

	static double distance(double x1, double y1, double x2, double y2);
	static int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
	static int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
	static vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
	static vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
	static constexpr double pi();
	static double deg2rad(double x);
	static double rad2deg(double x);
};




#endif