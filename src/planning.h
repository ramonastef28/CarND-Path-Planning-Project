#ifndef Planning_H_
#define Planning_H_

#include <vector>
#include <string>

using namespace std;

const double speed = 49.5;
const double accel = .224;

class Planner{

  public:
    /*
    **Constructer
    */
    Planner();
    
    /*
    **Deconstructer
    */
    virtual ~Planner();

    void LaneCar(vector<vector<double>> sensor_fusion, int prev_size, int lane, double car_s, bool &car_ahead, bool &car_left, bool &car_right);     

    void StateCar(double ref_vel, bool car_ahead, bool car_right, bool car_left, int &lane, double &speed_diff);

    double CarS(vector<vector<double>> sensor_fusion, int prev_size, int i);
    //void DriveCar(vector<double> ptsx, vector<double> ptsy, vector<double> &next_x_vals, vector<double> &next_y_vals, int prev_size, double rel_vel);
    void DriveCar(vector<double> ptsx, vector<double> ptsy, vector<double> &next_x_vals, vector<double> &next_y_vals, int prev_size, double ref_vel, double speed_diff,double ref_x, double ref_y, double ref_yaw);
};
#endif /*Planning_H_ */





