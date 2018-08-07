#include "planning.h"
#include "spline.h"
/*
*Constructor
*/
Planner::Planner(){
 std::cout << "constructor " << std::endl;
}

Planner::~Planner() {}

double Planner::CarS(vector<vector<double>> sensor_fusion, int prev_size, int i){
                // Find car speed.
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];
                // Estimate car s position after executing previous trajectory.
                check_car_s = ((double)prev_size*0.02*check_speed);

    return check_car_s;
};

void Planner::StateCar(double ref_vel, bool car_ahead, bool car_righ, bool car_left, int &lane, double &speed_diff) {
 
           if ( car_ahead ) { 
              if ( !car_left && lane > 0 ) {
                // check if there is no car left and there is a left lane.
                lane--; 
              } else if ( !car_righ && lane != 2 ){
                // if there is no car right and there is a right lane.
                lane++; 
              } else {
                speed_diff -= accel;
              }
            } else {
              if ( lane != 1 ) { // if we are not on the center lane.
                if ( ( lane == 0 && !car_righ ) || ( lane == 2 && !car_left ) ) {
                  lane = 1; // Back to center.
                }
              }
              if ( ref_vel < speed ) {
                speed_diff += accel;
              }
            }
};


