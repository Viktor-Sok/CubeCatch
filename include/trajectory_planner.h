#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include <Eigen/Dense>
using namespace Eigen;

class TrajectoryPlanner
{
private:
    Vector4d polynCoeff; //coefficients of the quibic polynomial q(t) = a0+a1*t+a2*t^2+a3*t^3
    Vector4d motionGenBoundCond; // (q1_i, v1_i, q1_f, v1_f) : q - coordinate, v - speed
    Vector4d otherVarBoundCond; // (q2_i, q2_f, q3_i, q3_f)
    //Vector4d gripperZaxisBoundCond; // (a_i, w_i, a_f, w_f ) : a - angle, w - angular velocity
    double t_init;
    
    
public:
    double t_fin;
      

public:
    TrajectoryPlanner() = delete;
    // motionGenInitCond
    TrajectoryPlanner(double t_init, double t_fin, Vector4d motionGenBoundCond, Vector4d otherVarBoundCond);
    double getMotionGenerator(double time); // chromo time
    void thirdDegreePolynomial();
    double straightLine2D(double motionGen);
    std::tuple<double, double> straightLine3D(double motionGen);
    // function to generate orientation (Euler angles)
};

#endif /* TRAJECTORY_PLANNER_H */