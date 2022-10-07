#include "trajectory_planner.h"
#include <Eigen/Dense>
#include <Eigen/LU>
using namespace Eigen;

TrajectoryPlanner::TrajectoryPlanner(double t_init, double t_fin, Vector4d motionGenBoundCond, Vector4d otherVarBoundCond)
{
    this->t_init = t_init;
    this->t_fin = t_fin;
    this->motionGenBoundCond = motionGenBoundCond;
    this->otherVarBoundCond = otherVarBoundCond;
}

    
void TrajectoryPlanner::thirdDegreePolynomial()
{
    // creating matrix
    auto t_init2 = t_init*t_init;
    auto t_init3 = t_init*t_init2;
    auto t_fin2 = t_fin*t_fin;
    auto t_fin3 = t_fin*t_fin2;
    Matrix4d A;
    A << 1, t_init, t_init2, t_init3,
         0,    1,   2*t_init, 3*t_init2,
         1, t_fin, t_fin2, t_fin3, 
         0,    1,   2*t_fin, 3*t_fin2;
    Matrix4d A_inv = A.inverse();
    polynCoeff = A_inv*motionGenBoundCond;
} 

double TrajectoryPlanner::getMotionGenerator(double time)
{
    double time2 = time*time;
    double time3 = time*time2;
    return polynCoeff(0) + polynCoeff(1)*time + polynCoeff(2)*time2 + polynCoeff(3)*time3; 

}
double TrajectoryPlanner::straightLine2D(double motionGen) // xz, xy, yz plane
{   double temp = (motionGen - motionGenBoundCond(0))/(motionGenBoundCond(2) - motionGenBoundCond(0));
    double q2 = temp*(otherVarBoundCond(1) - otherVarBoundCond(0)) + otherVarBoundCond(0);
    return q2;
}
std::tuple<double, double> TrajectoryPlanner::straightLine3D(double motionGen) 
{   double temp = (motionGen - motionGenBoundCond(0))/(motionGenBoundCond(2) - motionGenBoundCond(0));
    double q2 = temp*(otherVarBoundCond(1) - otherVarBoundCond(0)) + otherVarBoundCond(0);
    double q3 = temp*(otherVarBoundCond(3) - otherVarBoundCond(2)) + otherVarBoundCond(2);
    return std::make_tuple(q2, q3);
}

