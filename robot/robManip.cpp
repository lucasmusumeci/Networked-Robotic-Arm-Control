#include "robManip.hpp"
#include <cmath>

using namespace Eigen;

/*
 * If there is a tool attached to the end-effector, it is represented by the transformation matrix T_tool.
 * If there is no tool, T_tool should be set to the identity matrix.
 */

std::pair<Matrix4d, std::vector<Matrix4d>> Robot::MGD() {
    int N = static_cast<int>(alpha.size());
    Matrix4d T = Matrix4d::Identity();
    std::vector<Matrix4d> MT;

    for (int i = 0; i < N; ++i) {
        double ctheta_i = std::cos(theta(i)+offset_theta(i));
        double stheta_i = std::sin(theta(i)+offset_theta(i));
        double calpha_i = std::cos(alpha(i));
        double salpha_i = std::sin(alpha(i));
        double d_i = d(i);
        double r_i = r(i);

        Matrix4d Ti;
        Ti << ctheta_i, -stheta_i, 0.0, d_i,
              stheta_i*calpha_i, ctheta_i*calpha_i, -salpha_i, -r_i*salpha_i,
              stheta_i*salpha_i, ctheta_i*salpha_i,  calpha_i,  r_i*calpha_i,
              0.0, 0.0, 0.0, 1.0;

        T = T * Ti;
        MT.push_back(T);
    }

    if (T_tool != Matrix4d::Identity()) {
        T = T * T_tool;
        MT.push_back(T);
    }

    return {T, MT};
}

MatrixXd Robot::Jacobienne(const Vector3d& P) {
    int N = static_cast<int>(alpha.size());
    MatrixXd J = MatrixXd::Zero(6, N);
    Matrix4d T = Matrix4d::Identity();

    for (int i = 0; i < N; ++i) {
        double ctheta_i = std::cos(theta(i)+offset_theta(i));
        double stheta_i = std::sin(theta(i)+offset_theta(i));
        double calpha_i = std::cos(alpha(i));
        double salpha_i = std::sin(alpha(i));
        
        Matrix4d Ti;
        Ti << ctheta_i, -stheta_i, 0.0, d(i),
              stheta_i*calpha_i, ctheta_i*calpha_i, -salpha_i, -r(i)*salpha_i,
              stheta_i*salpha_i, ctheta_i*salpha_i,  calpha_i,  r(i)*calpha_i,
              0.0, 0.0, 0.0, 1.0;

        T = T * Ti;
        
        // Extract z-axis (3rd column) and origin (4th column)
        Vector3d z = T.block<3,1>(0, 2);
        Vector3d o = T.block<3,1>(0, 3);
        
        // Linear velocity part (cross product)
        J.block<3,1>(0, i) = z.cross(P - o);
        // Angular velocity part
        J.block<3,1>(3, i) = z;

    }
    return J;
}

/*
 * Create a Robotis-H robot with the given DH parameters
 */
Robot CreateRobotisH(const Eigen::VectorXd& initialTheta)
{
    Robot robot;
    const double pi = M_PI;
    int dof = 6;

    Eigen::VectorXd offset_theta(dof); offset_theta << 0.0, -1.4576453, -0.898549163, 0.0, 0.0, 0.0;
    Eigen::Matrix4d T_tool;
    T_tool << -1.0, 0.0, 0.0, 0.0,
               0.0,-1.0, 0.0, 0.0,
               0.0, 0.0, 1.0, 0.235,
               0.0, 0.0, 0.0, 1.0;
    Eigen::VectorXd a(dof); a << 0.0, -pi/2.0, 0.0, -pi/2.0, pi/2.0, -pi/2.0;
    Eigen::VectorXd d(dof); d << 0.0, 0.0, 0.26569, 0.03, 0.0, 0.0;
    Eigen::VectorXd r(dof); r << 0.159, 0.0, 0.0, 0.258, 0.0, 0.0;
    
    // Not used in this project
    Eigen::VectorXd m(dof); m.setZero();
    Eigen::MatrixXd c = Eigen::MatrixXd::Zero(4, dof);


    robot.setTheta(initialTheta);
    robot.setTTool(T_tool);
    robot.setOffsetTheta(offset_theta);
    robot.setAlpha(a);
    robot.setD(d);
    robot.setR(r);
    robot.setM(m);
    robot.setC(c);

    return robot;
}

/*
 * Rotation conversions
 */

Matrix3d RTL2R(double phi, double theta, double psi) {
    Matrix3d A;
    double cp = std::cos(phi), sp = std::sin(phi);
    double ct = std::cos(theta), st = std::sin(theta);
    double cs = std::cos(psi), ss = std::sin(psi);

    A << ct*cp, -sp*cs + cp*st*ss,  ss*sp + cp*st*cs,
         ct*sp,  cp*cs + st*ss*sp, -cp*ss + st*sp*cs,
         -st,    ct*ss,             ct*cs;
    return A;
}

Vector3d R2RTL(const Matrix3d& R) {
    // Porting Julia's 1-based R[3,1] to 0-based R(2,0)
    double theta = -std::asin(R(2, 0));
    double phi   = std::atan2(R(1, 0), R(0, 0));
    double psi   = std::atan2(R(2, 1), R(2, 2));
    return Vector3d(phi, theta, psi);
}