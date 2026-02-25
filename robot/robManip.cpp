#include "robManip.hpp"
#include <cmath>

using namespace Eigen;

/*
 * Hat operator for 3D vectors
 */

Matrix3d Robot::hat(const Vector3d& v) {
    Matrix3d S;
    S <<  0.0,  -v(2),  v(1),
          v(2),  0.0,  -v(0),
         -v(1),  v(0),  0.0;
    return S;
}

/*
 * Compute L matrix used in MCI
 */
Matrix3d Robot::calcul_L(const Matrix3d& Ad, const Matrix3d& Ae) {
    return -0.5 * (hat(Ae.col(0)) * hat(Ad.col(0))
                 + hat(Ae.col(1)) * hat(Ad.col(1))
                 + hat(Ae.col(2)) * hat(Ad.col(2)));
}


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

    if (!T_tool.isApprox(Matrix4d::Identity())) {
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
 * Command Trapeze
 */

Trapeze Robot::calculTrapeze(const VectorXd& qi,
                              const VectorXd& qf,
                              double duree) const {
    int N = getN();
    VectorXd dq = qf - qi;

    // Compute per-joint tf and find the slowest joint
    VectorXd tf_calc(N);
    for (int i = 0; i < N; ++i)
        tf_calc(i) = std::abs(dq(i)) / V_max(i) + V_max(i) / a_max(i);

    int i_max;
    double tf = tf_calc.maxCoeff(&i_max);
    double t1 = V_max(i_max) / a_max(i_max);
    double t2 = tf - t1;

    // If the slowest joint produces a triangle profile, recalculate
    if (t1 > std::abs(dq(i_max)) / V_max(i_max)) {
        while (true) {
            tf_calc.maxCoeff(&i_max);

            double ai_cur  = dq(i_max) / (t1 * t2);
            double vi_cur  = dq(i_max) / t2;

            if (std::abs(vi_cur) / std::abs(ai_cur) >
                std::abs(dq(i_max)) / std::abs(vi_cur)) {
                // Triangle case — recompute tf for this joint
                tf = 2.0 * std::sqrt(std::abs(dq(i_max)) / a_max(i_max));
                tf_calc(i_max) = tf;
                t1 = tf / 2.0;
                t2 = t1;

                int new_max;
                tf_calc.maxCoeff(&new_max);
                if (new_max == i_max) break;  // still the slowest, done
            } else {
                break;
            }
        }
    }

    // Stretch to requested duration if needed
    if (duree > tf) {
        t1 = (t1 * duree) / tf;
        t2 = duree - t1;
        tf = duree;
    }

    // Final per-joint velocities and accelerations
    VectorXd Vi_max(N), ai(N);
    for (int i = 0; i < N; ++i) {
        Vi_max(i) = dq(i) / t2;
        ai(i)     = dq(i) / (t2 * t1);
    }

    return {t1, t2, tf, Vi_max, ai};
}

MatrixXd Robot::calculQ(const VectorXd& qi,
                         const Trapeze& trap,
                         const VectorXd& t) const {
    int N  = getN();
    int K  = static_cast<int>(t.size());
    MatrixXd q(K, N);

    for (int k = 0; k < K; ++k) {
        double tk = t(k);
        for (int i = 0; i < N; ++i) {
            if (tk < trap.t1) {
                q(k,i) = 0.5 * trap.ai(i) * tk*tk + qi(i);
            } else if (tk < trap.t2) {
                q(k,i) = 0.5 * trap.ai(i) * trap.t1*trap.t1
                        + qi(i)
                        + trap.Vi_max(i) * (tk - trap.t1);
            } else if (tk < trap.tf) {
                double dt = tk - trap.t2;
                q(k,i) = 0.5 * trap.ai(i) * trap.t1*trap.t1
                        + qi(i)
                        + trap.Vi_max(i) * (trap.t2 - trap.t1)
                        + trap.Vi_max(i) * dt
                        - 0.5 * trap.ai(i) * dt*dt;
            } else {
                q(k,i) = trap.t2 * trap.Vi_max(i) + qi(i);
            }
        }
    }
    return q;
}

/*
 * Simulations
 */

void Robot::simuTrapezePosition(int clientID, int *handles, const VectorXd& qf,
                                double duree, double dt) const
{
    // Build time vector
    Trapeze trap = calculTrapeze(theta, qf, duree);
    double t_max = std::max(duree, trap.tf);

    int K = static_cast<int>(std::floor(t_max / dt)) + 1;
    VectorXd t_vec(K);
    for (int k = 0; k < K; ++k)
        t_vec(k) = k * dt;

    // Evaluate full trajectory
    MatrixXd q = calculQ(theta, trap, t_vec);

    // Step through and send position commands
    for (int k = 0; k < K; ++k) {
        for (int i=0; i < 6; i++) {
            simxSetJointTargetPosition(clientID, handles[i], q(k,i), simx_opmode_oneshot);
        }
        simxSynchronousTrigger(clientID);
    }
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

    Eigen::VectorXd a_max(dof); a_max << 30.0, 30.0, 30.0, 30.0, 30.0, 30.0; // rad/s^2
    Eigen::VectorXd V_max(dof); V_max << 3.3, 3.3, 3.3, 3.3, 3.2, 3.2; // rad/s
    Eigen::VectorXd q_min(dof); q_min << -pi, -pi/2.0, -pi, -pi, -pi, -pi; // rad
    Eigen::VectorXd q_max(dof); q_max << pi, pi/2.0, pi, pi, pi, pi; // rad

    robot.setTheta(initialTheta);
    robot.setTTool(T_tool);
    robot.setOffsetTheta(offset_theta);
    robot.setAlpha(a);
    robot.setD(d);
    robot.setR(r);
    robot.setM(m);
    robot.setC(c);
    robot.setAMax(a_max);
    robot.setVMax(V_max);
    robot.setQMin(q_min);
    robot.setQMax(q_max);

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
    double theta = -std::asin(R(2, 0));
    double phi   = std::atan2(R(1, 0), R(0, 0));
    double psi   = std::atan2(R(2, 1), R(2, 2));
    return Vector3d(phi, theta, psi);
}