#include "robManip.hpp"
#include <cmath>
#include <iostream>

using namespace Eigen;

/*
 * Hat operator for 3D vectors
 */

Matrix3d Robot::hat(const Vector3d& v) const {
    Matrix3d S;
    S <<  0.0,  -v(2),  v(1),
          v(2),  0.0,  -v(0),
         -v(1),  v(0),  0.0;
    return S;
}

/*
 * Compute L matrix used in MCI
 */
Matrix3d Robot::calcul_L(const Matrix3d& Ad, const Matrix3d& Ae) const {
    return -0.5 * (hat(Ae.col(0)) * hat(Ad.col(0))
                 + hat(Ae.col(1)) * hat(Ad.col(1))
                 + hat(Ae.col(2)) * hat(Ad.col(2)));
}


/*
 * If there is a tool attached to the end-effector, it is represented by the transformation matrix T_tool.
 * If there is no tool, T_tool should be set to the identity matrix.
 */

std::pair<Matrix4d, std::vector<Matrix4d>> Robot::MGD() const {
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

MatrixXd Robot::Jacobienne(const Vector3d& P) const {
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
    VectorXd V_max(N), a_max(N);
    for (int i = 0; i < N; ++i) {
        V_max(i) = dq(i) / t2;
        a_max(i)     = dq(i) / (t2 * t1);
    }

    return {t1, t2, tf, V_max, a_max};
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
                q(k,i) = 0.5 * trap.a_max(i) * tk*tk + qi(i);
            } else if (tk < trap.t2) {
                q(k,i) = 0.5 * trap.a_max(i) * trap.t1*trap.t1
                        + qi(i)
                        + trap.V_max(i) * (tk - trap.t1);
            } else if (tk < trap.tf) {
                double dt = tk - trap.t2;
                q(k,i) = 0.5 * trap.a_max(i) * trap.t1*trap.t1
                        + qi(i)
                        + trap.V_max(i) * (trap.t2 - trap.t1)
                        + trap.V_max(i) * dt
                        - 0.5 * trap.a_max(i) * dt*dt;
            } else {
                q(k,i) = trap.t2 * trap.V_max(i) + qi(i);
            }
        }
    }
    return q;
}

MatrixXd Robot::calculQdot(const VectorXd& qi,
                         const Trapeze& trap,
                         const VectorXd& t) const {
    int N  = getN();
    int K  = static_cast<int>(t.size());
    MatrixXd qdot(K, N);

    for (int k = 0; k < K; ++k) {
        double tk = t(k);
        for (int i = 0; i < N; ++i) {
            if (tk < trap.t1) {
                qdot(k,i) = trap.a_max(i) * tk;
            } else if (tk < trap.t2) {
                qdot(k,i) = trap.V_max(i);
            } else if (tk < trap.tf) {
                double dt = tk - trap.t2;
                qdot(k,i) = trap.V_max(i) - trap.a_max(i) * dt;
            } else {
                qdot(k,i) = 0;
            }
        }
    }
    return qdot;
}

/*
 * Simulation methods
 */

void Robot::simuTrapeze(int clientID, int *handles, const VectorXd& qf,
                                double duree, double dt, CmdType_t cmdType)
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

    switch (cmdType)
    {
    case POSITION:
        for (int k = 0; k < K; ++k) {
            sendCmd(clientID, handles, q.row(k), POSITION);
            theta = q.row(k);
        }
        break;
    
    case VELOCITY:
        MatrixXd qdot = calculQdot(theta, trap, t_vec);
        for (int k = 0; k < K; ++k) {
            sendCmd(clientID, handles, qdot.row(k), VELOCITY);
            theta = q.row(k);
        }        
        break;
    }
}

/*
 * Commande cinématique
 */

 void Robot::cmdCinematique(int clientID, int *handles,
                            const Vector3d&  Pd,
                            const Matrix3d&  Ad,
                            const Vector3d&  dPd,
                            const Vector3d&  omega_d,
                            double Kp, double K0, double dt,
                            double alpha, double lambda_L, 
                            CmdType_t cmdType)
{
    VectorXd qc = theta;

    auto [Te, MTe] = MGD();
    Matrix3d Ae = Te.block<3,3>(0,0);
    Vector3d Pe = Te.block<3,1>(0,3);

    Vector3d eps0 = Vector3d::Ones();  // to enter loop
    int iter = 0;
    int N = getN();

    double threshold_position = 1e-2;
    double threshold_orientation = 1e-2;

    switch (cmdType)
    {
    case POSITION:
        threshold_position = 1e-2;
        threshold_orientation = 1e-2;
        break;
    case VELOCITY:
        threshold_position = 1e-2;
        threshold_orientation = 1e-2;
        break;
    }


    while ((Pe - Pd).norm() > threshold_position || eps0.norm() > threshold_orientation) {
        if (++iter > 2000) break;

        // Orientation error
        eps0 = 0.5 * (Ae.col(0).cross(Ad.col(0))
                    + Ae.col(1).cross(Ad.col(1))
                    + Ae.col(2).cross(Ad.col(2)));

        // Angular velocity command
        Vector3d omega_e;
        if (eps0.norm() < 1e-2) {
            omega_e = omega_d;
        } else {
            Matrix3d L = calcul_L(Ad, Ae);
            // Damped pseudo-inverse of L to avoid numerical blow-up near singularities
            Matrix3d L_pinv = L.transpose() *
                              (L * L.transpose() + lambda_L*lambda_L * Matrix3d::Identity()).inverse();
            omega_e = L_pinv * (K0 * eps0 + L.transpose() * omega_d);
        }

        // Linear velocity command
        Vector3d dPe = dPd + Kp * (Pd - Pe);

        // Joint velocity via damped Jacobian
        MatrixXd J  = Jacobienne(Pe);
        Matrix<double,6,1> xdot;
        xdot << dPe, omega_e;

        // Eloignement des butées articulaires
        VectorXd dq = eloignement_butees_articulaires(J, xdot, alpha);

        // Clamp and integrate
        for (int i = 0; i < N; ++i)
            dq(i) = std::clamp(dq(i), -V_max(i), V_max(i));

        qc += dq * dt;

        for (int i = 0; i < N; ++i)
            qc(i) = std::clamp(qc(i), q_min(i), q_max(i));

        // Send command to CoppeliaSim
        switch (cmdType)
        {
        case POSITION:
            sendCmd(clientID, handles, qc, POSITION);
            break;
        
        case VELOCITY:
            sendCmd(clientID, handles, dq, VELOCITY);
            break;
        }

        // Update state
        if(getAllJointsPosition(clientID, handles, &theta) == -1) {
            //std::cerr << "Error getting joint positions from simulator." << std::endl;
            theta = qc; // Fallback to estimated position if reading fails
        }
        
        auto [Te_new, MT_new] = MGD();
        Ae = Te_new.block<3,3>(0,0);
        Pe = Te_new.block<3,1>(0,3);
    }
        
    // Send null command to CoppeliaSim when in velocity mode to stop the robot
    if (cmdType == VELOCITY)
    {
        VectorXd dq = VectorXd::Zero(getN());
        sendCmd(clientID, handles, dq, VELOCITY);
    }
}

Eigen::VectorXd Robot::eloignement_butees_articulaires(const Eigen::MatrixXd& J,
                                                      const Eigen::VectorXd& Xdot,
                                                      double alpha) const
{
    int n = getN();
    
    // Pseudo-inverse of Jacobian
    Eigen::MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
    
    // Compute potential function gradient ∇φ using member limits q_min and q_max
    Eigen::VectorXd theta_moy = (q_max + q_min) / 2.0;
    Eigen::VectorXd theta_range = q_max - q_min;
    
    // Element-wise: ∇ϕ = 2 * (θ - θmoy) / (θrange²)
    Eigen::VectorXd grad_phi(n);
    for (int i = 0; i < n; ++i) {
        grad_phi(i) = 2.0 * (theta(i) - theta_moy(i)) / (theta_range(i) * theta_range(i));
    }
    
    // Main equation: θ̇ = J⁺Ẋ + (I - J⁺J)α∇φ
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd null_space = I - J_pinv * J;
    
    Eigen::VectorXd theta_dot = J_pinv * Xdot + alpha * null_space * grad_phi;
    
    return theta_dot;
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
    Eigen::VectorXd q_min(dof); q_min << -pi, -pi/2.0, -pi/2, -pi, -pi/2, -pi; // rad
    Eigen::VectorXd q_max(dof); q_max << pi, pi/2.0, 3*pi/4, pi, pi, pi; // rad

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