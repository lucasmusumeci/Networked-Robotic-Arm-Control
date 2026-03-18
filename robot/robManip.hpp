#ifndef ROBMANIP_HPP
#define ROBMANIP_HPP

#include <Eigen/Dense>
#include <vector>
#include <utility>

extern "C" {
    #include "extApi.h"
}

// Struct used for command trapeze
struct Trapeze {
    double t1;               // acceleration phase end
    double t2;               // constant velocity phase end
    double tf;               // total duration
    Eigen::VectorXd V_max;   // max velocity per joint
    Eigen::VectorXd a_max;   // max acceleration per joint
};

enum CmdType_t {
    POSITION,
    VELOCITY
};

class Robot 
{
private:
    Eigen::VectorXd theta;
    Eigen::VectorXd offset_theta;
    Eigen::Matrix4d T_tool; // Transformation matrix of the tool added to the end-effector (set to Identity if not used)
    Eigen::VectorXd alpha;
    Eigen::VectorXd d;
    Eigen::VectorXd r;
    Eigen::VectorXd m;
    Eigen::MatrixXd c;

    Eigen::VectorXd V_max;
    Eigen::VectorXd a_max;
    Eigen::VectorXd q_min;
    Eigen::VectorXd q_max;

    int getN() const { return static_cast<int>(alpha.size()); }
    Eigen::Matrix3d hat(const Eigen::Vector3d& v) const;
    Eigen::Matrix3d calcul_L(const Eigen::Matrix3d& Ad,
                                    const Eigen::Matrix3d& Ae) const;

public:
    // Constructors
    Robot() = default;
    Robot(const Eigen::VectorXd& theta, const Eigen::VectorXd& offset_theta, const Eigen::Matrix4d& T_tool,
             const Eigen::VectorXd& alpha, const Eigen::VectorXd& d,
             const Eigen::VectorXd& r, const Eigen::VectorXd& m, const Eigen::MatrixXd& c,
             const Eigen::VectorXd& V_max, const Eigen::VectorXd& a_max,
             const Eigen::VectorXd& q_min, const Eigen::VectorXd& q_max)
        : theta(theta), offset_theta(offset_theta), T_tool(T_tool), alpha(alpha), d(d), r(r), m(m), c(c),
          V_max(V_max), a_max(a_max), q_min(q_min), q_max(q_max) {}

    // Getters
    const Eigen::VectorXd& getTheta() const { return theta; }
    const Eigen::VectorXd& getOffsetTheta() const { return offset_theta; }
    const Eigen::Matrix4d& getTTool() const { return T_tool; }
    const Eigen::VectorXd& getAlpha() const { return alpha; }
    const Eigen::VectorXd& getD() const { return d; }
    const Eigen::VectorXd& getR() const { return r; }
    const Eigen::VectorXd& getM() const { return m; }
    const Eigen::MatrixXd& getC() const { return c; }
    const Eigen::VectorXd& getVMax() const { return V_max; }
    const Eigen::VectorXd& getAMax() const { return a_max; }
    const Eigen::VectorXd& getQMin() const { return q_min; }
    const Eigen::VectorXd& getQMax() const { return q_max; }

    // Setters
    void setTheta(const Eigen::VectorXd& v)        { theta = v; }
    void setOffsetTheta(const Eigen::VectorXd& v)  { offset_theta = v; }
    void setTTool(const Eigen::Matrix4d& v)        { T_tool = v; }
    void setAlpha(const Eigen::VectorXd& v)        { alpha = v; }
    void setD(const Eigen::VectorXd& v)            { d = v; }
    void setR(const Eigen::VectorXd& v)            { r = v; }
    void setM(const Eigen::VectorXd& v)            { m = v; }
    void setC(const Eigen::MatrixXd& v)            { c = v; }
    void setAMax(const Eigen::VectorXd& v)         { a_max = v; }
    void setVMax(const Eigen::VectorXd& v)         { V_max = v; }
    void setQMin(const Eigen::VectorXd& v)         { q_min = v; }
    void setQMax(const Eigen::VectorXd& v)         { q_max = v; }

    // Methods
    std::pair<Eigen::Matrix4d, std::vector<Eigen::Matrix4d>> MGD() const;
    Eigen::MatrixXd Jacobienne(const Eigen::Vector3d& P) const;
    Trapeze calculTrapeze(const Eigen::VectorXd& qi,
                          const Eigen::VectorXd& qf,
                          double duree) const;
    Eigen::MatrixXd calculQ(const Eigen::VectorXd& qi,
                            const Trapeze& trapeze,
                            const Eigen::VectorXd& t) const;
    Eigen::MatrixXd calculQdot(const Eigen::VectorXd& qi,
                            const Trapeze& trapeze,
                            const Eigen::VectorXd& t) const;                        

    // Simulation methods
    void simuTrapeze(int clientID, int *handles,const Eigen::VectorXd& qf,
                             double duree, double dt, CmdType_t cmdType = POSITION);
    // Commande cinématique
    void cmdCinematique(int clientID, int *handles,
                        const Eigen::Vector3d&  Pd,
                        const Eigen::Matrix3d&  Ad,
                        const Eigen::Vector3d&  dPd,
                        const Eigen::Vector3d&  omega_d,
                        double Kp, double K0, double dt,
                        double alpha, double lambda_L,
                        CmdType_t cmdType = POSITION);

    // Joint limit avoidance using null-space redundancy
    Eigen::VectorXd eloignement_butees_articulaires(const Eigen::MatrixXd& J,
                                                    const Eigen::VectorXd& Xdot,
                                                    double alpha) const;

};

// Create a Robotis-H robot with the given DH parameters
Robot CreateRobotisH(const Eigen::VectorXd& initialTheta);

// Rotation conversions
Eigen::Matrix3d RTL2R(double phi, double theta, double psi);
Eigen::Vector3d R2RTL(const Eigen::Matrix3d& R);

// Communication with simulator
void sendCmd(int clientID, int *handles, const Eigen::VectorXd& q, CmdType_t cmdType);
void getJointPosition(int clientID, int jointHandle, double* position);

#endif // ROBOT_KINEMATICS_HPP