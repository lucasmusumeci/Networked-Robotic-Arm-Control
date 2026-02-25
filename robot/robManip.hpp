#ifndef ROBMANIP_HPP
#define ROBMANIP_HPP

#include <Eigen/Dense>
#include <vector>
#include <utility>

// Structure to hold Robot DH parameters
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
public:
    // Constructors
    Robot() = default;
    Robot(const Eigen::VectorXd& theta, const Eigen::VectorXd& offset_theta, const Eigen::Matrix4d& T_tool,
             const Eigen::VectorXd& alpha, const Eigen::VectorXd& d,
             const Eigen::VectorXd& r, const Eigen::VectorXd& m, const Eigen::MatrixXd& c)
        : theta(theta), offset_theta(offset_theta), alpha(alpha), d(d), r(r), m(m), c(c) {}

    // Getters
    const Eigen::VectorXd& getTheta() const { return theta; }
    const Eigen::VectorXd& getOffsetTheta() const { return offset_theta; }
    const Eigen::Matrix4d& getTTool() const { return T_tool; }
    const Eigen::VectorXd& getAlpha() const { return alpha; }
    const Eigen::VectorXd& getD() const { return d; }
    const Eigen::VectorXd& getR() const { return r; }
    const Eigen::VectorXd& getM() const { return m; }
    const Eigen::MatrixXd& getC() const { return c; }

    // Setters
    void setTheta(const Eigen::VectorXd& newTheta) { theta = newTheta; }
    void setOffsetTheta(const Eigen::VectorXd& newOffsetTheta) { offset_theta = newOffsetTheta; }
    void setTTool(const Eigen::Matrix4d& newTTool) { T_tool = newTTool; }
    void setAlpha(const Eigen::VectorXd& newAlpha) { alpha = newAlpha; }
    void setD(const Eigen::VectorXd& newD) { d = newD;  }
    void setR(const Eigen::VectorXd& newR) { r = newR; }
    void setM(const Eigen::VectorXd& newM) { m = newM; }
    void setC(const Eigen::MatrixXd& newC) { c = newC; }

    // Methods
    std::pair<Eigen::Matrix4d, std::vector<Eigen::Matrix4d>> MGD();
    Eigen::MatrixXd Jacobienne(const Eigen::Vector3d& P);
    

};

// Create a Robotis-H robot with the given DH parameters
Robot CreateRobotisH(const Eigen::VectorXd& initialTheta);

// Rotation conversions
Eigen::Matrix3d RTL2R(double phi, double theta, double psi);
Eigen::Vector3d R2RTL(const Eigen::Matrix3d& R);

#endif // ROBOT_KINEMATICS_HPP