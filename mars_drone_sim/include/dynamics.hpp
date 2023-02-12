#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <stdio.h>

// using namespace std;
using namespace Eigen;

class dynamics
{
protected:
    Matrix3d Internal_mat;
    double mass;
    Vector4d q;
    Matrix3d R_body2world;
    Vector3d w_body, w_world;
    Vector3d acceleration;
    Vector3d velocity;
    Vector3d pos;
    Vector3d Force_world;
    Vector3d Force_body;
    Vector3d Torque_world;
    Vector3d Torque_body;

public:
    dynamics(double m, Eigen::Matrix3d I);
    ~dynamics();
    void initialize(Eigen::Vector3d p, Eigen::Vector4d quat);
    void step(double dt);

    Vector3d getPos();
    Vector3d getVel();
    Vector3d getAcc();
    Vector4d getQuat();
    Vector3d getAngularVel();
    Matrix3d getR();
};

dynamics::dynamics(double m, Eigen::Matrix3d I)
{
    mass = m;
    Internal_mat = I;
}

dynamics::~dynamics()
{
}

void dynamics::initialize(Eigen::Vector3d p, Eigen::Vector4d quat)
{
    pos = p;
    // Eigen::Quaterniond q_temp(quat(0), quat(1), quat(2), quat(3));
    // q = q_temp;
    // q.normalize();
    q << quat(0), quat(1), quat(2), quat(3);
    Eigen::Quaterniond quaternion(quat(0), quat(1), quat(2), quat(3));
    R_body2world = quaternion.matrix();
    w_body << 0,0,0;
    velocity << 0,0,0;
    acceleration << 0,0,0;
}

void dynamics::step(double dt)
{
    //from force to position
    acceleration = Force_world/mass;
    pos = pos + velocity*dt;
    velocity = velocity + acceleration*dt;

    //ouler equation
    w_body = w_body + dt * Internal_mat.inverse() * (Torque_body - w_body.cross(Internal_mat*w_body));

    //from angular velocity to quat
    Eigen::Matrix4d q_transmat;
    Vector4d quat_temp;
    quat_temp = q;
    q_transmat << quat_temp(0), -quat_temp(1), -quat_temp(2), -quat_temp(3),
                quat_temp(1), quat_temp(0), -quat_temp(3), quat_temp(2),
                quat_temp(2), quat_temp(3), quat_temp(0), -quat_temp(1),
                quat_temp(3), -quat_temp(2), quat_temp(1), quat_temp(0);
    Eigen::Vector4d w_body_withzero, d_q;
    w_body_withzero << 0, w_body;
    d_q = 0.5*q_transmat*w_body_withzero;
    q = q + d_q * dt;
    q.normalize();
    Eigen::Quaterniond temp_quaternion(q(0), q(1), q(2), q(3));
    R_body2world = temp_quaternion.matrix();
}

Vector3d dynamics::getPos()
{
    return pos;
}

Vector3d dynamics::getVel()
{
    return velocity;
}

Vector3d dynamics::getAcc()
{
    return acceleration;
}

Vector4d dynamics::getQuat()
{
    // Vector4d q_temp;
    // q_temp << q.w(), q.x(), q.y(), q.z();
    return q;
}

Vector3d dynamics::getAngularVel()
{
    return w_body;
}

Matrix3d dynamics::getR()
{
    return R_body2world;
}