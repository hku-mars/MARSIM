#include <math.h>
#include <dynamics.hpp>

using namespace std;
using namespace Eigen;

class quadrotor_dynamics: public dynamics
{
    private:
        Vector4d actuator_outputs;
        Vector4d motorspeed;
        Vector4d motorRPM;
        double arm_length;
        MatrixXd motor_pos;
        double min_rpm = 0;
        double max_rpm = 35000;
        double g = 9.81;
        double k_F = 3* 8.98132e-9;//fixed parameter for calculate motor drag from motor speed
        double k_T = 0.07 * (3 * 0.062) * k_F; //fixed parameter for calculate motor torque from motor speed

    public:
        // quadrotor_dynamics(double m, Marix3d I):dynamics(m,I){};
        // ~quadrotor_dynamics();

        //use dynamics constructor
        using dynamics::dynamics;
        
        void init(Vector3d p, Vector4d quat);
        void step_forward(double dt);
        void setRPM(Vector4d motorRPM_);
        void setActuatoroutput(Vector4d actuator_outputs_);
};

void quadrotor_dynamics::init(Vector3d p, Vector4d quat)
{
    initialize(p, quat);
    actuator_outputs << 0,0,0,0;
    motorspeed << 0,0,0,0;
    motorRPM << 0,0,0,0;
    arm_length = 0.22;
    motor_pos.resize(4,3);
    motor_pos << arm_length*sqrt(2)/2.0, -arm_length*sqrt(2)/2.0, 0,
                -arm_length*sqrt(2)/2.0, arm_length*sqrt(2)/2.0, 0,
                 arm_length*sqrt(2)/2.0, arm_length*sqrt(2)/2.0, 0,
                -arm_length*sqrt(2)/2.0,-arm_length*sqrt(2)/2.0, 0;
}

void quadrotor_dynamics::step_forward(double dt)
{
    //generate force and torque
    Vector4d F_motor;
    F_motor(0) = k_F * motorRPM(0)*motorRPM(0);
    F_motor(1) = k_F * motorRPM(1)*motorRPM(1);
    F_motor(2) = k_F * motorRPM(2)*motorRPM(2);
    F_motor(3) = k_F * motorRPM(3)*motorRPM(3);
    
    //forward x, left y, up z coordinate, x type quadrotor
    Vector3d z_body, g_body;
    z_body << 0,0,1;
    g_body = mass * R_body2world.transpose() * g * (-z_body);
    Force_body = F_motor.sum() * z_body + g_body;
    Force_world = R_body2world * Force_body;
    // std::cout << motor_pos.row(0)<< std::endl;
    // Torque_body = motor_pos.row(0).cross(F_motor(0)*z_body) + motor_pos.row(1).cross(F_motor(1)*z_body)
    //             + motor_pos.row(2).cross(F_motor(2)*z_body) + motor_pos.row(3).cross(F_motor(3)*z_body)
    //             - k_T * motorRPM(0) * motorRPM(0) * z_body - k_T * motorRPM(1) * motorRPM(1) * z_body
    //             + k_T * motorRPM(1) * motorRPM(1) * z_body + k_T * motorRPM(2) * motorRPM(2) * z_body;
    
    Vector3d torque_temp;
    torque_temp << 0,0,0;
    for(int i = 0; i < 4; i++)
    {
        Vector3d temp;
        temp = motor_pos.row(i);
        torque_temp = torque_temp+ temp.cross(F_motor(i)*z_body);
    }

    Torque_body = torque_temp
                - k_T * motorRPM(0) * motorRPM(0) * z_body - k_T * motorRPM(1) * motorRPM(1) * z_body
                + k_T * motorRPM(2) * motorRPM(2) * z_body + k_T * motorRPM(3) * motorRPM(3) * z_body;
    
    for(int i = 0;i<3;i++)
    {
        Torque_body(i) = Torque_body(i) ;//+ 0.000001 * (rand() % 100 - 50)
    }    

    // Torque_body(2) = 0.0005;
    Torque_world = R_body2world * Torque_body;

    // std::cout << "Force_world = " << Force_world << std::endl;
    // std::cout << "Torque_world = " << Torque_world << std::endl;

    step(dt);
}

void quadrotor_dynamics::setRPM(Vector4d motorRPM_)
{
    motorRPM = motorRPM_;

    for(int i = 0;i<4;i++)
    {
        if(motorRPM(i)>max_rpm)
        {
            motorRPM(i) = max_rpm;
        }else if (motorRPM(i) < min_rpm)
        {
            motorRPM(i) = min_rpm;
        }
    }
}

void quadrotor_dynamics::setActuatoroutput(Vector4d actuator_outputs_)
{
    actuator_outputs = actuator_outputs_;
    Vector4d min_rpm_vec;
    min_rpm_vec << min_rpm,min_rpm,min_rpm,min_rpm;
    motorRPM = min_rpm_vec + actuator_outputs*(max_rpm-min_rpm);
}
