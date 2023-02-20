#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <ros/package.h>

using namespace std;
using namespace Eigen;

class cascadePID
{
private:
    double dt;
    Vector3d Pos_des;
    Vector3d Vel_des;
    Vector3d Acc_des;
    Vector3d Angle_Kd,Angle_Kp,Angle_Ki;
    Vector3d Torque_des;
    Vector3d euler_angle,last_euler_angle, angle_des, last_angle_des;
    Vector3d current_pos, current_vel, current_acc;
    Eigen::Quaterniond q;
    Matrix3d R_body2world;
    double Yaw_des, Yaw_cur;
    int angle_in_flag = 0;
    int angledes_in_flag = 0;
    Vector3d Angle_error, Angle_last_error, d_Angle_error, Pos_error, Vel_error;
    int first_control_flag = 0;
    double angle_wn;
    Matrix3d Internal_mat;
    double mass = 1.9;
    double arm_length;
    double k_F = 8.98132e-9;//fixed parameter for calculate motor drag from motor speed
    double k_T = 0.07 * (3 * 0.062) * k_F; //fixed parameter for calculate motor torque from motor speed
    Matrix4d Mixermatrix_rpm2torque;
    MatrixXd motor_pos;
    Vector4d RPM_output, RPM_output_last;
    Vector3d PID_POS_Z, PID_POS_XY;
    double min_rpm = 2000;//10500
    double max_rpm = 35000;//35000
    double g = 9.81;
    double z_intergral_val = 0;
    double z_intergral_limit = 2;
    double RPM_change_limit = 1000;
    double Torque_limit = 20;
    double Torque_limit_yaw = 0.1;
    std::ofstream myfile;
    std::string pkg_path;
    int droneid = 0;

public:
    cascadePID(double control_rate);
    ~cascadePID();
    void setdroneid(int id);
    void setrate(double control_rate);
    void setParam(double stable_time, double damping_ratio);
    void setInternal(double m, Matrix3d Internal_mat, double arm, double kF);
    void FeedbackInput(Vector3d pos_fb, Vector3d vel_fb, Eigen::Quaterniond q_in);
    void setAngledes(Vector3d angle);
    void setOdomdes(Vector3d P_des, Vector3d V_des, Vector3d A_des, double Y_des);
    void RunController();
    Vector3d getTorquedes();
    Vector4d getRPMoutputs();
    void Quat2EulerAngle(const Quaterniond& q_input, double& roll, double& pitch, double& yaw);
};

cascadePID::cascadePID(double control_rate)
{
    dt = 1.0/control_rate;
    RPM_output<< 0,0,0,0;
    RPM_output_last << 0,0,0,0;
    RPM_change_limit = 1000.0*(dt/0.005);

    // pkg_path = ros::package::getPath("cascadePID");
    // pkg_path.append("/data/log_" + std::to_string(droneid) + ".txt");
    // std::cout << "\nFound pkg_path = " << pkg_path << std::endl;
    // myfile.open(pkg_path.c_str(), std::ios_base::out);
}

cascadePID::~cascadePID()
{
    myfile.close();
}

void cascadePID::setdroneid(int id){
    droneid = id;

    pkg_path = ros::package::getPath("cascadePID");
    pkg_path.append("/data/log_" + std::to_string(droneid) + ".txt");
    std::cout << "\nFound pkg_path = " << pkg_path << std::endl;
    myfile.open(pkg_path.c_str(), std::ios_base::out);
}

void cascadePID::setrate(double control_rate)
{
    dt = 1.0/control_rate;
}

void cascadePID::Quat2EulerAngle(const Quaterniond& q_input, double& roll, double& pitch, double& yaw)
{
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q_input.w() * q_input.x() + q_input.y() * q_input.z());
    double cosr_cosp = +1.0 - 2.0 * (q_input.x() * q_input.x() + q_input.y() * q_input.y());
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q_input.w() * q_input.y() - q_input.z() * q_input.x());
    if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
    pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q_input.w() * q_input.z() + q_input.x() * q_input.y());
    double cosy_cosp = +1.0 - 2.0 * (q_input.y() * q_input.y() + q_input.z() * q_input.z());
    yaw = atan2(siny_cosp, cosy_cosp);

    // if(roll > 3.1415)
    // {
    //     roll = roll - 3.1415926;
    // }else if(roll < -3.1415){
    //     roll = roll + 3.1415926;
    // }

    // if(pitch > 3.1415)
    // {
    //     pitch = pitch - 3.1415926;
    // }else if(pitch < -3.1415){
    //     pitch = pitch + 3.1415926;
    // }

    // if(yaw > 3.1415)
    // {
    //     yaw = yaw - 3.1415926;
    // }else if(roll < -3.1415){
    //     yaw = yaw + 3.1415926;
    // }
}

void cascadePID::setInternal(double m, Matrix3d I, double arm, double kF)
{
    Internal_mat = I;
    arm_length = arm;
    k_F = kF;
    k_T = 0.07 * (3 * 0.062) * k_F;

    // std::cout << "K_F = " << k_F<<std::endl;

    motor_pos.resize(4,3);
    motor_pos << arm_length*sqrt(2)/2.0, -arm_length*sqrt(2)/2.0, 0,
                -arm_length*sqrt(2)/2.0, arm_length*sqrt(2)/2.0, 0,
                 arm_length*sqrt(2)/2.0, arm_length*sqrt(2)/2.0, 0,
                -arm_length*sqrt(2)/2.0,-arm_length*sqrt(2)/2.0, 0;

    Vector3d z_body;
    z_body << 0,0,1;
    MatrixXd Mixermatrix_rpm2torque_noForce;
    Mixermatrix_rpm2torque_noForce.resize(3,4);

    for(int i = 0; i < 4; i++)
    {
        Vector3d temp;
        temp = motor_pos.row(i);
        // torque_temp = torque_temp+ temp.cross(F_motor(i)*z_body);
        Mixermatrix_rpm2torque_noForce.col(i) = k_F* temp.cross(z_body);// + k_T* z_body
    }

    // Vector3d a = k_F* motor_pos.row(0).transpose().cross(z_body) + k_T* z_body;
    Mixermatrix_rpm2torque_noForce.col(0) = Mixermatrix_rpm2torque_noForce.col(0) - k_T* z_body;
    Mixermatrix_rpm2torque_noForce.col(1) = Mixermatrix_rpm2torque_noForce.col(1) - k_T* z_body;
    Mixermatrix_rpm2torque_noForce.col(2) = Mixermatrix_rpm2torque_noForce.col(2) + k_T* z_body;
    Mixermatrix_rpm2torque_noForce.col(3) = Mixermatrix_rpm2torque_noForce.col(3) + k_T* z_body;
    Mixermatrix_rpm2torque.row(0) << k_F, k_F, k_F, k_F;
    Mixermatrix_rpm2torque.block<3,4>(1,0) = Mixermatrix_rpm2torque_noForce;
    std::cout << Mixermatrix_rpm2torque <<std::endl;
}

void cascadePID::setParam(double stable_time, double damping_ratio)
{
    angle_wn = 4.6/(damping_ratio*stable_time);
    Angle_Kd << 2*damping_ratio*angle_wn, 2*damping_ratio*angle_wn, 2*damping_ratio*angle_wn;
    Angle_Kp << angle_wn*angle_wn, angle_wn*angle_wn, angle_wn*angle_wn;
    // Angle_Kp(2) = 0.03*Angle_Kp(2);//0.3 0.25
    // Angle_Kd(2) = 0.05*Angle_Kd(2);//0.5
    Angle_Kp(2) = 1*Angle_Kp(2);//0.3 0.25
    Angle_Kd(2) = 1*Angle_Kd(2);//0.5
    // Angle_Kp(2) = (4.6/(0.7*1.0))*4.6/(0.7*1.0);//0.3 0.25
    // Angle_Kd(2) = 2*0.7*4.6/(0.7*1.0);//0.5
}

void cascadePID::FeedbackInput(Vector3d pos_fb, Vector3d vel_fb, Eigen::Quaterniond q_in)
{
    q = q_in;
    R_body2world = q_in.matrix();
    current_pos = pos_fb;
    current_vel = vel_fb;
    Eigen::Vector3d angle = q_in.matrix().eulerAngles(0,1,2);
    // Eigen::Vector3d angle;
    // Quat2EulerAngle(q_in,angle(0),angle(1),angle(2));
    if(angle_in_flag == 1)
    {
        last_euler_angle = euler_angle;
    }else{
        angle_in_flag=1;
        last_euler_angle << 0,0,0;
    }
    euler_angle = angle;

    //calculate Yaw
    Eigen::Vector3d x_body;
    x_body << 1,0,0;
    x_body = R_body2world*x_body;
    Yaw_cur = atan2(x_body(1),x_body(0));
}

void cascadePID::setAngledes(Vector3d angle)
{
    if(angledes_in_flag == 1)
    {
        last_angle_des = angle_des;
    }else{
        angledes_in_flag=1;
        last_angle_des << 0,0,0;
    }
    angle_des = angle;
}

void cascadePID::setOdomdes(Vector3d P_des, Vector3d V_des, Vector3d A_des, double Y_des)
{
    Pos_des = P_des;
    Vel_des = V_des;
    Acc_des = A_des;
    Yaw_des = Y_des;
}

void cascadePID::RunController()
{
    //altitude controller
    double t_z_stable = 2.0;
    double damping_ratio_z = 0.8;
    double wn_z = 4.6/(damping_ratio_z*t_z_stable);
    double KP_Z = wn_z*wn_z;
    double KI_Z = 0;//50
    double KD_Z = wn_z*2*damping_ratio_z;
    double t_xy_stable = 3.0;
    double damping_ratio_xy = 0.90;
    double wn_xy = 4.6/(damping_ratio_xy*t_xy_stable);
    double KP_XY = wn_xy*wn_xy;
    double KD_XY = wn_xy*2*damping_ratio_xy;

    Pos_error = Pos_des - current_pos;
    Vel_error = Vel_des - current_vel;

    double max_pos_error = 1*10.0;//*dt/(1.0/20.0)
    double max_vel_error = 1*10.0;//*dt/(1.0/20.0)
    Pos_error(0) = std::max(std::min(Pos_error(0), max_pos_error), -max_pos_error);
    Pos_error(1) = std::max(std::min(Pos_error(1), max_pos_error), -max_pos_error);
    Pos_error(2) = std::max(std::min(Pos_error(2), max_pos_error), -max_pos_error);
    Vel_error(0) = std::max(std::min(Vel_error(0), max_vel_error), -max_vel_error);
    Vel_error(1) = std::max(std::min(Vel_error(1), max_vel_error), -max_vel_error);
    Vel_error(2) = std::max(std::min(Vel_error(2), max_vel_error), -max_vel_error);

    z_intergral_val = z_intergral_val + Pos_error(2) * dt;
    if(z_intergral_val > z_intergral_limit)
    {
        z_intergral_val = z_intergral_limit;
    }else if(z_intergral_val < -z_intergral_limit)
    {
        z_intergral_val = -z_intergral_limit;
    }

    // std::cout << "Pos_error = " << Pos_error(0) <<" "<< Pos_error(1) <<" "<< Pos_error(2) <<" " 
    //              "Vel_error = " << Vel_error(0) <<" "<< Vel_error(1) <<" "<< Vel_error(2) <<" " 
    //              "z_intergral_val = "<< z_intergral_val << std::endl;

    double a_des_z = (KP_Z*Pos_error(2) + KD_Z*Vel_error(2) + KI_Z * z_intergral_val + Acc_des(2));

    if(a_des_z > 4*g)
    {
        a_des_z = 4*g;
    }else if(a_des_z<(-0.8*g)){
        a_des_z = -0.8*g;
    }

    double F_des_z = mass * (a_des_z+g);//
    Vector3d z_body;
    z_body << 0,0,1;
    double current_tilt_angle = acos((R_body2world* z_body).dot(z_body));
    double F_des = F_des_z / cos(current_tilt_angle);

    // std::cout << "F_des_z = " << F_des_z << " "<< "F_des = " << F_des << std::endl;

    F_des = std::max(std::min(F_des, mass*4*g), 0.0);

    //Position XY controller
    //ax to pitch
    //ay to roll
    double acc_x_des = (KP_XY*Pos_error(0) + KD_XY*Vel_error(0) + Acc_des(0));
    double acc_y_des = (KP_XY*Pos_error(1) + KD_XY*Vel_error(1) + Acc_des(1));

    acc_x_des = std::max(std::min(acc_x_des, 10.0), -10.0);
    acc_y_des = std::max(std::min(acc_y_des, 10.0), -10.0);

    //second solution to calculate angle des 
    Eigen::Vector3d new_z_body, rotation_vec;
    new_z_body << mass*acc_x_des,mass*acc_y_des,F_des_z;//F_des

    // std::cout << "new_z_body = " << new_z_body << std::endl;

    new_z_body.normalize();
    Matrix3d new_z_body_inv,R_des,I;
    //if at a line
    if(new_z_body.dot(z_body) == 1.0)
    {
    R_des << 1,0,0,
            0,1,0,
            0,0,1;
    }else{
    rotation_vec = z_body.cross(new_z_body);
    rotation_vec.normalize();

    new_z_body_inv << 0,-rotation_vec(2),rotation_vec(1),
                        rotation_vec(2),0,-rotation_vec(0),
                        -rotation_vec(1),rotation_vec(0),0;
    double rotation_angle = acos(z_body.dot(new_z_body));
    rotation_angle = std::max(std::min(rotation_angle, 0.35*1.5), -0.35*1.5);//0.35*1.5
    I << 1,0,0,
        0,1,0,
        0,0,1;
    R_des = I + sin(rotation_angle)*new_z_body_inv + (1-cos(rotation_angle))*new_z_body_inv*new_z_body_inv;        
    }

    angle_des = R_des.eulerAngles(0,1,2);

    //trans R into ROLL PITCH angle
    // angle_des(0) = acos(R_des(1,1));
    // angle_des(1) = acos(R_des(0,0));
    // angle_des(0) = atan2(R_des(0,1) , R_des(0,2));
    // angle_des(1) = atan2(R_des(0,2) , R_des(2,2));
    // Eigen::Quaterniond quat_des(R_des);
    // Quat2EulerAngle(quat_des, angle_des(0), angle_des(1), angle_des(2));

    // std::cout << "R_des = " << R_des << std::endl;

    // //? tilt angle des only use accx??
    // double theta_des = atan2(acc_y_des, acc_x_des);
    // double tilt_angle_des = asin((mass* sqrt(acc_x_des*acc_x_des + acc_y_des*acc_y_des))/ (F_des*cos(theta_des)));
    // //compute desire rotation
    // Matrix3d R_z, R_y, R_des;
    // // R_z << 1,0,0,
    // //         0,cos(theta_des),-sin(theta_des),
    // //         0,sin(theta_des),cos(theta_des);
    // R_z << cos(theta_des),-sin(theta_des),0,
    //         sin(theta_des),cos(theta_des),0,
    //         0,0,1;
    // R_y << cos(tilt_angle_des), 0, sin(tilt_angle_des),
    //         0,1,0,
    //         -sin(tilt_angle_des), 0, cos(tilt_angle_des);
    // R_des = R_z*R_y;
    // angle_des = R_des.eulerAngles(0,1,2);

    double angle_limit = 30.0/180.0*3.1415926;
    // angle_des(0) = std::max(std::min(angle_des(0), angle_limit), -angle_limit);
    // angle_des(1) = std::max(std::min(angle_des(1), angle_limit), -angle_limit);

    // std::cout << "Angle_des = " << angle_des(0) << " " << angle_des(1) << " "<< angle_des(2) 
    //             // << " theta_des = " << theta_des << " tilt_angle_des = " << tilt_angle_des << std::endl;
    //             << "euler angle = " << euler_angle(0) << " " << euler_angle(1) << " " << euler_angle(2) << std::endl;

    //add yaw rotation
    //limit 
    // double max_yaw_change = 90.0 / 180.0 * 3.14159265357;
    // if(Yaw_des > 3.1415926)
    // {
    //     Yaw_des = Yaw_des - floor(Yaw_des/3.1415926) * 3.1415926;
    // }else if(Yaw_des < -3.1415926 )
    // {
    //     Yaw_des = Yaw_des + floor(Yaw_des/(-3.1415926)) * 3.1415926;
    // }
    // if(Yaw_cur > 3.1415926)
    // {
    //     Yaw_cur = Yaw_cur - floor(Yaw_cur/3.1415926) * 3.1415926;
    // }else if(Yaw_des < -3.1415926 )
    // {
    //     Yaw_cur = Yaw_cur + floor(Yaw_cur/(-3.1415926)) * 3.1415926;
    // }

    // double yaw_error = Yaw_des - Yaw_cur;
    // if(yaw_error > max_yaw_change)
    // {
    //     Yaw_des = Yaw_cur + max_yaw_change;
    // }else if(yaw_error < -max_yaw_change)
    // {
    //     Yaw_des = Yaw_cur - max_yaw_change;
    // }

    Matrix3d R_z;
    R_z << cos(Yaw_des),-sin(Yaw_des),0,
            sin(Yaw_des),cos(Yaw_des),0,
            0,0,1;

    //error state
    Matrix3d R_diff;
    R_diff = R_body2world.transpose() * (R_des*R_z);//*

    std::cout << R_des<< std::endl;

    //method 1: trans delta_R to euler angle and compute desire Torque
    // Eigen::Quaterniond quat_diff(R_diff);
    // Quat2EulerAngle(quat_diff, Angle_error(0), Angle_error(1), Angle_error(2));
    // // Angle_error = R_diff.eulerAngles(0,1,2);

    // if(isnan(Angle_error(0))){Angle_error(0) = Angle_last_error(0);}
    // if(isnan(Angle_error(1))){Angle_error(1) = Angle_last_error(1);}
    // if(isnan(Angle_error(2))){Angle_error(2) = Angle_last_error(2);}

    // if(first_control_flag == 0)
    // {
    //     d_Angle_error << 0,0,0;
    //     first_control_flag = 1;
    // }else{
    //     d_Angle_error = (Angle_error - Angle_last_error)/dt;
    // }

    // double max_angvel_error = 2;
    // double max_ang_error = 2;
    // // Angle_error(2) = std::max(std::min(Angle_error(2), max_ang_error), -max_ang_error);
    // // d_Angle_error(2) = std::max(std::min(d_Angle_error(2), max_angvel_error), -max_angvel_error);
    // Angle_last_error = Angle_error;

    // //angle controller
    // // Angle_error = angle_des - euler_angle;
    // // d_Angle_error = (angle_des-last_angle_des)/dt - (euler_angle-last_euler_angle)/dt;
    // Torque_des(0) =  (Angle_Kp(0)*Angle_error(0) + Angle_Kd(0)*d_Angle_error(0));
    // Torque_des(1) =  (Angle_Kp(1)*Angle_error(1) + Angle_Kd(1)*d_Angle_error(1));
    // Torque_des(2) =  (Angle_Kp(2)*Angle_error(2) + Angle_Kd(2)*d_Angle_error(2));
    // Torque_des = Internal_mat * Torque_des;

    //method 2: trans delta_R to rotation vector and compute desire torque
    Eigen::AngleAxisd rotation_vec_des(R_diff);
    // rotation_vec_des.fromRotationMatrix(R_diff);
    // std::cout << "rotation vector = " << rotation_vec_des.axis().transpose()  << ", rotation angle = "<< rotation_vec_des.angle()*(180/M_PI) << std::endl;

    if(first_control_flag == 0)
    {
        d_Angle_error << 0,0,0;
        first_control_flag = 1;
    }else{
        d_Angle_error = (rotation_vec_des.axis()*rotation_vec_des.angle() - Angle_last_error)/dt;
    }
    Angle_last_error = rotation_vec_des.axis()*rotation_vec_des.angle();

    Torque_des(0) =  (Angle_Kp(0)*rotation_vec_des.axis()(0)*rotation_vec_des.angle() + Angle_Kd(0)*d_Angle_error(0));
    Torque_des(1) =  (Angle_Kp(1)*rotation_vec_des.axis()(1)*rotation_vec_des.angle() + Angle_Kd(1)*d_Angle_error(1));
    Torque_des(2) =  (Angle_Kp(2)*rotation_vec_des.axis()(2)*rotation_vec_des.angle() + Angle_Kd(2)*d_Angle_error(2));
    Torque_des = Internal_mat * Torque_des;

    for(int i = 0;i<2;i++)
    {
        if(Torque_des(i) > Torque_limit)
        {
            Torque_des(i) = Torque_limit;
        }else if(Torque_des(i) < -Torque_limit)
        {
            Torque_des(i) = -Torque_limit;
        }
    }

        if(Torque_des(2) > Torque_limit_yaw)
        {
            Torque_des(2) = Torque_limit_yaw;
        }else if(Torque_des(2) < -Torque_limit_yaw)
        {
            Torque_des(2) = -Torque_limit_yaw;
        }

        // std::cout << "K_F = " << k_F<<std::endl;
    
    //mixer
    for(int i=0;i<4;i++)
    {
        // sqrt cause nan
        RPM_output(i) = F_des/(4.0*k_F)
                        + Torque_des(0)/(4.0 * Mixermatrix_rpm2torque(1,i))
                        + Torque_des(1)/(4.0 * Mixermatrix_rpm2torque(2,i))
                        + Torque_des(2)/(4.0 * Mixermatrix_rpm2torque(3,i));

        // double yaw2rpm = Torque_des(2)/(4.0 * Mixermatrix_rpm2torque(3,i));
        // double yaw2rpm_limit = 5000;
        // if(yaw2rpm > yaw2rpm_limit)
        // {
        //     yaw2rpm = yaw2rpm_limit;
        // }else if(yaw2rpm < -yaw2rpm_limit)
        // {
        //     yaw2rpm = -yaw2rpm_limit;
        // }
        // RPM_output(i) = RPM_output(i) + yaw2rpm;

        if(RPM_output(i) < 0)
        {
            RPM_output(i) = 0;
        }else{
            RPM_output(i) = sqrt(RPM_output(i));
        }
    }
    //limit
    for(int i = 0;i<4;i++)
    {
        if((RPM_output(i) - RPM_output_last(i)) > RPM_change_limit)
        {
            RPM_output(i) = RPM_output_last(i) + RPM_change_limit;
        }else if((RPM_output(i) - RPM_output_last(i)) < -RPM_change_limit)
        {
            RPM_output(i) = RPM_output_last(i) - RPM_change_limit;
        }

        if(RPM_output(i)>max_rpm)
        {
            RPM_output(i) = max_rpm;
        }else if (RPM_output(i) < min_rpm)
        {
            RPM_output(i) = min_rpm;
        }        

        
    }
    RPM_output_last = RPM_output;

    //record log
    myfile << Pos_des(0) << " " << Pos_des(1) << " " << Pos_des(2) << " " 
            << current_pos(0) << " " << current_pos(1) << " " << current_pos(2) << " "
            << Vel_des(0) << " " << Vel_des(1) << " " << Vel_des(2) << " " 
            << current_vel(0) << " " << current_vel(1) << " " << current_vel(2) << " "
            << angle_des(0) << " " << angle_des(1) << " " << Yaw_des << " " 
            << euler_angle(0) << " " << euler_angle(1) << " " << Yaw_cur << " "
            << Angle_error(0) << " " << Angle_error(1) << " " << Angle_error(2) << " "
            << Acc_des(0) << " " << Acc_des(1) << " " << Acc_des(2) << " "
            << Torque_des(0) << " " << Torque_des(1) << " " << Torque_des(2) << " "
            << new_z_body(0) << " " << new_z_body(1) << " " << new_z_body(2) << " " 
            << RPM_output(0) << " " << RPM_output(1) << " " << RPM_output(2) << " " << RPM_output(3) << std::endl;
}

Vector3d cascadePID::getTorquedes()
{
    return Torque_des;
}

Vector4d cascadePID::getRPMoutputs()
{
    return RPM_output;
}