/*
 * RoK-3 Gazebo Simulation Code 
 * 
 * Robotics & Control Lab.
 * 
 * Master : BKCho
 * First developer : Yunho Han
 * Second developer : Minho Park
 * 
 * ======
 * Update date : 2022.03.16 by Yunho Han
 * ======
 */
//* Header file for C++
#include <stdio.h>
#include <iostream>
#include <boost/bind.hpp>



//* Header file for Gazebo and Ros
#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <functional>
#include <ignition/math/Vector3.hh>

//* Header file for RBDL and Eigen
#include <rbdl/rbdl.h> // Rigid Body Dynamics Library (RBDL)
#include <rbdl/addons/urdfreader/urdfreader.h> // urdf model read using RBDL
#include <Eigen/Dense> // Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.

#define PI      3.141592
#define D2R     PI/180.
#define R2D     180./PI

//Print color
#define C_BLACK   "\033[30m"
#define C_RED     "\x1b[91m"
#define C_GREEN   "\x1b[92m"
#define C_YELLOW  "\x1b[93m"
#define C_BLUE    "\x1b[94m"
#define C_MAGENTA "\x1b[95m"
#define C_CYAN    "\x1b[96m"
#define C_RESET   "\x1b[0m"

//Eigen//
using Eigen::MatrixXd;
using Eigen::VectorXd;

//RBDL//
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using namespace std;


namespace gazebo
{

    class rok3_plugin : public ModelPlugin
    {
        //*** Variables for RoK-3 Simulation in Gazebo ***//
        //* TIME variable
        common::Time last_update_time;
        event::ConnectionPtr update_connection;
        double dt;
        double time = 0;

        //* Model & Link & Joint Typedefs
        physics::ModelPtr model;

        physics::JointPtr L_Hip_yaw_joint;
        physics::JointPtr L_Hip_roll_joint;
        physics::JointPtr L_Hip_pitch_joint;
        physics::JointPtr L_Knee_joint;
        physics::JointPtr L_Ankle_pitch_joint;
        physics::JointPtr L_Ankle_roll_joint;

        physics::JointPtr R_Hip_yaw_joint;
        physics::JointPtr R_Hip_roll_joint;
        physics::JointPtr R_Hip_pitch_joint;
        physics::JointPtr R_Knee_joint;
        physics::JointPtr R_Ankle_pitch_joint;
        physics::JointPtr R_Ankle_roll_joint;
        physics::JointPtr torso_joint;

        physics::JointPtr LS, RS;

        // 25.05.26
        bool ik_initialized             = false;                            // IK initialized flag
        VectorXd theta_left_nonsingular = VectorXd::Zero(6);                // Non-singular solution for left foot IK
        VectorXd theta_right_nonsingular = VectorXd::Zero(6);               // Non-singular solution for right foot IK
        VectorXd theta_init_left        = VectorXd::Zero(6);                // Initial guess for left foot IK
        VectorXd theta_init_right       = VectorXd::Zero(6);                // Initial guess for right foot IK
        VectorXd theta_final_left       = VectorXd::Zero(6);                // Final value for left foot IK
        VectorXd theta_final_right      = VectorXd::Zero(6);                // Final value for right foot IK

        VectorXd theta_left_6_2_down    = VectorXd::Zero(6);                // Left foot IK solution for 6-2 down
        VectorXd theta_right_6_2_down   = VectorXd::Zero(6);                // Right foot IK solution for 6-2 down

        VectorXd theta_left_6_2_up      = VectorXd::Zero(6);                // Left foot IK solution for 6-2 up
        VectorXd theta_right_6_2_up     = VectorXd::Zero(6);                // Right foot IK solution for 6-2 up

        VectorXd init_degrees_left      = VectorXd::Zero(6);                // Initial joint angles for left foot
        VectorXd init_degrees_right     = VectorXd::Zero(6);                // Initial joint angles for right foot

        // Set Variable for usage
        //  double dt                      = 0.0;                              // Time step
        VectorXd q_des_left_           = VectorXd::Zero(6);                // Desired Left joint angles, Calculated by IK
        VectorXd q_des_right_          = VectorXd::Zero(6);                // Desired Left joint angles, Calculated by IK
        VectorXd q_init_               = VectorXd::Zero(13);             // Initial guess for inverse kinematics
        VectorXd q_des_joint_          = VectorXd::Zero(13);             // Desired joint angles, including base position and orientation

        Vector3d r0_L, r1_L, r2_L, r3_L;
        Vector3d r0_R, r1_R, r2_R, r3_R;
        VectorXd q_guess_L, q_guess_R;

        Vector3d r_des_L, r_des_R;


        //* Index setting for each joint
        
        enum
        {
            WST = 0, LHY, LHR, LHP, LKN, LAP, LAR, RHY, RHR, RHP, RKN, RAP, RAR
        };

        //* Joint Variables
        int nDoF; // Total degrees of freedom, except position and orientation of the robot

        typedef struct RobotJoint //Joint variable struct for joint control 
        {
            double targetDegree; //The target deg, [deg]
            double targetRadian; //The target rad, [rad]

            double targetVelocity; //The target vel, [rad/s]
            double targetTorque; //The target torque, [N·m]

            double actualDegree; //The actual deg, [deg]
            double actualRadian; //The actual rad, [rad]
            double actualVelocity; //The actual vel, [rad/s]
            double actualRPM; //The actual rpm of input stage, [rpm]
            double actualTorque; //The actual torque, [N·m]

            double Kp;
            double Ki;
            double Kd;

        } ROBO_JOINT;
        ROBO_JOINT* joint;

    public:
        //*** Functions for RoK-3 Simulation in Gazebo ***//
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/); // Loading model data and initializing the system before simulation 
        void UpdateAlgorithm(); // Algorithm update while simulation

        void jointController(); // Joint Controller for each joint

        void GetJoints(); // Get each joint data from [physics::ModelPtr _model]
        void GetjointData(); // Get encoder data of each joint

        void initializeJoint(); // Initialize joint variables for joint control
        void SetJointPIDgain(); // Set each joint PID gain for joint control
    };
    GZ_REGISTER_MODEL_PLUGIN(rok3_plugin);
}

/*--------------------------------------------*/
// Link 1, Link2, Link3 length 1m
// q(0) = 10deg, q(1) = 20deg, q(3) = 30(deg)



// MatrixXd getTransformI0()
// {

//     MatrixXd temp_matrix = MatrixXd::Identity(4,4);
     

//     return temp_matrix;
// }

// MatrixXd jointToTransform01(VectorXd q)
// {
//     MatrixXd temp_matrix = MatrixXd::Identity(4,4);

//     temp_matrix(0,0) = cos(q(0)); temp_matrix(0, 2) = sin(q(0));
//     temp_matrix(2,0) = -sin(q(0)); temp_matrix(2,2) = cos(q(0));
//     temp_matrix(2,3) = 1.0;

//     return temp_matrix;
// }

// MatrixXd jointToTransform12(VectorXd q)
// {
//     MatrixXd temp_matrix = MatrixXd::Identity(4,4);

//     temp_matrix(0,0) = cos(q(1)); temp_matrix(0, 2) = sin(q(1));
//     temp_matrix(2,0) = -sin(q(1)); temp_matrix(2,2) = cos(q(1));
//     temp_matrix(2,3) = 1.0;

//     return temp_matrix;
// }

// MatrixXd jointToTransform23(VectorXd q)
// {
//     MatrixXd temp_matrix = MatrixXd::Identity(4,4);

//     temp_matrix(0,0) = cos(q(2)); temp_matrix(0, 2) = sin(q(2));
//     temp_matrix(2,0) = -sin(q(2)); temp_matrix(2,2) = cos(q(2));
//     temp_matrix(2,3) = 1.0;


//     return temp_matrix;
// }

// MatrixXd jointToTransform34(VectorXd q)
// {
//     MatrixXd temp_matrix = MatrixXd::Identity(4,4);

//     temp_matrix(0,0) = cos(q(2)); temp_matrix(0, 2) = sin(q(2));
//     temp_matrix(2,0) = -sin(q(2)); temp_matrix(2,2) = cos(q(2));
//     temp_matrix(2,3) = 1.0;


//     return temp_matrix;
// }

// MatrixXd jointToTransform45(VectorXd q)
// {
//     MatrixXd temp_matrix = MatrixXd::Identity(4,4);

//     temp_matrix(0,0) = cos(q(2)); temp_matrix(0, 2) = sin(q(2));
//     temp_matrix(2,0) = -sin(q(2)); temp_matrix(2,2) = cos(q(2));
//     temp_matrix(2,3) = 1.0;


//     return temp_matrix;
// }

// MatrixXd jointToTransform56(VectorXd q)
// {
//     MatrixXd temp_matrix = MatrixXd::Identity(4,4);

//     temp_matrix(0,0) = cos(q(2)); temp_matrix(0, 2) = sin(q(2));
//     temp_matrix(2,0) = -sin(q(2)); temp_matrix(2,2) = cos(q(2));
//     temp_matrix(2,3) = 1.0;


//     return temp_matrix;
// }


// MatrixXd getTransform6E(VectorXd q)
// {
//     MatrixXd temp_matrix = MatrixXd::Identity(4,4);

//     temp_matrix(0,0) = cos(q(2)); temp_matrix(0, 2) = sin(q(2));
//     temp_matrix(2,0) = -sin(q(2)); temp_matrix(2,2) = cos(q(2));
//     temp_matrix(2,3) = 1.0;


//     return temp_matrix;
// }



// MatrixXd getTransform3E()
// {

//     MatrixXd temp_matrix = MatrixXd::Identity(4,4);

//     temp_matrix(2,3) = 1.0;

//     return temp_matrix;
// }




/*--------------------------------------------------------------------------------------------------*/
// Practice 2
// Homogeneous Transformation Matrix , Foward Kinematics
MatrixXd getTransformI0(const VectorXd& q)
{
    MatrixXd temp_matrix = MatrixXd::Identity(4,4);


    return temp_matrix;
}



MatrixXd jointToTransform01(const VectorXd& q)
{
    MatrixXd temp_matrix = MatrixXd::Identity(4,4);
    
    temp_matrix(0,0) = cos(q(0)); temp_matrix(0,1) = -sin(q(0));
    temp_matrix(1,0) = sin(q(0)); temp_matrix(1,1) = cos(q(0));
    temp_matrix(1,3) = 0.105;     temp_matrix(2,3) = -0.1512;

    return temp_matrix;

}
MatrixXd jointToTransform12(const VectorXd& q)
{
    MatrixXd temp_matrix = MatrixXd::Identity(4,4);
    temp_matrix(1,1) = cos(q(1)); temp_matrix(1,2) = -sin(q(1));
    temp_matrix(2,1) = sin(q(1)); temp_matrix(2,2) = cos(q(1));

    return temp_matrix;
}


MatrixXd jointToTransform23(const VectorXd& q)
{
    MatrixXd temp_matrix = MatrixXd::Identity(4,4);
    temp_matrix(0,0) = cos(q(2)); temp_matrix(0,2) = sin(q(2));
    temp_matrix(2,0) = -sin(q(2)); temp_matrix(2,2) = cos(q(2));

    return temp_matrix;
}

MatrixXd jointToTransform34(const VectorXd& q)
{
    MatrixXd temp_matrix = MatrixXd::Identity(4,4);
    temp_matrix(0,0) = cos(q(3)); temp_matrix(0,2) = sin(q(3));
    temp_matrix(2,0) = -sin(q(3)); temp_matrix(2,2) = cos(q(3));
    temp_matrix(2,3) = -0.35;

    return temp_matrix;
}
MatrixXd jointToTransform45(const VectorXd& q)
{
    MatrixXd temp_matrix = MatrixXd::Identity(4,4);
    temp_matrix(0,0) = cos(q(4)); temp_matrix(0,2) = sin(q(4));
    temp_matrix(2,0) = -sin(q(4)); temp_matrix(2,2) = cos(q(4));
    temp_matrix(2,3) = -0.35;

    return temp_matrix;
}
MatrixXd jointToTransform56(const VectorXd& q)
{
    MatrixXd temp_matrix = MatrixXd::Identity(4,4);
    temp_matrix(1,1) = cos(q(5)); temp_matrix(1,2) = -sin(q(5));
    temp_matrix(2,1) = sin(q(5)); temp_matrix(2,2) = cos(q(5));

    return temp_matrix;
}
MatrixXd getTransform6E(const VectorXd& q)
{
    MatrixXd temp_matrix = MatrixXd::Identity(4,4);
    temp_matrix(2,3) = -0.09;

    return temp_matrix;
}

VectorXd jointToPosition(const VectorXd& q)
{
    MatrixXd TI0(4,4), T6E(4,4), T0E(4,4);
    
    TI0 = getTransformI0(q);
    T6E = getTransform6E(q);

    T0E = TI0*jointToTransform01(q)*jointToTransform12(q)*jointToTransform23(q)*jointToTransform34(q)*jointToTransform45(q)*jointToTransform56(q)*T6E;

    return T0E.block<3,1>(0,3);
}

MatrixXd jointToRotMat(const VectorXd& q)
{
    MatrixXd TI0(4,4), T6E(4,4), T0E(4,4);
    
    TI0 = getTransformI0(q);
    T6E = getTransform6E(q);

    T0E = TI0*jointToTransform01(q)*jointToTransform12(q)*jointToTransform23(q)*jointToTransform34(q)*jointToTransform45(q)*jointToTransform56(q)*T6E;

    return T0E.block<3,3>(0,0);
}
VectorXd rotToEuler(MatrixXd rotMat, const VectorXd& q)
{
    MatrixXd TI0(4,4), T6E(4,4), T0E(4,4);
    VectorXd euler_rot(3);
    
    TI0 = getTransformI0(q);
    T6E = getTransform6E(q);

    T0E = TI0*jointToTransform01(q)*jointToTransform12(q)*jointToTransform23(q)*jointToTransform34(q)*jointToTransform45(q)*jointToTransform56(q)*T6E;

    euler_rot(0) = atan2(rotMat(1,0), rotMat(0,0));
    euler_rot(1) = atan2(-rotMat(2,0), sqrt(rotMat(2,1)*rotMat(2,1) + (rotMat(2,2)*rotMat(2,2))));
    euler_rot(2) = atan2(rotMat(2,1), rotMat(2,2));

    return euler_rot;
}   

MatrixXd jointToPosJac(VectorXd q)
{   

    // Input: vector of generalized coordinates (joint angles)
    // Output: J_P, Jacobian of the end-effector translation which maps joint velocities to end-effector linear velocities in I frame.
    MatrixXd J_P = MatrixXd::Zero(3,6);
    MatrixXd T_I0(4,4), T_01(4,4), T_12(4,4), T_23(4,4), T_34(4,4), T_45(4,4), T_56(4,4), T_6E(4,4);
    MatrixXd T_I1(4,4), T_I2(4,4), T_I3(4,4), T_I4(4,4), T_I5(4,4), T_I6(4,4);
    MatrixXd R_I1(3,3), R_I2(3,3), R_I3(3,3), R_I4(3,3), R_I5(3,3), R_I6(3,3);
    Vector3d r_I_I1, r_I_I2, r_I_I3, r_I_I4, r_I_I5, r_I_I6;
    Vector3d n_1, n_2, n_3, n_4, n_5, n_6;
    Vector3d n_I_1,n_I_2,n_I_3,n_I_4,n_I_5,n_I_6;
    Vector3d r_I_IE;


    //* Compute the relative homogeneous transformation matrices.
    T_I0 = MatrixXd::Identity(4,4);
    T_01 = jointToTransform01(q);
    T_12 = jointToTransform12(q);
    T_23 = jointToTransform23(q);
    T_34 = jointToTransform34(q);
    T_45 = jointToTransform45(q);
    T_56 = jointToTransform56(q);
    T_6E = getTransform6E(q);

    //* Compute the homogeneous transformation matrices from frame k to the inertial frame I.
    T_I1 = T_I0*jointToTransform01(q);
    T_I2 = T_I0*jointToTransform01(q)*jointToTransform12(q);
    T_I3 = T_I0*jointToTransform01(q)*jointToTransform12(q)*jointToTransform23(q);
    T_I4 = T_I0*jointToTransform01(q)*jointToTransform12(q)*jointToTransform23(q)*jointToTransform34(q);
    T_I5 = T_I0*jointToTransform01(q)*jointToTransform12(q)*jointToTransform23(q)*jointToTransform34(q)*jointToTransform45(q);
    T_I6 = T_I0*jointToTransform01(q)*jointToTransform12(q)*jointToTransform23(q)*jointToTransform34(q)*jointToTransform45(q)*jointToTransform56(q);

    //* Extract the rotation matrices from each homogeneous transformation matrix. Use sub-matrix of EIGEN. https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
    R_I1 = T_I1.block<3,3>(0,0);
    R_I2 = T_I2.block<3,3>(0,0);
    R_I3 = T_I3.block<3,3>(0,0);
    R_I4 = T_I4.block<3,3>(0,0);
    R_I5 = T_I5.block<3,3>(0,0);
    R_I6 = T_I6.block<3,3>(0,0);

    //* Extract the position vectors from each homogeneous transformation matrix. Use sub-matrix of EIGEN.
    r_I_I1 = T_I1.block<3,1>(0,3);
    r_I_I2 = T_I2.block<3,1>(0,3);
    r_I_I3 = T_I3.block<3,1>(0,3);
    r_I_I4 = T_I4.block<3,1>(0,3);
    r_I_I5 = T_I5.block<3,1>(0,3);
    r_I_I6 = T_I6.block<3,1>(0,3);

    //* Define the unit vectors around which each link rotate in the precedent coordinate frame.
    n_1 << 0,0,1;
    n_2 << 1,0,0;
    n_3 << 0,1,0;
    n_4 << 0,1,0;
    n_5 << 0,1,0;
    n_6 << 1,0,0;

    //* Compute the unit vectors for the inertial frame I.
    n_I_1 = R_I1*n_1;
    n_I_2 = R_I2*n_2;
    n_I_3 = R_I3*n_3;
    n_I_4 = R_I4*n_4;
    n_I_5 = R_I5*n_5;
    n_I_6 = R_I6*n_6;

    //* Compute the end-effector position vector.
    r_I_IE = (T_I6 * T_6E).block<3,1>(0,3);


    //* Compute the translational Jacobian. Use cross of EIGEN.
    J_P.col(0) << n_I_1.cross(r_I_IE-r_I_I1);
    J_P.col(1) << n_I_2.cross(r_I_IE-r_I_I2);
    J_P.col(2) << n_I_3.cross(r_I_IE-r_I_I3);
    J_P.col(3) << n_I_4.cross(r_I_IE-r_I_I4);
    J_P.col(4) << n_I_5.cross(r_I_IE-r_I_I5);
    J_P.col(5) << n_I_6.cross(r_I_IE-r_I_I6);

    // std::cout << "Test, JP:" << std::endl << J_P << std::endl;

    return J_P;
}


MatrixXd jointToRotJac(VectorXd& q)
{
   // Input: vector of generalized coordinates (joint angles)
    // Output: J_R, Jacobian of the end-effector orientation which maps joint velocities to end-effector angular velocities in I frame.
    MatrixXd J_R(3,6);
    MatrixXd T_I0(4,4), T_01(4,4), T_12(4,4), T_23(4,4), T_34(4,4), T_45(4,4), T_56(4,4), T_6E(4,4);
    MatrixXd T_I1(4,4), T_I2(4,4), T_I3(4,4), T_I4(4,4), T_I5(4,4), T_I6(4,4);
    MatrixXd R_I1(3,3), R_I2(3,3), R_I3(3,3), R_I4(3,3), R_I5(3,3), R_I6(3,3);
    Vector3d n_1, n_2, n_3, n_4, n_5, n_6;
    Vector3d n_I_1,n_I_2,n_I_3,n_I_4,n_I_5,n_I_6;

    //* Compute the relative homogeneous transformation matrices.
    T_I0 = MatrixXd::Identity(4,4);
    T_01 = jointToTransform01(q);
    T_12 = jointToTransform12(q);
    T_23 = jointToTransform23(q);
    T_34 = jointToTransform34(q);
    T_45 = jointToTransform45(q);
    T_56 = jointToTransform56(q);
    T_6E = getTransform6E(q);

    //* Compute the homogeneous transformation matrices from frame k to the inertial frame I.
    T_I1 = T_I0*jointToTransform01(q);
    T_I2 = T_I0*jointToTransform01(q)*jointToTransform12(q);
    T_I3 = T_I0*jointToTransform01(q)*jointToTransform12(q)*jointToTransform23(q);
    T_I4 = T_I0*jointToTransform01(q)*jointToTransform12(q)*jointToTransform23(q)*jointToTransform34(q);
    T_I5 = T_I0*jointToTransform01(q)*jointToTransform12(q)*jointToTransform23(q)*jointToTransform34(q)*jointToTransform45(q);
    T_I6 = T_I0*jointToTransform01(q)*jointToTransform12(q)*jointToTransform23(q)*jointToTransform34(q)*jointToTransform45(q)*jointToTransform56(q);

    //* Extract the rotation matrices from each homogeneous transformation matrix.
    R_I1 = T_I1.block<3,3>(0,0);
    R_I2 = T_I2.block<3,3>(0,0);
    R_I3 = T_I3.block<3,3>(0,0);
    R_I4 = T_I4.block<3,3>(0,0);
    R_I5 = T_I5.block<3,3>(0,0);
    R_I6 = T_I6.block<3,3>(0,0);

    //* Define the unit vectors around which each link rotate in the precedent coordinate frame.
    n_1 << 0,0,1;
    n_2 << 1,0,0;
    n_3 << 0,1,0;
    n_4 << 0,1,0;
    n_5 << 0,1,0;
    n_6 << 1,0,0;

    //* Compute the translational Jacobian.
    J_R.col(0) << R_I1*n_1;
    J_R.col(1) << R_I2*n_2;
    J_R.col(2) << R_I3*n_3;
    J_R.col(3) << R_I4*n_4;
    J_R.col(4) << R_I5*n_5;
    J_R.col(5) << R_I6*n_6;


    // std::cout << "Test, J_R:" << std::endl << J_R << std::endl;

    return J_R;
}

MatrixXd pseudoInverseMat(MatrixXd A, double lambda)
{
    // Input Any Mat m-by-n matrix
    // Output Any Mat n-by-m pseudo-inverse of the input according to Moore-Penrose Formula
    // for damped pseudo-inverse
    // Left Damped Pseudo-Inverse 
    // [(A^T A + lambda ^2 * I_{n*n})^{-1} * A^T]
    // Right Damped Pseudo-Inverse
    // [A^T (A * A^T + lambda^2 I_{m*m})^{-1}]
    MatrixXd pinvA;
    int m = A.rows();
    int n = A.cols();

    if (m >= n)
    {
        // Left Damped Pseudo-Inverse
        MatrixXd I = MatrixXd::Identity(n,n);
        pinvA = (A.transpose() * A + lambda * lambda * I).inverse() * A.transpose();
    }
    else
    {
        // Right Damped Pseudo-Inverse
        MatrixXd I = MatrixXd::Identity(m,m);
        pinvA = A.transpose() * (A * A.transpose() + lambda * lambda * I).inverse();
    }


    return pinvA;
}

VectorXd rotMatToRotVec(MatrixXd C)
{
    // Input : a rotation Matrix C
    // Output : the rotational Vector which describes the rotation C
    Vector3d phi, n;
    double th;

    th = acos((C(0,0) + C(1, 1) + C(2, 2) - 1) / 2.0); // Calc Rotation Angle


    if (fabs(th) < 0.001) {
            n << 0,0,0;  // If too small -> no rotation -> calc as rotates zero
    }
    else
    {
        n(0) = (C(2,1) - C(1,2)) / (2 * sin(th));
        n(1) = (C(0,2) - C(2,0)) / (2 * sin(th));
        n(2) = (C(1,0) - C(0,1)) / (2 * sin(th));

    }

    phi = th * n;

    return phi;
}


VectorXd inverseKinematics(const VectorXd& r_des, const MatrixXd& C_des, const VectorXd& q0, double tol)
{
    // Input  : desired eef position, desired eef orientation, initial guess for joint angles, threshold for the stopping-criterion
    // Output : joint angles which match desired eef position and orientation
    //* Set maximum number of iterations
    const double max_it = 200;
    //* Damping Factor
    const double lambda = 0.001;

    //* Initialize the solution with initial guess
    int num_it = 0;


    // 1) Initialize first err
    
    VectorXd q  = q0;
    // VectorXd dq = VectorXd::Zero(6);
    VectorXd dXe = VectorXd::Zero(6);

    Vector3d dr = r_des - jointToPosition(q);
    MatrixXd C_IE = jointToRotMat(q);
    MatrixXd C_err =C_des * C_IE.transpose();
    Vector3d dph = rotMatToRotVec(C_err);
    // VectorXd dXe(6);
    dXe << dr(0), dr(1), dr(2), 
           dph(0), dph(1), dph(2);
    

    ////////////////////////////////////////////////
    //** Iterative inverse kinematics
    ////////////////////////////////////////////////

    //* Iterate util terminating condition
    while (num_it < max_it && dXe.norm()>tol)
    {
        MatrixXd J(6,6), J_P(3, 6), J_R(3, 6);
        // Compute Inverse Jacobian
        J_P = jointToPosJac(q);
        J_R = jointToRotJac(q);
        J.setZero();
        J.block<3,6>(0,0) = J_P;
        J.block<3,6>(3,0) = J_R;  // Geometric Jacobian
        

        // Convert to Geometric Jacobian to Analytic Jacobian
        MatrixXd J_plus = pseudoInverseMat(J, lambda);

        // Error Vector
        dr = r_des -jointToPosition(q);
        C_IE = jointToRotMat(q);
        C_err = C_des * C_IE.transpose();
        dph = rotMatToRotVec(C_err);
        dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);



        // Update q
        VectorXd dq = J_plus * dXe;
        q += 0.5 * dq;

        num_it++;

    }

    // std::cout << "iteration: " << num_it << ", Value" << q << std::endl;


    return q;
}









void Practice()
{   
    // Update time 1ms
    // common::Time current_time = model->GetWorld()->SimTime();
    // double dt = current_time.Double() - last_update_time.Double();

    // time = time + dt;

    // //* setting for getting dt at next step
    // last_update_time = current_time;

    // if (time < 5.)
    // joint[0].targetRadian = poly3(time, 0, 목표, 5);


    VectorXd q = VectorXd::Zero(6);
    // MatrixXd T0E(4,4), TI0(4,4), T3E(4,4); // Practice 1
    MatrixXd T0E(4,4), TI0(4,4), T6E(4,4);
    MatrixXd C0E(3,3);
    VectorXd pos, euler;
    MatrixXd rotMat;


    // std::cout << "\033"
    /* Setting For q */
    // Set q for degree
    q << 10, 20, 30, 40, 50, 60;
    // q << 10, 10, 10, 10, 10, 10;
    q = q*D2R; // change deg to radian




    ////////////////////////////////////////////////////////////////////////////////
    //                                Project 2                                   //
    ////////////////////////////////////////////////////////////////////////////////
    {
        std::cout << C_BLUE << "\n===================== [ Project 2: FK, Euler ] =====================\n" << C_RESET;

        VectorXd pos = jointToPosition(q);
        MatrixXd rotMat = jointToRotMat(q);
        VectorXd eulerZYX = rotToEuler(rotMat, q) * R2D;

        std::cout << C_GREEN << "[End-Effector Position (x y z) in m]\n" << C_RESET;
        std::cout << pos.transpose() << "\n\n";

        std::cout << C_GREEN << "[Rotation Matrix (3x3)]\n" << C_RESET;
        std::cout << rotMat << "\n\n";

        std::cout << C_GREEN << "[Euler Angles (ZYX, deg)]\n" << C_RESET;
        std::cout << eulerZYX.transpose() << "\n";
    }














    // TI0 = getTransformI0(q);
    // T3E = getTransform3E(); // Practice 01
    // T6E = getTransform6E(q);


    // T0E = TI0*jointToTransform01(q)*jointToTransform12(q)*jointToTransform23(q)*T3E; // Practice 01
    // T0E = TI0*jointToTransform01(q)*jointToTransform12(q)*jointToTransform23(q)*jointToTransform34(q)*jointToTransform45(q)*jointToTransform56(q)*T6E;
    // C0E = T0E.block<3,3>(0, 0);
    // rotMat = T0E.block<3,3>(0,0);
    // pos = jointToPosition(q);
    // pos = T0E.block<3,1>(0, 3);

    // euler = rotMat.eulerAngles(2, 1, 0) * R2D;
    // rotMat = jointToRotMat(q);
    // euler = rotToEuler(rotMat);

    // cout << "position = " << pos << endl;
    // cout << "Rotation Matrix = " << rotMat << endl;
    // cout << "EulerZYX = " << euler.transpose() << endl; 

    // cout << "Position = " << jointToPosition(q) << endl;
    // cout << "Rotation Matrix = " << jointToRotMat(q) << endl;
    // cout << "Euler ZYX = " << rotToEuler(jointToRotMat(q), q) << endl;

    // cout << "Translational Jacobian" << endl;
    // jointToPosJac(q);

    // cout << "Rotational Jacobian" << endl;
    // jointToRotJac(q);

    // * Practice 4 25.05.07 //
    // Kimgyoomin //
    // 1. Desired Angle (Deg -> Ang)
    // VectorXd q_des(6);
    // q_des << 10, 20, 30, 40, 50, 60;
    // q_des *= D2R;

    // 2. Initial Joint Angle (q_init = 0.5 * q_des)
    VectorXd q_init = 0.5 * q;

    // 3. Desired Position and Angle
    VectorXd x_des = jointToPosition(q);
    MatrixXd C_des = jointToRotMat(q);

    // 4. Present Position and Angle
    VectorXd x_init = jointToPosition(q_init);
    MatrixXd C_init = jointToRotMat(q_init);

    // 5. Calc error between des, init
    VectorXd dx = x_des - x_init;
    MatrixXd C_err = C_des * C_init.transpose();
    VectorXd dph = rotMatToRotVec(C_err); // Rotation error

    // 6. Jacobian Calc
    MatrixXd Jp = jointToPosJac(q);
    MatrixXd Jr = jointToRotJac(q);
    MatrixXd J_full(6,6);
    J_full << Jp,
              Jr;


    // 7. Pseudo-Inverse of Jacobian
    double lambda = 0.001;
    MatrixXd pinvJ = pseudoInverseMat(J_full, lambda);

    // 8. Lets' get output of Geometric Jacobian, Pseudo-inverse, dph
    std::cout << C_CYAN << "Geometric Jacobain \n" << C_RESET << std::endl;
    std::cout << J_full << std::endl;
    std::cout << C_MAGENTA << "Pseudo-Inverse \n"   << C_RESET << std::endl;
    std::cout << pinvJ  << std::endl;
    std::cout << C_YELLOW << "Rotation Error Vector(dph)\n" << C_RESET << std::endl;
    std::cout << dph.transpose() << std::endl; 




    ////////////////////////////////////////////////////////////////////////////////
    //                                Project 5-1                                 //
    ////////////////////////////////////////////////////////////////////////////////
    VectorXd r_des, q_calc;
    {
        r_des = jointToPosition(q);
        C_des = jointToRotMat(q);

        q_calc = inverseKinematics(r_des, C_des, q*0.5, 0.001);

        std::cout << "Calculated q" << q_calc.transpose() << std::endl;
    }


    ////////////////////////////////////////////////////////////////////////////////
    //                                Project 5-2                                 //
    ////////////////////////////////////////////////////////////////////////////////
    // Desired Pos for Base -> Desired_Pos = [0; 0.105; -0.55] & Desired Oreintation Base
    // Result for each joint position = [0; 0; -63.756; 127.512; -63.756; 0]
    {
        // 1) Desired End-Effector Position (foot가 아니라 base를 향하도록 설정)
        VectorXd r_des(3);
        r_des << 0.0, 0.105, -0.55;

        // 2) Desired End-Effector Orientation = Base frame
        Matrix3d C_des = Matrix3d::Identity();

        // 3) 초기 추정 관절각 (예: 전단계에서 구한 q*0.5)
        VectorXd q0 = q * 0.5;  

        // 4) IK 호출
        VectorXd q_calc = inverseKinematics(r_des, C_des, q0, 0.001);

        // 5) 결과 출력
        std::cout << C_MAGENTA
                  << "[Project 5-2] IK result for Base → Pos = [0,0.105,-0.55], Ori = I\n"
                  << C_RESET;
        std::cout << "q_calc (rad): " << q_calc.transpose() << std::endl;
        std::cout << "q_calc (deg): ";
        for (int i = 0; i < q_calc.size(); ++i) {
            std::cout << q_calc[i] * R2D << (i+1<q_calc.size()? "  " : "\n");
        }
    }



}



void gazebo::rok3_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    /*
     * Loading model data and initializing the system before simulation 
     */

    //* model.sdf file based model data input to [physics::ModelPtr model] for gazebo simulation
    model = _model;

    //* [physics::ModelPtr model] based model update
    GetJoints();



    //* RBDL API Version Check
    int version_test;
    version_test = rbdl_get_api_version();
    printf(C_GREEN "RBDL API version = %d\n" C_RESET, version_test);

    //* model.urdf file based model data input to [Model* rok3_model] for using RBDL
    Model* rok3_model = new Model();
    Addons::URDFReadFromFile("/home/kim/.gazebo/models/rok3_model/urdf/rok3_model.urdf", rok3_model, true, true);
    //↑↑↑ Check File Path ↑↑↑
    nDoF = rok3_model->dof_count - 6; // Get degrees of freedom, except position and orientation of the robot
    joint = new ROBO_JOINT[nDoF]; // Generation joint variables struct

    //* initialize and setting for robot control in gazebo simulation
    initializeJoint();
    SetJointPIDgain();

        // Set initial joint angles to avoid singularity
    r0_L = Eigen::Vector3d(0.0, 0.105, -0.9); // Initial position of left foot
    r0_R = Eigen::Vector3d(0.0, 0.105, -0.9); // Initial position of right foot

    r1_L = Eigen::Vector3d(0.0, 0.105, -0.55); // Position of left foot at the end of the first phase
    r1_R = Eigen::Vector3d(0.0, 0.105, -0.55); // Position of right foot at the end of the first phase

    r2_L = Eigen::Vector3d(0.0, 0.105, -0.35); // Position of left foot at the end of the second phase
    r2_R = Eigen::Vector3d(0.0, 0.105, -0.35); // Position of right foot at the end of the second phase

    // IK initialization
    q_guess_L = VectorXd::Zero(6); // Initial guess for left foot IK
    q_guess_R = VectorXd::Zero(6); // Initial guess for right foot IK
    //* setting for getting dt
    last_update_time = model->GetWorld()->SimTime();
    update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&rok3_plugin::UpdateAlgorithm, this));

    Practice();

}

// Polynomial Trajectory
// Make t time for 3th 
double poly3(double t, double init, double final, double tf)
{
    double tmp;
    double a0, a1, a2, a3;

    a0      = init;
    a1      = 0.;
    a2      = 3 * (final - init) / (tf * tf);
    a3      = -2 * (final - init) / (tf * tf * tf);


    tmp     = a0 
            + a1 * t
            + a2 * t * t
            + a3 * t * t* t;

    return tmp;
}

// for 3D interpolation using poly3
static Eigen::Vector3d poly3Vec(double t, 
                                const Eigen::Vector3d& p0, 
                                const Eigen::Vector3d& p1, 
                                double tf)
{
  return Eigen::Vector3d(
    poly3(t, p0.x(), p1.x(), tf),
    poly3(t, p0.y(), p1.y(), tf),
    poly3(t, p0.z(), p1.z(), tf)

  );
}



void gazebo::rok3_plugin::UpdateAlgorithm()
{
    /*
     * Algorithm update while simulation
     */
    //* UPDATE TIME : 1ms
    common::Time current_time = model->GetWorld()->SimTime();
    dt = current_time.Double() - last_update_time.Double();

    // IK 초기 추정치 (지역 변수로 사용해도 무방, 또는 멤버 this->q_init_ 사용)
    VectorXd local_q_ik_guess = VectorXd::Zero(6);
    local_q_ik_guess << 0., 0., -63.756, 127.512, -63.756, 0.; // "Walk Ready" 자세
    local_q_ik_guess *= D2R;

    VectorXd q_sol_L(6), q_sol_R(6); // Variable for storing IK solutions for left and right feet

    if(!this->ik_initialized) // If IK is not initialized
    {   

        Eigen::Vector3d r_ground_left_nonsing(0.0, 0.105, -0.9);
        Eigen::Vector3d r_ground_right_nonsing(0.0, 0.105, -0.9);

        Eigen::Vector3d r_ground_left_vec(0.0, 0.105, -0.55);
        Eigen::Vector3d r_ground_right_vec(0.0, 0.105, -0.55);
        Matrix3d C_des = Matrix3d::Identity();

        // 0-5초 동작의 최종 자세 (발을 든 자세)에 대한 관절각 계산 -> 멤버 변수에 저장
        Eigen::Vector3d r_lifted_left_vec(0.0, 0.105, -0.55 + 0.2);
        Eigen::Vector3d r_lifted_right_vec(0.0, 0.105, -0.55 + 0.2);


        // 10 - 15 second lift up for 0.2m within 5 second, down for 0.2m within 5 seconds
        // Eigen::Vector3d r_lift_left_6_2(0.0, 0.105, );

        this->theta_left_nonsingular  = inverseKinematics(r_ground_left_nonsing, C_des, local_q_ik_guess, 0.001);
        this->theta_right_nonsingular = inverseKinematics(r_ground_right_nonsing, C_des, local_q_ik_guess, 0.001);

        this->theta_init_left  = inverseKinematics(r_ground_left_vec, C_des, this->theta_left_nonsingular, 0.001);
        this->theta_init_right  = inverseKinematics(r_ground_right_vec, C_des, this->theta_right_nonsingular, 0.001);

        this->theta_final_left = inverseKinematics(r_lifted_left_vec, C_des, this->theta_init_left, 0.001); // 이전 IK 결과를 추정치로 사용
        this->theta_final_right = inverseKinematics(r_lifted_right_vec, C_des, this->theta_init_right, 0.001);



        // Calculate IK once and calculate desired joint angles
        for (int i = 0; i < 6; ++i)
        {   
            joint[i + 1].targetRadian = this->theta_left_nonsingular(i); // Set initial joint angles
            joint[i + 7].targetRadian = this->theta_right_nonsingular(i); // Set initial joint angles

            // joint[i + 1].targetRadian = this->theta_init_left(i); // Set initial joint angles
            // joint[i + 7].targetRadian = this->theta_init_right(i); // Set initial joint angles
        }
        

        this->ik_initialized = true; // Set IK initialized flag
    }

    if (this->ik_initialized && time <= 5.0)
    { 
      // For 0~5s : r0 ~ r1
      // double fL = poly3(time, )
      // 멤버 변수에 저장된 init/fin)
      r_des_L = poly3Vec(time, r0_L, r1_L, 5.0);
      r_des_R = poly3Vec(time, r0_R, r1_R, 5.0);
        // for (int i = 0; i < 6; ++i)
        // {   
        //     joint[i + 1].targetRadian = poly3(time, this->theta_left_nonsingular(i), this->theta_init_left(i), 5.0);
        //     joint[i + 7].targetRadian = poly3(time, this->theta_right_nonsingular(i), this->theta_init_right(i), 5.0);
        // }
        std::cout << C_YELLOW << "[IK] Heading to z = -0.55" << C_RESET << std::endl;
    }
    // For 5 second ; -0.55 -> -0.35
    else if (this->ik_initialized && (5.0 < time && time <= 10.0))
    {
        double segment_time = time - 5.0; // 5초 이후의 시간
        r_des_L = poly3Vec(segment_time, r1_L, r2_L, 5.0);
        r_des_R = poly3Vec(segment_time, r1_R, r2_R, 5.0);
        // 멤버 변수에 저장된 init/final 관절각을 사용하여 보간
        // for (int i = 0; i < 6; ++i)
        // { 
        //     joint[i + 1].targetRadian = poly3(segment_time, this->theta_init_left(i), this->theta_final_left(i), 5.0);
        //     joint[i + 7].targetRadian = poly3(segment_time, this->theta_init_right(i), this->theta_final_right(i), 5.0);
        // }
        std::cout << C_YELLOW << "[IK] Lifting Leg for 0.2m z" << C_RESET << std::endl;


    }
    // For 5 second ; -0.35 -> -0.55
    else if (this->ik_initialized && (10.0 < time && time <= 15.0))
    {
        double segment_time = time - 10.0; // 10초 이후의 시간
        r_des_L = poly3Vec(segment_time, r2_L, r1_L, 5.0);
        r_des_R = poly3Vec(segment_time, r2_R, r1_R, 5.0);
        // for (int i = 0; i < 6; ++i)
        // {
        //     joint[i + 1].targetRadian = poly3(segment_time, this->theta_final_left(i), this->theta_init_left(i), 5.0);
        //     joint[i + 7].targetRadian = poly3(segment_time, this->theta_final_right(i), this->theta_init_right(i), 5.0);

        // }
        std::cout << C_YELLOW << "[IK] Downwarding Leg for 0.2m z" << C_RESET << std::endl;
    }
    else
    {
        r_des_L = r1_L; // Maintain the final position
        r_des_R = r1_R; // Maintain the final position
    }

    q_sol_L = inverseKinematics(r_des_L, Matrix3d::Identity(), q_guess_L, 0.001); // Calculate IK for left foot
    q_sol_R = inverseKinematics(r_des_R, Matrix3d::Identity(), q_guess_R, 0.001); // Calculate IK for right foot
    q_guess_L = q_sol_L; // Update guess for next iteration
    q_guess_R = q_sol_R; // Update guess for next iteration

    for (int i = 0; i < 6 ; ++i)
    {
        joint[i + 1].targetRadian = q_sol_L(i); // Set target joint angles for left foot
        joint[i + 7].targetRadian = q_sol_R(i); // Set target joint angles for right foot
    }
    // else if (this->ik_initialized && (time > 10.0 && time <= 15.0))
    // {
    //     // After 10 seconds to 15 seconds, place leg down 0.2m for z coordinate within 5 secodns 
    //     // and place leg up 0.2m for z coordinate within 5 seconds between 10.0 and 15.0 secodns

    //     double segment_time = time - 10.0; // Time after 10 seconds
    //     for (int i = 0; i < 6; ++i)
    //     {
    //         joint[i + 1].targetRadian = poly3(segment_time, this)

    //     }




    // }


    // else if (this->ik_initialized && time > 15.0)
    // {
    //     // 5초 이후의 동작 (마지막 자세 유지 또는 다음 동작으로 전환)
    //     for (int i = 0; i < 6; ++i) {
    //         joint[i + 1].targetRadian = this->theta_final_left(i);
    //         joint[i + 7].targetRadian = this->theta_final_right(i);
    //     }
    //     std::cout << C_YELLOW << "[IK] Plz keep stand still!!!!" << C_RESET << std::endl;
    // }

    // // Poly3을 활용해서 내가 원하는 값을 여기서 알아내도록 하면 됨. -> 5초 동안!!!!
    // if (time <= 5.0)
    // { 
    //   // 5초 동안, z방향으로 0.2m 이동하기  (In Cartesian Coordinates)
    //   VectorXd r_final_left(3), r_final_right(3); // Final position for left and right foot

    //   // For 5 seconds, move in z direction by 0.2m (In Cartesian Coordinates)
    //   double z_init_left        = -0.55;                           // Initial z position for left foot
    //   double z_init_right       = -0.55;                           // Initial z position for right foot
      
    //   double z_final_left       = -0.55 + 0.2;                     // Final z position for left foot
    //   double z_final_right      = -0.55 + 0.2;                     // Final z position for right foot

    //   MatrixXd C_des            = Matrix3d::Identity();            // Desired orientation for feet, Identity matrix  
      
    //   // Initial cartesian coordinate values for left and right feet
    //   r_final_left  << 0.0, 0.105, z_final_left;        // Final position for left foot
    //   r_final_right << 0.0, -0.105, z_final_right;     // Final position for right foot
      
    //   // Initial theta values for left and right feet
    //   VectorXd theta_init_left           = q_des_left_;
    //   VectorXd theta_init_right          = q_des_right_;
      
    //   // Calculate IK for each foot
    //   theta_final_left          = inverseKinematics(r_final_left, C_des, theta_init_left*0.5, 0.001);  // Final joint angles for left foot
    //   theta_final_right         = inverseKinematics(r_final_right, C_des, theta_init_right*0.5, 0.001); // Final joint angles for right foot
        
    //   for (int i = 0; i < 6; ++i)
    //   {
    //       // Set target joint angles for left and right feet
    //       joint[i + 1].targetRadian = poly3(time, theta_init_left(i), theta_final_left(i), 5.0); // Left foot
    //       joint[i + 7].targetRadian = poly3(time, theta_init_right(i), theta_final_right(i), 5.0); // Right foot

    //       // q_des_joint_(i+1) = joint[i + 1].targetRadian;
    //       // q_des_joint_(i+7) = joint[i + 7].targetRadian;
    //   }

    // }
    // else if (time <= 10.0)
    // {
    //     // 5초 동안, z방향으로 0.2m 이동하기  (In Cartesian Coordinates)




    // }

    //* Read Sensors data, 현재 값을 알아내는 부분
    GetjointData();
    
    //* Joint Controller, 
    jointController();

    // cout << "dt:" << dt << endl;
    time = time + dt;
    // cout << "time:" << time << endl;

    //* setting for getting dt at next step
    last_update_time = current_time;



}

void gazebo::rok3_plugin::jointController()
{
    /*
     * Joint Controller for each joint
     */

    // Update target torque by control
    for (int j = 0; j < nDoF; j++) {
        joint[j].targetTorque = joint[j].Kp * (joint[j].targetRadian-joint[j].actualRadian)\
                              + joint[j].Kd * (joint[j].targetVelocity-joint[j].actualVelocity);
    }

    // Update target torque in gazebo simulation     
    L_Hip_yaw_joint->SetForce(0, joint[LHY].targetTorque);
    L_Hip_roll_joint->SetForce(0, joint[LHR].targetTorque);
    L_Hip_pitch_joint->SetForce(0, joint[LHP].targetTorque);
    L_Knee_joint->SetForce(0, joint[LKN].targetTorque);
    L_Ankle_pitch_joint->SetForce(0, joint[LAP].targetTorque);
    L_Ankle_roll_joint->SetForce(0, joint[LAR].targetTorque);

    R_Hip_yaw_joint->SetForce(0, joint[RHY].targetTorque);
    R_Hip_roll_joint->SetForce(0, joint[RHR].targetTorque);
    R_Hip_pitch_joint->SetForce(0, joint[RHP].targetTorque);
    R_Knee_joint->SetForce(0, joint[RKN].targetTorque);
    R_Ankle_pitch_joint->SetForce(0, joint[RAP].targetTorque);
    R_Ankle_roll_joint->SetForce(0, joint[RAR].targetTorque);

    torso_joint->SetForce(0, joint[WST].targetTorque);
}

void gazebo::rok3_plugin::GetJoints()
{
    /*
     * Get each joints data from [physics::ModelPtr _model]
     */

    //* Joint specified in model.sdf
    L_Hip_yaw_joint = this->model->GetJoint("L_Hip_yaw_joint");
    L_Hip_roll_joint = this->model->GetJoint("L_Hip_roll_joint");
    L_Hip_pitch_joint = this->model->GetJoint("L_Hip_pitch_joint");
    L_Knee_joint = this->model->GetJoint("L_Knee_joint");
    L_Ankle_pitch_joint = this->model->GetJoint("L_Ankle_pitch_joint");
    L_Ankle_roll_joint = this->model->GetJoint("L_Ankle_roll_joint");
    R_Hip_yaw_joint = this->model->GetJoint("R_Hip_yaw_joint");
    R_Hip_roll_joint = this->model->GetJoint("R_Hip_roll_joint");
    R_Hip_pitch_joint = this->model->GetJoint("R_Hip_pitch_joint");
    R_Knee_joint = this->model->GetJoint("R_Knee_joint");
    R_Ankle_pitch_joint = this->model->GetJoint("R_Ankle_pitch_joint");
    R_Ankle_roll_joint = this->model->GetJoint("R_Ankle_roll_joint");
    torso_joint = this->model->GetJoint("torso_joint");

    //* FTsensor joint
    LS = this->model->GetJoint("LS");
    RS = this->model->GetJoint("RS");
}

void gazebo::rok3_plugin::GetjointData()
{
    /*
     * Get encoder and velocity data of each joint
     * encoder unit : [rad] and unit conversion to [deg]
     * velocity unit : [rad/s] and unit conversion to [rpm]
     */
    joint[LHY].actualRadian = L_Hip_yaw_joint->Position(0);
    joint[LHR].actualRadian = L_Hip_roll_joint->Position(0);
    joint[LHP].actualRadian = L_Hip_pitch_joint->Position(0);
    joint[LKN].actualRadian = L_Knee_joint->Position(0);
    joint[LAP].actualRadian = L_Ankle_pitch_joint->Position(0);
    joint[LAR].actualRadian = L_Ankle_roll_joint->Position(0);

    joint[RHY].actualRadian = R_Hip_yaw_joint->Position(0);
    joint[RHR].actualRadian = R_Hip_roll_joint->Position(0);
    joint[RHP].actualRadian = R_Hip_pitch_joint->Position(0);
    joint[RKN].actualRadian = R_Knee_joint->Position(0);
    joint[RAP].actualRadian = R_Ankle_pitch_joint->Position(0);
    joint[RAR].actualRadian = R_Ankle_roll_joint->Position(0);

    joint[WST].actualRadian = torso_joint->Position(0);

    for (int j = 0; j < nDoF; j++) {
        joint[j].actualDegree = joint[j].actualRadian*R2D;
    }


    joint[LHY].actualVelocity = L_Hip_yaw_joint->GetVelocity(0);
    joint[LHR].actualVelocity = L_Hip_roll_joint->GetVelocity(0);
    joint[LHP].actualVelocity = L_Hip_pitch_joint->GetVelocity(0);
    joint[LKN].actualVelocity = L_Knee_joint->GetVelocity(0);
    joint[LAP].actualVelocity = L_Ankle_pitch_joint->GetVelocity(0);
    joint[LAR].actualVelocity = L_Ankle_roll_joint->GetVelocity(0);

    joint[RHY].actualVelocity = R_Hip_yaw_joint->GetVelocity(0);
    joint[RHR].actualVelocity = R_Hip_roll_joint->GetVelocity(0);
    joint[RHP].actualVelocity = R_Hip_pitch_joint->GetVelocity(0);
    joint[RKN].actualVelocity = R_Knee_joint->GetVelocity(0);
    joint[RAP].actualVelocity = R_Ankle_pitch_joint->GetVelocity(0);
    joint[RAR].actualVelocity = R_Ankle_roll_joint->GetVelocity(0);

    joint[WST].actualVelocity = torso_joint->GetVelocity(0);


    //    for (int j = 0; j < nDoF; j++) {
    //        cout << "joint[" << j <<"]="<<joint[j].actualDegree<< endl;
    //    }

}

void gazebo::rok3_plugin::initializeJoint()
{
    /*
     * Initialize joint variables for joint control
     */
    
    for (int j = 0; j < nDoF; j++) {
        joint[j].targetDegree = 0;
        joint[j].targetRadian = 0;
        joint[j].targetVelocity = 0;
        joint[j].targetTorque = 0;
        
        joint[j].actualDegree = 0;
        joint[j].actualRadian = 0;
        joint[j].actualVelocity = 0;
        joint[j].actualRPM = 0;
        joint[j].actualTorque = 0;
    }
}

void gazebo::rok3_plugin::SetJointPIDgain()
{
    /*
     * Set each joint PID gain for joint control
     */
    joint[LHY].Kp = 2000;
    joint[LHR].Kp = 9000;
    joint[LHP].Kp = 2000;
    joint[LKN].Kp = 5000;
    joint[LAP].Kp = 3000;
    joint[LAR].Kp = 3000;

    joint[RHY].Kp = joint[LHY].Kp;
    joint[RHR].Kp = joint[LHR].Kp;
    joint[RHP].Kp = joint[LHP].Kp;
    joint[RKN].Kp = joint[LKN].Kp;
    joint[RAP].Kp = joint[LAP].Kp;
    joint[RAR].Kp = joint[LAR].Kp;

    joint[WST].Kp = 2.;

    joint[LHY].Kd = 2.;
    joint[LHR].Kd = 2.;
    joint[LHP].Kd = 2.;
    joint[LKN].Kd = 4.;
    joint[LAP].Kd = 2.;
    joint[LAR].Kd = 2.;

    joint[RHY].Kd = joint[LHY].Kd;
    joint[RHR].Kd = joint[LHR].Kd;
    joint[RHP].Kd = joint[LHP].Kd;
    joint[RKN].Kd = joint[LKN].Kd;
    joint[RAP].Kd = joint[LAP].Kd;
    joint[RAR].Kd = joint[LAR].Kd;

    joint[WST].Kd = 2.;
}

