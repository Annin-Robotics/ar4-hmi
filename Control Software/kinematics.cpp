
#include <cmath>
#include <algorithm>  // for std::copy, std::abs
#include <cstring>   // for memcpy
#include <vector>
#include <array>     // optional for Matrix4x4
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/// DHM Table parameters
#define DHM_Alpha 0
#define DHM_A 1
#define DHM_Theta 2
#define DHM_D 3

#define ROBOT_nDOFs 6
#define Table_Size 6


float xyzuvw_In[6];          
float JangleIn[ROBOT_nDOFs];
float JangleOut[ROBOT_nDOFs];
float joints_estimate[ROBOT_nDOFs];
float SolutionMatrix[ROBOT_nDOFs][6];  
int KinematicError = 0;


typedef float tRobot[11 * Table_Size];  // matches the memory layout used in your code

typedef float Matrix4x4[16]; // a flat 4x4 matrix


//Variables
// TODO: Use presets similar to SCARA
float delta_segments_per_second = 5; //SCARA 

/// Custom robot base (user frame)
Matrix4x4 Robot_BaseFrame = { 1,0,0,0 , 0,1,0,0, 0,0,1,0 , 0,0,0,1 };

/// Custom robot tool (tool frame, end of arm tool or TCP)
Matrix4x4 Robot_ToolFrame = { 1,0,0,0 , 0,1,0,0, 0,0,1,0 , 0,0,0,1 };


/// Robot parameters
/// All robot data is held in a large array
tRobot Robot_Data = { 0 };

/// DHM table
float* Robot_Kin_DHM_Table = Robot_Data + 0 * Table_Size;

/// xyzwpr of the base
float* Robot_Kin_Base = Robot_Data + 6 * Table_Size;

/// xyzwpr of the tool
float* Robot_Kin_Tool = Robot_Data + 7 * Table_Size;

/// Robot lower limits
float* Robot_JointLimits_Upper = Robot_Data + 8 * Table_Size;

/// Robot upper limits
float* Robot_JointLimits_Lower = Robot_Data + 9 * Table_Size;

/// Robot axis senses
float* Robot_Senses = Robot_Data + 10 * Table_Size;

// A value mappings

float* Robot_Kin_DHM_L1 = Robot_Kin_DHM_Table + 0 * Table_Size;
float* Robot_Kin_DHM_L2 = Robot_Kin_DHM_Table + 1 * Table_Size;
float* Robot_Kin_DHM_L3 = Robot_Kin_DHM_Table + 2 * Table_Size;
float* Robot_Kin_DHM_L4 = Robot_Kin_DHM_Table + 3 * Table_Size;
float* Robot_Kin_DHM_L5 = Robot_Kin_DHM_Table + 4 * Table_Size;
float* Robot_Kin_DHM_L6 = Robot_Kin_DHM_Table + 5 * Table_Size;


float& Robot_Kin_DHM_A2(Robot_Kin_DHM_Table[1 * Table_Size + 1]);
float& Robot_Kin_DHM_A3(Robot_Kin_DHM_Table[2 * Table_Size + 1]);
float& Robot_Kin_DHM_A4(Robot_Kin_DHM_Table[3 * Table_Size + 1]);

// D value mappings
float& Robot_Kin_DHM_D1(Robot_Kin_DHM_Table[0 * Table_Size + 3]);
float& Robot_Kin_DHM_D2(Robot_Kin_DHM_Table[1 * Table_Size + 3]);
float& Robot_Kin_DHM_D4(Robot_Kin_DHM_Table[3 * Table_Size + 3]);
float& Robot_Kin_DHM_D6(Robot_Kin_DHM_Table[5 * Table_Size + 3]);

// Theta value mappings (mastering)
float& Robot_Kin_DHM_Theta1(Robot_Kin_DHM_Table[0 * Table_Size + 2]);
float& Robot_Kin_DHM_Theta2(Robot_Kin_DHM_Table[1 * Table_Size + 2]);
float& Robot_Kin_DHM_Theta3(Robot_Kin_DHM_Table[2 * Table_Size + 2]);
float& Robot_Kin_DHM_Theta4(Robot_Kin_DHM_Table[3 * Table_Size + 2]);
float& Robot_Kin_DHM_Theta5(Robot_Kin_DHM_Table[4 * Table_Size + 2]);
float& Robot_Kin_DHM_Theta6(Robot_Kin_DHM_Table[5 * Table_Size + 2]);


float JointPosLimit[ROBOT_nDOFs];
float JointNegLimit[ROBOT_nDOFs];


template <typename T>
void inverse_kinematics_raw(const T pose[16], const tRobot DK, const T joints_approx_in[6], T joints[6], int* nsol);

template <typename T>
void DHM_2_pose(T rx, T tx, T rz, T tz, Matrix4x4 pose);

template <typename T>
void pose_2_xyzuvw(const Matrix4x4 pose, T xyzwpr[6]);


#define Matrix_Multiply(out,inA,inB)\
    (out)[0] = (inA)[0]*(inB)[0] + (inA)[4]*(inB)[1] + (inA)[8]*(inB)[2];\
    (out)[1] = (inA)[1]*(inB)[0] + (inA)[5]*(inB)[1] + (inA)[9]*(inB)[2];\
    (out)[2] = (inA)[2]*(inB)[0] + (inA)[6]*(inB)[1] + (inA)[10]*(inB)[2];\
    (out)[3] = 0;\
    (out)[4] = (inA)[0]*(inB)[4] + (inA)[4]*(inB)[5] + (inA)[8]*(inB)[6];\
    (out)[5] = (inA)[1]*(inB)[4] + (inA)[5]*(inB)[5] + (inA)[9]*(inB)[6];\
    (out)[6] = (inA)[2]*(inB)[4] + (inA)[6]*(inB)[5] + (inA)[10]*(inB)[6];\
    (out)[7] = 0;\
    (out)[8] = (inA)[0]*(inB)[8] + (inA)[4]*(inB)[9] + (inA)[8]*(inB)[10];\
    (out)[9] = (inA)[1]*(inB)[8] + (inA)[5]*(inB)[9] + (inA)[9]*(inB)[10];\
    (out)[10] = (inA)[2]*(inB)[8] + (inA)[6]*(inB)[9] + (inA)[10]*(inB)[10];\
    (out)[11] = 0;\
    (out)[12] = (inA)[0]*(inB)[12] + (inA)[4]*(inB)[13] + (inA)[8]*(inB)[14] + (inA)[12];\
    (out)[13] = (inA)[1]*(inB)[12] + (inA)[5]*(inB)[13] + (inA)[9]*(inB)[14] + (inA)[13];\
    (out)[14] = (inA)[2]*(inB)[12] + (inA)[6]*(inB)[13] + (inA)[10]*(inB)[14] + (inA)[14];\
    (out)[15] = 1;

#define Matrix_Inv(out,in)\
    (out)[0] = (in)[0];\
    (out)[1] = (in)[4];\
    (out)[2] = (in)[8];\
    (out)[3] = 0;\
    (out)[4] = (in)[1];\
    (out)[5] = (in)[5];\
    (out)[6] = (in)[9];\
    (out)[7] = 0;\
    (out)[8] = (in)[2];\
    (out)[9] = (in)[6];\
    (out)[10] = (in)[10];\
    (out)[11] = 0;\
    (out)[12] = -((in)[0]*(in)[12] + (in)[1]*(in)[13] + (in)[2]*(in)[14]);\
    (out)[13] = -((in)[4]*(in)[12] + (in)[5]*(in)[13] + (in)[6]*(in)[14]);\
    (out)[14] = -((in)[8]*(in)[12] + (in)[9]*(in)[13] + (in)[10]*(in)[14]);\
    (out)[15] = 1;

#define Matrix_Copy(out,in)\
    (out)[0]=(in)[0];\
    (out)[1]=(in)[1];\
    (out)[2]=(in)[2];\
    (out)[3]=(in)[3];\
    (out)[4]=(in)[4];\
    (out)[5]=(in)[5];\
    (out)[6]=(in)[6];\
    (out)[7]=(in)[7];\
    (out)[8]=(in)[8];\
    (out)[9]=(in)[9];\
    (out)[10]=(in)[10];\
    (out)[11]=(in)[11];\
    (out)[12]=(in)[12];\
    (out)[13]=(in)[13];\
    (out)[14]=(in)[14];\
    (out)[15]=(in)[15];

#define Matrix_Eye(inout)\
    (inout)[0] = 1;\
    (inout)[1] = 0;\
    (inout)[2] = 0;\
    (inout)[3] = 0;\
    (inout)[4] = 0;\
    (inout)[5] = 1;\
    (inout)[6] = 0;\
    (inout)[7] = 0;\
    (inout)[8] = 0;\
    (inout)[9] = 0;\
    (inout)[10] = 1;\
    (inout)[11] = 0;\
    (inout)[12] = 0;\
    (inout)[13] = 0;\
    (inout)[14] = 0;\
    (inout)[15] = 1;

#define Matrix_Multiply_Cumul(inout,inB){\
    Matrix4x4 out;\
    Matrix_Multiply(out,inout,inB);\
    Matrix_Copy(inout,out);}


extern float* Robot_Kin_Tool;

void set_robot_tool_frame(float x, float y, float z, float rz_deg, float ry_deg, float rx_deg) {
    Robot_Kin_Tool[0] = x;
    Robot_Kin_Tool[1] = y;
    Robot_Kin_Tool[2] = z;
    Robot_Kin_Tool[3] = rz_deg * M_PI / 180.0f;
    Robot_Kin_Tool[4] = ry_deg * M_PI / 180.0f;
    Robot_Kin_Tool[5] = rx_deg * M_PI / 180.0f;
}

std::vector<float> get_robot_tool_frame() {
    std::vector<float> tool(6);
    tool[0] = Robot_Kin_Tool[0];
    tool[1] = Robot_Kin_Tool[1];
    tool[2] = Robot_Kin_Tool[2];
    tool[3] = Robot_Kin_Tool[3] * 180.0f / M_PI;
    tool[4] = Robot_Kin_Tool[4] * 180.0f / M_PI;
    tool[5] = Robot_Kin_Tool[5] * 180.0f / M_PI;
    return tool;
}


void robot_data_reset() {
    // Reset user base and tool frames
    Matrix_Eye(Robot_BaseFrame);
    Matrix_Eye(Robot_ToolFrame);

    // Reset internal base frame and tool frames
    for (int i = 0; i < 6; i++) {
        Robot_Kin_Base[i] = 0.0;
        Robot_Kin_Tool[i] = 0.0;
    }

    // Reset joint senses and joint limits
    for (int i = 0; i < ROBOT_nDOFs; i++) {
        Robot_Senses[i] = +1.0;
        Robot_JointLimits_Lower[i] = -180.0;
        Robot_JointLimits_Upper[i] = +180.0;
    }

    // Alpha parameters
    Robot_Kin_DHM_L1[DHM_Alpha] = 0.0;
    Robot_Kin_DHM_L2[DHM_Alpha] = -0.5 * M_PI;
    Robot_Kin_DHM_L3[DHM_Alpha] = 0.0;
    Robot_Kin_DHM_L4[DHM_Alpha] = -0.5 * M_PI;
    Robot_Kin_DHM_L5[DHM_Alpha] = 0.5 * M_PI;
    Robot_Kin_DHM_L6[DHM_Alpha] = -0.5 * M_PI;

    // Theta parameters
    Robot_Kin_DHM_L1[DHM_Theta] = 0.0;
    Robot_Kin_DHM_L2[DHM_Theta] = 0.0;
    Robot_Kin_DHM_L3[DHM_Theta] = -0.5 * M_PI;
    Robot_Kin_DHM_L4[DHM_Theta] = 0.0;
    Robot_Kin_DHM_L5[DHM_Theta] = 0.0;
    Robot_Kin_DHM_L6[DHM_Theta] = M_PI;

    // A parameters
    Robot_Kin_DHM_L1[DHM_A] = 0.0;
    Robot_Kin_DHM_L2[DHM_A] = 0.0; // this value can be different from 0
    Robot_Kin_DHM_L3[DHM_A] = 0.0; // this value can be different from 0
    Robot_Kin_DHM_L4[DHM_A] = 0.0; // this value can be different from 0
    Robot_Kin_DHM_L5[DHM_A] = 0.0;
    Robot_Kin_DHM_L6[DHM_A] = 0.0;

    // D parameters
    Robot_Kin_DHM_L1[DHM_D] = 0.0; // this value can be different from 0
    Robot_Kin_DHM_L2[DHM_D] = 0.0; // this value can be different from 0
    Robot_Kin_DHM_L3[DHM_D] = 0.0;
    Robot_Kin_DHM_L4[DHM_D] = 0.0; // this value can be different from 0
    Robot_Kin_DHM_L5[DHM_D] = 0.0;
    Robot_Kin_DHM_L6[DHM_D] = 0.0; // this value can be different from 0

}

/// Set the robot (according to 3D model)
void robot_set() {
    robot_data_reset();

    // Alpha (link twist)
    Robot_Kin_DHM_L1[DHM_Alpha] = 0.0;
    Robot_Kin_DHM_L2[DHM_Alpha] = -0.5f * M_PI;
    Robot_Kin_DHM_L3[DHM_Alpha] = 0.0;
    Robot_Kin_DHM_L4[DHM_Alpha] = -0.5f * M_PI;
    Robot_Kin_DHM_L5[DHM_Alpha] = 0.5f * M_PI;
    Robot_Kin_DHM_L6[DHM_Alpha] = M_PI;

    // Theta (joint mastering offsets)
    Robot_Kin_DHM_L1[DHM_Theta] = 0.0f;
    Robot_Kin_DHM_L2[DHM_Theta] = -0.5f * M_PI;  // -90 deg
    Robot_Kin_DHM_L3[DHM_Theta] = 0.0f;
    Robot_Kin_DHM_L4[DHM_Theta] = 0.0f;
    Robot_Kin_DHM_L5[DHM_Theta] = 0.0f;
    Robot_Kin_DHM_L6[DHM_Theta] = M_PI;          // +180 deg

    // A parameters
    Robot_Kin_DHM_L2[DHM_A] = 64.2;
    Robot_Kin_DHM_L3[DHM_A] = 305;

    // D parameters
    Robot_Kin_DHM_L1[DHM_D] = 169.77;
    Robot_Kin_DHM_L2[DHM_D] = 0;
    Robot_Kin_DHM_L4[DHM_D] = 222.63;
    Robot_Kin_DHM_L6[DHM_D] = 41;

    // Define joint limits in degrees (match Teensy)
    Robot_JointLimits_Upper[0] = 170.0f;
    Robot_JointLimits_Lower[0] = 170.0f;

    Robot_JointLimits_Upper[1] = 90.0f;
    Robot_JointLimits_Lower[1] = 42.0f;

    Robot_JointLimits_Upper[2] = 52.0f;
    Robot_JointLimits_Lower[2] = 89.0f;

    Robot_JointLimits_Upper[3] = 180.0f;
    Robot_JointLimits_Lower[3] = 180.0f;

    Robot_JointLimits_Upper[4] = 105.0f;
    Robot_JointLimits_Lower[4] = 105.0f;

    Robot_JointLimits_Upper[5] = 180.0f;
    Robot_JointLimits_Lower[5] = 180.0f;

    // Theta parameters
    // (default)

}
void robot_set_kinematics() {

}





template <typename T>
bool robot_joints_valid(const T joints[ROBOT_nDOFs]) {
    for (int i = 0; i < ROBOT_nDOFs; i++) {
        if (joints[i] < -Robot_JointLimits_Lower[i] || joints[i] > Robot_JointLimits_Upper[i]) {
            return false;
        }
    }
    return true;
}

/// Returns the pose given the position (mm) and Euler angles (rad) as an array [x,y,z,rx,ry,rz].
/// The result is the same as calling: H = transl(x,y,z)*rotx(rx)*roty(ry)*rotz(rz)
template <typename T>
void xyzwpr_2_pose(const T xyzwpr[6], Matrix4x4 pose)
{
    T srx;
    T crx;
    T sry;
    T cry;
    T srz;
    T crz;
    T H_tmp;
    srx = std::sin(xyzwpr[3]);
    crx = std::cos(xyzwpr[3]);
    sry = std::sin(xyzwpr[4]);
    cry = std::cos(xyzwpr[4]);
    srz = std::sin(xyzwpr[5]);
    crz = std::cos(xyzwpr[5]);
    pose[0] = cry * crz;
    pose[4] = -cry * srz;
    pose[8] = sry;
    pose[12] = xyzwpr[0];
    H_tmp = crz * srx;
    pose[1] = crx * srz + H_tmp * sry;
    crz *= crx;
    pose[5] = crz - srx * sry * srz;
    pose[9] = -cry * srx;
    pose[13] = xyzwpr[1];
    pose[2] = srx * srz - crz * sry;
    pose[6] = H_tmp + crx * sry * srz;
    pose[10] = crx * cry;
    pose[14] = xyzwpr[2];
    pose[3] = 0.0;
    pose[7] = 0.0;
    pose[11] = 0.0;
    pose[15] = 1.0;
}

/// Calculate the robot forward kinematics (ignores the customized base frame and tool frame)
/// The calculation is from the robot base frame to the tool flange, ignoring the TCP
template <typename T>
void forward_kinematics_arm(const T* joints, Matrix4x4 pose) {
    xyzwpr_2_pose(Robot_Kin_Base, pose);
    for (int i = 0; i < ROBOT_nDOFs; i++) {
        Matrix4x4 hi;
        float* dhm_i = Robot_Kin_DHM_Table + i * Table_Size;
        T ji_rad = joints[i] * Robot_Senses[i] * M_PI / 180.0;
        DHM_2_pose(dhm_i[0], dhm_i[1], dhm_i[2] + ji_rad, dhm_i[3], hi);
        Matrix_Multiply_Cumul(pose, hi);
    }
    Matrix4x4 tool_pose;
    xyzwpr_2_pose(Robot_Kin_Tool, tool_pose);
    Matrix_Multiply_Cumul(pose, tool_pose);


    /*
      SERIAL_ECHOLNPAIR(
        "SCARA FK Angle a=", a,
        " b=", b,
        " a_sin=", a_sin,
        " a_cos=", a_cos,
        " b_sin=", b_sin,
        " b_cos=", b_cos
      );
      SERIAL_ECHOLNPAIR(" cartes (X,Y) = "(cartes.x, ", ", cartes.y, ")");
    //*/
}


template <typename T>
int inverse_kinematics_robot(const Matrix4x4 target, T joints[ROBOT_nDOFs], const T* joints_estimate) {
    Matrix4x4 invToolFrame;
    Matrix4x4 pose_arm;
    int nsol;
    Matrix_Inv(invToolFrame, Robot_ToolFrame); // invRobot_Tool could be precalculated, the tool does not change so often
    Matrix_Multiply(pose_arm, Robot_BaseFrame, target);
    Matrix_Multiply_Cumul(pose_arm, invToolFrame);
    if (joints_estimate != nullptr) {
        inverse_kinematics_raw(pose_arm, Robot_Data, joints_estimate, joints, &nsol);
    }
    else {
        // Warning! This is dangerous if joints does not have a valid/reasonable result
        T joints_approx[6];
        memcpy(joints_approx, joints, ROBOT_nDOFs * sizeof(T));
        inverse_kinematics_raw(pose_arm, Robot_Data, joints_approx, joints, &nsol);
    }
    if (nsol == 0) {
        return 0;
    }

    return 1;
}


/// Calcualte the pose given XYZ and UVW rotation vector
template <typename T>
void xyzuvw_2_pose(const T xyzuvw[6], Matrix4x4 pose)
{
    T s;
    T angle;
    T axisunit[3];
    T ex;
    T c;
    T pose_tmp;
    T b_pose_tmp;
    s = std::sqrt((xyzuvw[3] * xyzuvw[3] + xyzuvw[4] * xyzuvw[4]) + xyzuvw[5] *
        xyzuvw[5]);
    angle = s * 180.0 / 3.1415926535897931;
    if (std::abs(angle) < 1.0E-6) {
        memset(&pose[0], 0, sizeof(T) << 4);
        pose[0] = 1.0;
        pose[5] = 1.0;
        pose[10] = 1.0;
        pose[15] = 1.0;
    }
    else {
        axisunit[1] = std::abs(xyzuvw[4]);
        axisunit[2] = std::abs(xyzuvw[5]);
        ex = std::abs(xyzuvw[3]);
        if (std::abs(xyzuvw[3]) < axisunit[1]) {
            ex = axisunit[1];
        }

        if (ex < axisunit[2]) {
            ex = axisunit[2];
        }

        if (ex < 1.0E-6) {
            memset(&pose[0], 0, sizeof(T) << 4);
            pose[0] = 1.0;
            pose[5] = 1.0;
            pose[10] = 1.0;
            pose[15] = 1.0;
        }
        else {
            axisunit[0] = xyzuvw[3] / s;
            axisunit[1] = xyzuvw[4] / s;
            axisunit[2] = xyzuvw[5] / s;
            s = angle * 3.1415926535897931 / 180.0;
            c = std::cos(s);
            s = std::sin(s);
            angle = axisunit[0] * axisunit[0];
            pose[0] = angle + c * (1.0 - angle);
            angle = axisunit[0] * axisunit[1] * (1.0 - c);
            ex = axisunit[2] * s;
            pose[4] = angle - ex;
            pose_tmp = axisunit[0] * axisunit[2] * (1.0 - c);
            b_pose_tmp = axisunit[1] * s;
            pose[8] = pose_tmp + b_pose_tmp;
            pose[1] = angle + ex;
            angle = axisunit[1] * axisunit[1];
            pose[5] = angle + (1.0 - angle) * c;
            angle = axisunit[1] * axisunit[2] * (1.0 - c);
            ex = axisunit[0] * s;
            pose[9] = angle - ex;
            pose[2] = pose_tmp - b_pose_tmp;
            pose[6] = angle + ex;
            angle = axisunit[2] * axisunit[2];
            pose[10] = angle + (1.0 - angle) * c;
            pose[3] = 0.0;
            pose[7] = 0.0;
            pose[11] = 0.0;
            pose[15] = 1.0;
        }
    }

    pose[12] = xyzuvw[0];
    pose[13] = xyzuvw[1];
    pose[14] = xyzuvw[2];
}

template <typename T>
int inverse_kinematics_robot_xyzuvw(const T target_xyzuvw[6], T joints[ROBOT_nDOFs], const T* joints_estimate) {
    Matrix4x4 pose;
    xyzuvw_2_pose(target_xyzuvw, pose);
    return inverse_kinematics_robot(pose, joints, joints_estimate);
}

template <typename T>
void forward_kinematics_robot(const T joints[ROBOT_nDOFs], Matrix4x4 target) {
    Matrix4x4 invBaseFrame;
    Matrix4x4 pose_arm;
    Matrix_Inv(invBaseFrame, Robot_BaseFrame); // invRobot_Tool could be precalculated, the tool does not change so often
    forward_kinematics_arm(joints, pose_arm);
    Matrix_Multiply(target, invBaseFrame, pose_arm);
    Matrix_Multiply_Cumul(target, Robot_ToolFrame);
}


template <typename T>
void forward_kinematics_robot_xyzuvw(const T joints[ROBOT_nDOFs], T target_xyzuvw[6]) {
    Matrix4x4 pose;
    forward_kinematics_robot(joints, pose);
    pose_2_xyzuvw(pose, target_xyzuvw);
}





void JointEstimate() {
    for (int i = 0; i < ROBOT_nDOFs; i++) {
        joints_estimate[i] = JangleIn[i];
    }
}



std::vector<float> SolveInverseKinematics(const std::vector<float>& xyzuvw_In,
                                          const std::vector<float>& JangleIn_in) {
    float joints[ROBOT_nDOFs];
    float target[6];

    float solbuffer[ROBOT_nDOFs] = { 0 };
    int NumberOfSol = 0;
    int solVal = 0;

    KinematicError = 0;

    for (int i = 0; i < ROBOT_nDOFs; i++) {
        JangleIn[i] = JangleIn_in[i];
    }

    JointEstimate();  // updates joints_estimate with best guess

    // Copy and convert target orientation to radians
    target[0] = xyzuvw_In[0];
    target[1] = xyzuvw_In[1];
    target[2] = xyzuvw_In[2];
    target[3] = xyzuvw_In[3] * M_PI / 180.0f;
    target[4] = xyzuvw_In[4] * M_PI / 180.0f;
    target[5] = xyzuvw_In[5] * M_PI / 180.0f;

    
    for (int i = -3; i <= 3; i++) {
        joints_estimate[4] = i * 30;  // sweep J5 to get multiple solutions
        int success = inverse_kinematics_robot_xyzuvw(target, joints, joints_estimate);
        if (success) {
            if (solbuffer[4] != joints[4]) {
                if (robot_joints_valid(joints)) {
                    for (int j = 0; j < ROBOT_nDOFs; j++) {
                        solbuffer[j] = joints[j];
                        SolutionMatrix[j][NumberOfSol] = solbuffer[j];
                    }
                    if (NumberOfSol <= 6) {
                        NumberOfSol++;
                    }
                }
            }
        } else {
            KinematicError = 1;
        }
    }

    joints_estimate[4] = JangleIn[4];  // restore original J5 guess

    solVal = 0;
    for (int i = 0; i < ROBOT_nDOFs; i++) {
        if ((std::abs(joints_estimate[i] - SolutionMatrix[i][0]) > 20.0f) && NumberOfSol > 1) {
            solVal = 1;
        } else if ((std::abs(joints_estimate[i] - SolutionMatrix[i][1]) > 20.0f) && NumberOfSol > 1) {
            solVal = 0;
        }
    }

    if (NumberOfSol == 0) {
        KinematicError = 1;
    }

    for (int i = 0; i < ROBOT_nDOFs; i++) {
        JangleOut[i] = SolutionMatrix[i][solVal];
    }
   return std::vector<float>(JangleOut, JangleOut + ROBOT_nDOFs); 

   

    /* 
    int success = inverse_kinematics_robot_xyzuvw(target, joints, joints_estimate);
    if (success) {
        std::cout << "IK succeeded\n";
        for (int j = 0; j < ROBOT_nDOFs; j++) {
            JangleOut[j] = joints[j];
            std::cout << "J" << j+1 << ": " << joints[j] << "\n";
        }
    } else {
        std::cout << "IK failed\n";
        KinematicError = 1;
        throw std::runtime_error("IK failed for initial estimate");
    }

    

    return std::vector<float>(JangleOut, JangleOut + ROBOT_nDOFs);
    */

}




/// Calculate the pose given Denavit Hartenber Modified parameters (as defined by Craig 1986)
/// Note that DHM is different from DH
/// DHM_2_pose(rx,tx,tz,rz) is the same as using rotx(rx)*transl(tx,0,tx)*rotz(rz)
template <typename T>
void DHM_2_pose(T rx, T tx, T rz, T tz, Matrix4x4 pose)
{
    T crx;
    T srx;
    T crz;
    T srz;
    crx = std::cos(rx);
    srx = std::sin(rx);
    crz = std::cos(rz);
    srz = std::sin(rz);
    pose[0] = crz;
    pose[4] = -srz;
    pose[8] = 0.0;
    pose[12] = tx;
    pose[1] = crx * srz;
    pose[5] = crx * crz;
    pose[9] = -srx;
    pose[13] = -tz * srx;
    pose[2] = srx * srz;
    pose[6] = crz * srx;
    pose[10] = crx;
    pose[14] = tz * crx;
    pose[3] = 0.0;
    pose[7] = 0.0;
    pose[11] = 0.0;
    pose[15] = 1.0;
}




/// Calculate the [x,y,z,u,v,w] position with rotation vector for a pose target
template <typename T>
void pose_2_xyzuvw(const Matrix4x4 pose, T out[6])
{
    T sin_angle;
    T angle;
    T vector[3];
    int iidx;
    int vector_tmp;
    signed char b_I[9];
    out[0] = pose[12];
    out[1] = pose[13];
    out[2] = pose[14];
    sin_angle = (((pose[0] + pose[5]) + pose[10]) - 1.0) * 0.5;
    if (sin_angle <= -1.0) {
        sin_angle = -1.0;
    }

    if (sin_angle >= 1.0) {
        sin_angle = 1.0;
    }

    angle = std::acos(sin_angle);
    if (angle < 1.0E-6) {
        vector[0] = 0.0;
        vector[1] = 0.0;
        vector[2] = 0.0;
    }
    else {
        sin_angle = std::sin(angle);
        if (std::abs(sin_angle) < 1.0E-6) {
            sin_angle = pose[0];
            iidx = 0;
            if (pose[0] < pose[5]) {
                sin_angle = pose[5];
                iidx = 1;
            }

            if (sin_angle < pose[10]) {
                sin_angle = pose[10];
                iidx = 2;
            }

            for (vector_tmp = 0; vector_tmp < 9; vector_tmp++) {
                b_I[vector_tmp] = 0;
            }

            b_I[0] = 1;
            b_I[4] = 1;
            b_I[8] = 1;
            sin_angle = 2.0 * (1.0 + sin_angle);
            if (sin_angle <= 0.0) {
                sin_angle = 0.0;
            }
            else {
                sin_angle = std::sqrt(sin_angle);
            }

            vector_tmp = iidx << 2;
            vector[0] = (pose[vector_tmp] + static_cast<T>(b_I[3 * iidx])) /
                sin_angle;
            vector[1] = (pose[1 + vector_tmp] + static_cast<T>(b_I[1 + 3 * iidx]))
                / sin_angle;
            vector[2] = (pose[2 + vector_tmp] + static_cast<T>(b_I[2 + 3 * iidx]))
                / sin_angle;
            angle = 3.1415926535897931;
        }
        else {
            sin_angle = 1.0 / (2.0 * sin_angle);
            vector[0] = (pose[6] - pose[9]) * sin_angle;
            vector[1] = (pose[8] - pose[2]) * sin_angle;
            vector[2] = (pose[1] - pose[4]) * sin_angle;
        }
    }

    sin_angle = angle * 180.0 / 3.1415926535897931;
    out[3] = vector[0] * sin_angle * 3.1415926535897931 / 180.0;
    out[4] = vector[1] * sin_angle * 3.1415926535897931 / 180.0;
    out[5] = vector[2] * sin_angle * 3.1415926535897931 / 180.0;
}


template <typename T>
void inverse_kinematics_raw(const T pose[16], const tRobot DK, const T joints_approx_in[6], T joints[6], int* nsol)
{
    int i0;
    T base[16];
    T joints_approx[6];
    T tool[16];
    int i;
    T Hout[16];
    T b_Hout[9];
    T dv0[4];
    bool guard1 = false;
    T make_sqrt;
    T P04[4];
    T q1;
    int i1;
    T c_Hout[16];
    T k2;
    T k1;
    T ai;
    T B;
    T C;
    T s31;
    T c31;
    T q13_idx_2;
    T bb_div_cc;
    T q13_idx_0;
    for (i0 = 0; i0 < 6; i0++) {
        joints_approx[i0] = DK[60 + i0] * joints_approx_in[i0];
    }

    xyzwpr_2_pose(*(T(*)[6]) & DK[36], base);
    xyzwpr_2_pose(*(T(*)[6]) & DK[42], tool);
    for (i0 = 0; i0 < 4; i0++) {
        i = i0 << 2;
        Hout[i] = base[i0];
        Hout[1 + i] = base[i0 + 4];
        Hout[2 + i] = base[i0 + 8];
        Hout[3 + i] = base[i0 + 12];
    }

    for (i0 = 0; i0 < 3; i0++) {
        i = i0 << 2;
        Hout[3 + i] = 0.0;
        b_Hout[3 * i0] = -Hout[i];
        b_Hout[1 + 3 * i0] = -Hout[1 + i];
        b_Hout[2 + 3 * i0] = -Hout[2 + i];
    }

    for (i0 = 0; i0 < 3; i0++) {
        Hout[12 + i0] = (b_Hout[i0] * base[12] + b_Hout[i0 + 3] * base[13]) +
            b_Hout[i0 + 6] * base[14];
    }

    for (i0 = 0; i0 < 4; i0++) {
        i = i0 << 2;
        base[i] = tool[i0];
        base[1 + i] = tool[i0 + 4];
        base[2 + i] = tool[i0 + 8];
        base[3 + i] = tool[i0 + 12];
    }

    for (i0 = 0; i0 < 3; i0++) {
        i = i0 << 2;
        base[3 + i] = 0.0;
        b_Hout[3 * i0] = -base[i];
        b_Hout[1 + 3 * i0] = -base[1 + i];
        b_Hout[2 + 3 * i0] = -base[2 + i];
    }

    for (i0 = 0; i0 < 3; i0++) {
        base[12 + i0] = (b_Hout[i0] * tool[12] + b_Hout[i0 + 3] * tool[13]) +
            b_Hout[i0 + 6] * tool[14];
    }

    dv0[0] = 0.0;
    dv0[1] = 0.0;
    dv0[2] = -DK[33];
    dv0[3] = 1.0;
    for (i0 = 0; i0 < 4; i0++) {
        for (i = 0; i < 4; i++) {
            i1 = i << 2;
            c_Hout[i0 + i1] = ((Hout[i0] * pose[i1] + Hout[i0 + 4] * pose[1 + i1]) +
                Hout[i0 + 8] * pose[2 + i1]) + Hout[i0 + 12] * pose[3 +
                i1];
        }

        P04[i0] = 0.0;
        for (i = 0; i < 4; i++) {
            i1 = i << 2;
            make_sqrt = ((c_Hout[i0] * base[i1] + c_Hout[i0 + 4] * base[1 + i1]) +
                c_Hout[i0 + 8] * base[2 + i1]) + c_Hout[i0 + 12] * base[3 +
                i1];
            tool[i0 + i1] = make_sqrt;
            P04[i0] += make_sqrt * dv0[i];
        }
    }

    guard1 = false;
    if (DK[9] == 0.0) {
        q1 = atan2(P04[1], P04[0]);
        guard1 = true;
    }
    else {
        make_sqrt = (P04[0] * P04[0] + P04[1] * P04[1]) - DK[9] * DK[9];
        if (make_sqrt < 0.0) {
            for (i = 0; i < 6; i++) {
                joints[i] = 0.0;
            }

            *nsol = 0;
        }
        else {
            q1 = atan2(P04[1], P04[0]) - atan2(DK[9], std::sqrt(make_sqrt));
            guard1 = true;
        }
    }

    if (guard1) {
        k2 = P04[2] - DK[3];
        k1 = (std::cos(q1) * P04[0] + std::sin(q1) * P04[1]) - DK[7];
        ai = (((k1 * k1 + k2 * k2) - DK[13] * DK[13]) - DK[21] * DK[21]) - DK[19] *
            DK[19];
        B = 2.0 * DK[21] * DK[13];
        C = 2.0 * DK[19] * DK[13];
        s31 = 0.0;
        c31 = 0.0;
        if (C == 0.0) {
            s31 = -ai / B;
            make_sqrt = 1.0 - s31 * s31;
            if (make_sqrt >= 0.0) {
                c31 = std::sqrt(make_sqrt);
            }
        }
        else {
            q13_idx_2 = C * C;
            bb_div_cc = B * B / q13_idx_2;
            make_sqrt = 2.0 * ai * B / q13_idx_2;
            make_sqrt = make_sqrt * make_sqrt - 4.0 * ((1.0 + bb_div_cc) * (ai * ai /
                q13_idx_2 - 1.0));
            if (make_sqrt >= 0.0) {
                s31 = (-2.0 * ai * B / q13_idx_2 + std::sqrt(make_sqrt)) / (2.0 * (1.0 +
                    bb_div_cc));
                c31 = (ai + B * s31) / C;
            }
        }

        if ((make_sqrt >= 0.0) && (std::abs(s31) <= 1.0)) {
            B = atan2(s31, c31);
            make_sqrt = std::cos(B);
            ai = std::sin(B);
            C = (DK[13] - DK[21] * ai) + DK[19] * make_sqrt;
            make_sqrt = DK[21] * make_sqrt + DK[19] * ai;
            q13_idx_0 = q1 + -DK[2];
            k2 = atan2(C * k1 - make_sqrt * k2, C * k2 + make_sqrt * k1) + (-DK[8] -
                1.5707963267948966);
            q13_idx_2 = B + -DK[14];
            bb_div_cc = joints_approx[3] * 3.1415926535897931 / 180.0 - (-DK[20]);
            q1 = q13_idx_0 + DK[2];
            B = k2 + DK[8];
            C = q13_idx_2 + DK[14];
            make_sqrt = B + C;
            s31 = std::cos(make_sqrt);
            c31 = std::cos(q1);
            Hout[0] = s31 * c31;
            ai = std::sin(q1);
            Hout[4] = s31 * ai;
            make_sqrt = std::sin(make_sqrt);
            Hout[8] = -make_sqrt;
            Hout[12] = (DK[3] * make_sqrt - DK[7] * s31) - DK[13] * std::cos(C);
            Hout[1] = -std::sin(B + C) * c31;
            Hout[5] = -std::sin(B + C) * ai;
            Hout[9] = -s31;
            Hout[13] = (DK[3] * s31 + DK[7] * make_sqrt) + DK[13] * std::sin(C);
            Hout[2] = -ai;
            Hout[6] = c31;
            Hout[10] = 0.0;
            Hout[14] = 0.0;
            Hout[3] = 0.0;
            Hout[7] = 0.0;
            Hout[11] = 0.0;
            Hout[15] = 1.0;
            for (i0 = 0; i0 < 4; i0++) {
                for (i = 0; i < 4; i++) {
                    i1 = i << 2;
                    base[i0 + i1] = ((Hout[i0] * tool[i1] + Hout[i0 + 4] * tool[1 + i1]) +
                        Hout[i0 + 8] * tool[2 + i1]) + Hout[i0 + 12] * tool[3
                        + i1];
                }
            }

            make_sqrt = 1.0 - base[9] * base[9];
            if (make_sqrt <= 0.0) {
                make_sqrt = 0.0;
            }
            else {
                make_sqrt = std::sqrt(make_sqrt);
            }

            if (make_sqrt < 1.0E-6) {
                C = atan2(make_sqrt, base[9]);
                make_sqrt = std::sin(bb_div_cc);
                ai = std::cos(bb_div_cc);
                make_sqrt = atan2(make_sqrt * base[0] + ai * base[2], make_sqrt * base[2]
                    - ai * base[0]);
            }
            else if (joints_approx[4] >= 0.0) {
                bb_div_cc = atan2(base[10] / make_sqrt, -base[8] / make_sqrt);
                C = atan2(make_sqrt, base[9]);
                make_sqrt = std::sin(C);
                make_sqrt = atan2(base[5] / make_sqrt, -base[1] / make_sqrt);
            }
            else {
                bb_div_cc = atan2(-base[10] / make_sqrt, base[8] / make_sqrt);
                C = atan2(-make_sqrt, base[9]);
                make_sqrt = std::sin(C);
                make_sqrt = atan2(base[5] / make_sqrt, -base[1] / make_sqrt);
            }

            joints[0] = q13_idx_0;
            joints[3] = bb_div_cc + -DK[20];
            joints[1] = k2;
            joints[4] = C + -DK[26];
            joints[2] = q13_idx_2;
            joints[5] = make_sqrt + (-DK[32] + 3.1415926535897931);
            make_sqrt = joints[5];


            if (joints[5] > 3.1415926535897931) {
                make_sqrt = joints[5] - 6.2831853071795862;
            }
            else {
                if (joints[5] <= -3.1415926535897931) {
                    make_sqrt = joints[5] + 6.2831853071795862;
                }
            }

            joints[5] = make_sqrt;
            for (i0 = 0; i0 < 6; i0++) {
                joints[i0] = DK[60 + i0] * (joints[i0] * 180.0 / 3.1415926535897931);
            }

            *nsol = 1.0;
        }
        else {
            for (i = 0; i < 6; i++) {
                joints[i] = 0.0;
            }

            *nsol = 0;
        }
    }
}


