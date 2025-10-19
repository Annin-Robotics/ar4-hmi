#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "kinematics.cpp"  // include your now-standalone logic

namespace py = pybind11;

void SetDHParametersExplicit(
    float theta1, float theta2, float theta3, float theta4, float theta5, float theta6,
    float alpha1, float alpha2, float alpha3, float alpha4, float alpha5, float alpha6,
    float a1,     float a2,     float a3,     float a4,     float a5,     float a6,
    float d1,     float d2,     float d3,     float d4,     float d5,     float d6
) {
    // Theta (joint offsets)
    Robot_Kin_DHM_L1[DHM_Theta] = theta1;
    Robot_Kin_DHM_L2[DHM_Theta] = theta2;
    Robot_Kin_DHM_L3[DHM_Theta] = theta3;
    Robot_Kin_DHM_L4[DHM_Theta] = theta4;
    Robot_Kin_DHM_L5[DHM_Theta] = theta5;
    Robot_Kin_DHM_L6[DHM_Theta] = theta6;

    // Alpha (link twist)
    Robot_Kin_DHM_L1[DHM_Alpha] = alpha1;
    Robot_Kin_DHM_L2[DHM_Alpha] = alpha2;
    Robot_Kin_DHM_L3[DHM_Alpha] = alpha3;
    Robot_Kin_DHM_L4[DHM_Alpha] = alpha4;
    Robot_Kin_DHM_L5[DHM_Alpha] = alpha5;
    Robot_Kin_DHM_L6[DHM_Alpha] = alpha6;

    // A (link length)
    Robot_Kin_DHM_L1[DHM_A] = a1;
    Robot_Kin_DHM_L2[DHM_A] = a2;
    Robot_Kin_DHM_L3[DHM_A] = a3;
    Robot_Kin_DHM_L4[DHM_A] = a4;
    Robot_Kin_DHM_L5[DHM_A] = a5;
    Robot_Kin_DHM_L6[DHM_A] = a6;

    // D (link offset)
    Robot_Kin_DHM_L1[DHM_D] = d1;
    Robot_Kin_DHM_L2[DHM_D] = d2;
    Robot_Kin_DHM_L3[DHM_D] = d3;
    Robot_Kin_DHM_L4[DHM_D] = d4;
    Robot_Kin_DHM_L5[DHM_D] = d5;
    Robot_Kin_DHM_L6[DHM_D] = d6;
}

void SetJointLimits(const std::vector<float>& pos_limits, const std::vector<float>& neg_limits) {
    if (pos_limits.size() != ROBOT_nDOFs || neg_limits.size() != ROBOT_nDOFs)
        throw std::runtime_error("Expected positive and negative limit vectors of size " + std::to_string(ROBOT_nDOFs));

    for (size_t i = 0; i < ROBOT_nDOFs; ++i) {
        JointPosLimit[i] = pos_limits[i];
        JointNegLimit[i] = neg_limits[i];
    }
}

PYBIND11_MODULE(robot_kinematics, m) {
    m.def("robot_set", &robot_set);
    m.def("robot_data_reset", &robot_data_reset);
    m.def("set_dh_parameters_explicit", &SetDHParametersExplicit);
    m.def("set_joint_limits", &SetJointLimits);

    m.def("forward_kinematics", [](const std::vector<float>& joints) {
        std::vector<float> result(6);
        forward_kinematics_robot_xyzuvw(joints.data(), result.data());
        return result;
    });

    m.def("inverse_kinematics", [](const std::vector<float>& target_xyzuvw,
                                    const std::vector<float>& estimate) {
        if (target_xyzuvw.size() != 6) throw std::runtime_error("Expected 6-element xyzuvw input");
        if (estimate.size() != 6) throw std::runtime_error("Expected 6-element joint estimate");

        std::vector<float> joints_out(6);
        inverse_kinematics_robot_xyzuvw<float>(target_xyzuvw.data(), joints_out.data(), estimate.data());
        return joints_out;
    });

    m.def("inverse_kinematics_no_estimate", [](const std::vector<float>& target_xyzuvw) {
        if (target_xyzuvw.size() != 6) throw std::runtime_error("Expected 6-element xyzuvw input");

        std::vector<float> joints_out(6);
        inverse_kinematics_robot_xyzuvw<float>(target_xyzuvw.data(), joints_out.data(), nullptr);
        return joints_out;
    });

    m.def("SolveInverseKinematics", &SolveInverseKinematics,
      py::arg("xyzuvw_In"), py::arg("JangleIn_in"));
      

    m.def("get_dh_parameters", []() {
      std::vector<std::vector<float>> out(6, std::vector<float>(4));

        out[0][0] = Robot_Kin_DHM_L1[DHM_Theta];
        out[0][1] = Robot_Kin_DHM_L1[DHM_Alpha];
        out[0][2] = Robot_Kin_DHM_L1[DHM_A];
        out[0][3] = Robot_Kin_DHM_L1[DHM_D];

        out[1][0] = Robot_Kin_DHM_L2[DHM_Theta];
        out[1][1] = Robot_Kin_DHM_L2[DHM_Alpha];
        out[1][2] = Robot_Kin_DHM_L2[DHM_A];
        out[1][3] = Robot_Kin_DHM_L2[DHM_D];

        out[2][0] = Robot_Kin_DHM_L3[DHM_Theta];
        out[2][1] = Robot_Kin_DHM_L3[DHM_Alpha];
        out[2][2] = Robot_Kin_DHM_L3[DHM_A];
        out[2][3] = Robot_Kin_DHM_L3[DHM_D];

        out[3][0] = Robot_Kin_DHM_L4[DHM_Theta];
        out[3][1] = Robot_Kin_DHM_L4[DHM_Alpha];
        out[3][2] = Robot_Kin_DHM_L4[DHM_A];
        out[3][3] = Robot_Kin_DHM_L4[DHM_D];

        out[4][0] = Robot_Kin_DHM_L5[DHM_Theta];
        out[4][1] = Robot_Kin_DHM_L5[DHM_Alpha];
        out[4][2] = Robot_Kin_DHM_L5[DHM_A];
        out[4][3] = Robot_Kin_DHM_L5[DHM_D];

        out[5][0] = Robot_Kin_DHM_L6[DHM_Theta];
        out[5][1] = Robot_Kin_DHM_L6[DHM_Alpha];
        out[5][2] = Robot_Kin_DHM_L6[DHM_A];
        out[5][3] = Robot_Kin_DHM_L6[DHM_D];

        return out;
    });

    m.def("get_joint_limits", []() {
        return std::make_pair(
            std::vector<float>(JointPosLimit, JointPosLimit + ROBOT_nDOFs),
            std::vector<float>(JointNegLimit, JointNegLimit + ROBOT_nDOFs)
        );
    });

    m.def("set_robot_tool_frame", &set_robot_tool_frame, 
      "Set the robot tool frame (x, y, z, rz°, ry°, rx°)");

    m.def("get_robot_tool_frame", &get_robot_tool_frame, 
        "Get the robot tool frame (x, y, z, rz°, ry°, rx°)");
}