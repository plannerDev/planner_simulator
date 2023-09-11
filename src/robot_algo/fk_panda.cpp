#include <assert.h>
#include <stdexcept>
#include <Eigen/Dense>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/link_model.h>

#include <gsmpl/utility/log_utility.h>
#include <gsmpl/base/math_utility.h>

#include "fk_panda.h"

namespace gsmpl {
FKPanda::FKPanda(moveit::core::RobotModelConstPtr robotModel,
                 const std::string group)
    : robotModel_(robotModel), group_(group) {
    const moveit::core::JointModelGroup* jmg =
        robotModel_->getJointModelGroup(group_);
    jmg->printGroupInfo(std::cout);
    activeLinkNames_ = jmg->getLinkModelNames();
    activeLinkNames_.pop_back();

    std::cout << "activeLinkNames" << std::endl;
    for (const auto& name : activeLinkNames_) {
        // if (!jmg->hasJointModel(name)) {
        //     auto jm = jmg->getJointModel(name);
        //     switch (jm->getType()) {
        //     case moveit::core::JointModel::REVOLUTE:
        //         activeJointTypes_.push_back(JointType::Revolute);
        //         break;
        //     case moveit::core::JointModel::PRISMATIC:
        //         activeJointTypes_.push_back(JointType::Prismatic);
        //         break;
        //     default:
        //         throw std::runtime_error("invalid Joint Type");
        //         break;
        //     }
        // }
        activeJointTypes_.push_back(
            JointType::Revolute); // TODO: add other joint type case
        std::cout << name << std::endl;
    }

    linkNames_ = jmg->getLinkModelNames();
    linkNames_.push_back("panda_hand");
    linkNames_.push_back("panda_leftfinger");
    linkNames_.push_back("panda_rightfinger");

    std::cout << "link names" << std::endl;
    for (const auto& name : linkNames_)
        std::cout << name << std::endl;

    linksTfOrigin_.clear();
    for (const auto& name : linkNames_) {
        const moveit::core::LinkModel* link = robotModel_->getLinkModel(name);
        Eigen::Isometry3d tf = link->getJointOriginTransform();
        linksTfOrigin_.insert(std::make_pair(name, tf));
    }
}
Eigen::Isometry3d FKPanda::tcpPose(const State& q) {
    update(q);
    return linksTf_["panda_hand"];
}

void FKPanda::update(const State& q) {
    assert(activeLinkNames_.size() == q.size());
    State qExtend = q;
    qExtend.resize(linkNames_.size());
    linksTf_.clear();
    lookupLinktf(qExtend, Eigen::Isometry3d::Identity(), 0);
}
void FKPanda::lookupLinktf(const State& q, const Eigen::Isometry3d& tfPre,
                           std::size_t index) {
    std::string name = linkNames_[index];
    assert(name != "");
    Eigen::Isometry3d tf = tfPre * linksTfOrigin_[name] * rotateZAxis(q[index]);
    linksTf_.insert(std::make_pair(name, tf));
    index++;
    if (index < (q.size() - 1))
        lookupLinktf(q, tf, index);
    else if (index == (q.size() - 1)) {
        lookupLinktf(q, tfPre, index);
        // both panda_leftfinger & panda_rightfinger's have a parent link
        // panda_hand
    }
}
Eigen::MatrixXd FKPanda::jacobian(const State& q) {
    update(q);
    int rows = 7;
    int columns = activeLinkNames_.size();
    Eigen::MatrixXd out = Eigen::MatrixXd::Zero(rows, columns);
    Eigen::Vector3d flangePose = linksTf_["panda_hand"].translation();

    for (std::size_t jointIndex = 0; jointIndex < activeLinkNames_.size();
         jointIndex++) {
        Eigen::Isometry3d jointTransform =
            linksTf_[activeLinkNames_[jointIndex]];
        Eigen::Vector3d z_axis(0, 0, 1);
        Eigen::Vector3d jointAxis = jointTransform.rotation() * z_axis;
        switch (activeJointTypes_[jointIndex]) {
            case JointType::Revolute: {
                out.block<3, 1>(0, jointIndex) =
                    jointAxis.cross(flangePose - jointTransform.translation());
                out.block<3, 1>(3, jointIndex) = jointAxis;
                break;
            }
            case JointType::Prismatic: {
                out.block<3, 1>(0, jointIndex) = jointAxis;
                break;
            }
        }
    }
    // Quaternion representation
    // From "Quaternion kinematics for the error-state KF" 4.5.1 Global-to-local
    // relations dq/dt = 0.5 * wg ⊗ q d/dt ( [w] ) = 1/2 * [ -x -y -z ]  * [
    // omega_1 ]
    //        [x]           [  w  z -y ]    [ omega_2 ]
    //        [y]           [ -z  w  x ]    [ omega_3 ]
    //        [z]           [  y -x  w ]
    Eigen::Quaterniond quat(linksTf_["panda_hand"].rotation());
    double w = quat.w(), x = quat.x(), y = quat.y(), z = quat.z();
    Eigen::MatrixXd quatUpdateMatrix(4, 3);
    quatUpdateMatrix << -x, -y, -z, w, z, -y, -z, w, x, y, -x, w;
    out.block(3, 0, 4, columns) =
        0.5 * quatUpdateMatrix * out.block(3, 0, 3, columns);
    return out;
}

Eigen::MatrixXd FKPanda::jacobian_1(const State& q) {
    update(q);
    int rows = 7;
    int columns = activeLinkNames_.size();
    Eigen::MatrixXd out = Eigen::MatrixXd::Zero(rows, columns);
    Eigen::Vector3d flangePose = linksTf_["panda_hand"].translation();

    for (std::size_t jointIndex = 0; jointIndex < activeLinkNames_.size();
         jointIndex++) {
        Eigen::Isometry3d jointTransform =
            linksTf_[activeLinkNames_[jointIndex]];
        Eigen::Vector3d z_axis(0, 0, 1);
        Eigen::Vector3d jointAxis = jointTransform.rotation() * z_axis;
        switch (activeJointTypes_[jointIndex]) {
            case JointType::Revolute: {
                out.block<3, 1>(0, jointIndex) =
                    jointAxis.cross(flangePose - jointTransform.translation());
                out.block<3, 1>(3, jointIndex) = jointAxis;
                break;
            }
            case JointType::Prismatic: {
                out.block<3, 1>(0, jointIndex) = jointAxis;
                break;
            }
        }
    }
    // Quaternion representation
    // From "Quaternion kinematics for the error-state KF" 4.5.1 Global-to-local
    // relations dq/dt = 0.5 * wg ⊗ q d/dt ( [w] ) = 1/2 * [ -x -y -z ]  * [
    // omega_1 ]
    //        [x]           [  w -z  y ]    [ omega_2 ]
    //        [y]           [  z  w -x ]    [ omega_3 ]
    //        [z]           [ -y  x  w ]
    Eigen::Quaterniond quat(linksTf_["panda_hand"].rotation());
    double w = quat.w(), x = quat.x(), y = quat.y(), z = quat.z();
    Eigen::MatrixXd quatUpdateMatrix(4, 3);
    quatUpdateMatrix << -x, -y, -z, w, -z, y, z, w, -x, -y, x, w;
    out.block(3, 0, 4, columns) =
        0.5 * quatUpdateMatrix * out.block(3, 0, 3, columns);
    return out;
}
Eigen::VectorXd se3ToVector(const Eigen::Isometry3d& tf) {
    auto translation = tf.translation();
    Eigen::Quaterniond quat(tf.rotation());
    quat.normalize();
    Eigen::VectorXd v(7);
    v << translation.x(), translation.y(), translation.z(), quat.w(), quat.x(),
        quat.y(), quat.z();
    return v;
}

Eigen::MatrixXd FKPanda::jacobianEstimate(const State& q) {
    update(q);
    Eigen::Isometry3d tf = linksTf_["panda_hand"];
    Eigen::VectorXd y0 = se3ToVector(tf);

    int raw = y0.size();
    int columns = q.size();
    Eigen::MatrixXd dY = Eigen::MatrixXd::Zero(y0.size(), columns);
    Eigen::MatrixXd dQ = Eigen::MatrixXd::Zero(q.size(), columns);
    double dq = 0.001;

    for (int i = 0; i < columns; i++) {
        State qn = q;
        qn.values[i] += dq;
        update(qn);
        Eigen::Isometry3d tfn = linksTf_["panda_hand"];
        Eigen::VectorXd yn = se3ToVector(tfn);
        dY.block(0, i, raw, 1) = yn - y0;
        Eigen::VectorXd dqn = Eigen::VectorXd::Zero(q.size());
        dqn(i) = dq;
        dQ.block(0, i, q.size(), 1) = dqn;
    }

    return dY * dQ.transpose() * (dQ * dQ.transpose()).inverse();
}
void FKPanda::Test_Jacobian(const State& q) {
    Eigen::MatrixXd J_global = jacobian(q);
    std::cout << "J_global:" << std::endl;
    std::cout << J_global << std::endl;

    Eigen::MatrixXd J_local = jacobian_1(q);
    std::cout << "J_local:" << std::endl;
    std::cout << J_local << std::endl;

    // Eigen::EigenSolver<Eigen::MatrixXd> solver(J_global);
    // if (solver.info() != Eigen::Success)
    //     std::cout << "Failed to compute J_global eigenvalues and
    //     eigenvectors!" << std::endl;
    // Eigen::VectorXcd eigenvalues = solver.eigenvalues();
    // std::cout << "J_global Eigenvalues:" << std::endl << eigenvalues <<
    // std::endl << std::endl;

    std::cout << "J_Estimate:" << std::endl;
    Eigen::MatrixXd J_Estimate = jacobianEstimate(q);
    std::cout << J_Estimate << std::endl;

    // Eigen::EigenSolver<Eigen::MatrixXd> solver2(J2);
    // if (solver2.info() != Eigen::Success)
    //     std::cout << "Failed to compute J2 eigenvalues and eigenvectors!" <<
    //     std::endl;
    // Eigen::VectorXcd eigenvalues2 = solver2.eigenvalues();
    // std::cout << "J_Estimate Eigenvalues:" << std::endl << eigenvalues2 <<
    // std::endl << std::endl;
    std::cout << "J_Estimate - J_global" << std::endl;
    std::cout << J_Estimate - J_global << std::endl;
    std::cout << "J_Estimate - J_global norm: "
              << (J_Estimate - J_global).norm() << std::endl;

    std::cout << "J_Estimate - J_local" << std::endl;
    std::cout << J_Estimate - J_local << std::endl;
    std::cout << "J_Estimate - J_local norm: " << (J_Estimate - J_local).norm()
              << std::endl;
}
} // namespace gsmpl
