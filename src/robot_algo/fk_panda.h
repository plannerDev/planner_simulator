#pragma once

#include <moveit/robot_model/robot_model.h>
#include <Eigen/Geometry>
#include <gsmpl/utility/export.h>
#include <gsmpl/utility/class_forward.h>
#include <gsmpl/base/state.h>
#include <gsmpl/robot_algo/fk.h>

namespace gsmpl {
GSMPL_CLASS_FORWARD(FKPanda)

class EXPORT FKPanda : public FKBase {
public:
    FKPanda(moveit::core::RobotModelConstPtr robotModel,
            const std::string group);

    // void init();
    void update(const State& q);
    const std::map<std::string, Eigen::Isometry3d>& getLinkTf() const {
        return linksTf_;
    }
    const std::vector<std::string>& getLinkNames() const { return linkNames_; }
    const std::vector<std::string>& getActiveLinkNames() const {
        return activeLinkNames_;
    }
    Eigen::Isometry3d tcpPose(const State& q) override;
    void Test_Jacobian(const State& q);

    Eigen::MatrixXd jacobian(const State& q);
    Eigen::MatrixXd jacobian_1(const State& q);
    Eigen::MatrixXd jacobianEstimate(const State& q);

private:
    void lookupLinktf(const State& q, const Eigen::Isometry3d& tfPre,
                      std::size_t index);

    moveit::core::RobotModelConstPtr robotModel_;
    std::string group_;
    std::vector<std::string> linkNames_;
    std::vector<std::string> activeLinkNames_;
    std::vector<JointType> activeJointTypes_;
    std::map<std::string, Eigen::Isometry3d> linksTf_;
    std::map<std::string, Eigen::Isometry3d> linksTfOrigin_;
};

} // namespace gsmpl
