#pragma once

#include <optional>
#include <gsmpl/tools/checker/all_state_validity_checker.h>
#include <gsmpl/base/bounds.h>
#include <gsmpl/base/se3_space.h>
#include <gsmpl/tools/local_planner/localPlanner.h>
#include "../robot_algo/ik_panda.h"
#include "../robot_algo/fk_panda.h"

namespace gsmpl {
class MoveL : public LocalPlannerBase {
public:
    MoveL(const FKPandaPtr& fk, const IKPandaPtr& ik,
          const AllStateValidityCheckerPtr& checker, std::size_t dimension)
        : LocalPlannerBase(nullptr, checker),
          fk_(fk),
          ik_(ik),
          dimension_(dimension) {}

    // from: Jps to: se3 return: Jps
    std::optional<State> interpolateState(const State& startJps,
                                          const State& targetSe3,
                                          double time) const override {
        assert(startJps.size() == dimension_);
        assert(targetSe3.size() == 7);
        Eigen::Isometry3d startSe3 = fk_->tcpPose(startJps);
        Eigen::Isometry3d pose =
            se3_.interpolateSe3(startSe3, se3_.state2Se3(targetSe3), time);
        return ik_->ik(startJps, pose);
    }
    // from: Jps to: se3 return: Jps
    std::optional<State> validInterpolateState(const State& startJps,
                                               const State& targetSe3,
                                               double time) const override {
        assert(startJps.size() == dimension_);
        assert(targetSe3.size() == 7);

        if (auto q = interpolateState(startJps, targetSe3, time)) {
            if (checker_->isValid(q.value()))
                return q;
        }
        return {};
    }
    // time[0.0, 1.0], return true, if interpolated path is valid
    std::optional<Path> validInterpolatePath(const State& startJps,
                                             const State& targetSe3,
                                             double stepSize) const override {
        assert(startJps.size() == dimension_);
        assert(targetSe3.size() == 7);

        Path path;
        Eigen::Isometry3d startSe3 = fk_->tcpPose(startJps);
        double distance =
            se3_.distanceTrans(startSe3, se3_.state2Se3(targetSe3));
        unsigned int steps = floor(distance / stepSize);
        double deltaTime = 1.0 / (double)steps;
        path.push_back(startJps);
        State refJps = startJps;
        for (double i = 1; i < steps; i++) {
            auto next = validInterpolateState(refJps, targetSe3, i * deltaTime);
            if (!next)
                return {};
            refJps = next.value();
            path.push_back(refJps);
        }
        auto target = validInterpolateState(refJps, targetSe3, 1);
        if (!target)
            return {};
        path.push_back(target.value());
        return path;
    }
    std::size_t dimension_;

private:
    Se3Space se3_;
    FKPandaPtr fk_;
    IKPandaPtr ik_;
};
} // namespace gsmpl