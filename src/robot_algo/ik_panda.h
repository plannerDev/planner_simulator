#pragma once

#include <optional>
#include <Eigen/Geometry>
#include <base/state.h>
#include <base/bounds.h>
#include <utility/class_forward.h>

namespace gsmpl {
GSMPL_CLASS_FORWARD(IKPanda)

class IKPanda
{
public:
    std::optional<State> ik(const State& refJps, const Eigen::Isometry3d& pose) { return {}; }

    std::optional<std::vector<State>> ik(const Eigen::Isometry3d& pose) { return {}; }
};
} // namespace gsmpl