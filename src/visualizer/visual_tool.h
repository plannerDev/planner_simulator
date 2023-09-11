#pragma once

#include <string>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model/joint_model_group.h>
#include <visualization_msgs/msg/marker.h>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <gsmpl/utility/log_utility.h>
#include <gsmpl/base/math_utility.h>
#include <gsmpl/planner_data/planner_record.h>
#include <gsmpl/base/tree.h>
#include "../robot_algo/fk_panda.h"

namespace gsmpl {

struct TreeVisualizeRecord {
    std::vector<Edge> edges;
    std::vector<const Vertex*> vertexes;
};

class VisualToolImpl {
public:
    VisualToolImpl(const rclcpp::Node::SharedPtr& node, const FKPandaPtr& fk,
                   const moveit::core::RobotModelConstPtr& robotModel)
        : frameId_("panda_link0"),
          fk_(fk),
          visualTool_(node, frameId_, "move_group_tutorial", robotModel) {}

protected:
    void deleteAllMarkers() { visualTool_.deleteAllMarkers(); }
    void visualizeAxis(const State& q, const std::string& poseDesc = "");
    void visualizeAxis(const geometry_msgs::msg::Pose& pose,
                       const std::string& poseDesc);
    void visualizeAxis(const Eigen::Isometry3d& pose,
                       const std::string& poseDesc);
    void visualizeAxises(const std::vector<State>& qVector);
    void visualizePoses(const std::vector<State>& qVector, const RGBA& color);
    void visualizePoint(const Eigen::Vector3d& p, const RGBA& color,
                        const std::string& name = "");
    void visualizePoint(const State& q, const RGBA& color,
                        const std::string& name = "");
    void visualizeTree(const Tree& tree, const RGBA& color,
                       const std::string& name = "");

private:
    geometry_msgs::msg::Point state2Point(const State& q);
    TreeVisualizeRecord tree2Record(const VertexPtr root) const;
    void updateRecord(const Vertex* v, TreeVisualizeRecord& record) const;

    std::string frameId_;
    FKPandaPtr fk_;
    moveit_visual_tools::MoveItVisualTools visualTool_;
};

class VisualTool : public VisualToolImpl {
public:
    VisualTool(const rclcpp::Node::SharedPtr& node, const FKPandaPtr& fk,
               const moveit::core::RobotModelConstPtr& robotModel)
        : VisualToolImpl(node, fk, robotModel) {}

    void visualizePlannerRecord(const PlannerRecord& data);
    void visualizePose(const State& q) { VisualToolImpl::visualizeAxis(q); }
    void visualizePoint(const State& q, const RGBA& color,
                        const std::string& name = "") {
        VisualToolImpl::visualizePoint(q, color, name);
    }
};
} // namespace gsmpl
