#ifndef RRT_PLUGIN__RRT_PLUGIN_HPP_
#define RRT_PLUGIN__RRT_PLUGIN_HPP_

#include <moveit/planning_interface/planning_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <planner_data/planner_solution.h>
#include <planner_data/planner_context.h>
#include "visualizer/visual_tool.h"
#include "robot_algo/fk_panda.h"

namespace rrt_interface {
MOVEIT_CLASS_FORWARD(RRTPlanningContext);

class RRTPlanningContext : public planning_interface::PlanningContext
{
public:
    RRTPlanningContext(const std::string& name, const std::string& group,
                       moveit::core::RobotModelConstPtr robotModel,
                       const rclcpp::Node::SharedPtr& node)
        : planning_interface::PlanningContext(name, group), robotModel_(robotModel), node_(node),
          fk_(std::make_shared<gsmpl::FKPanda>(robotModel_, group)),
          visual_tool_(node_, fk_, robotModel_)
    {
        std::cout << "RRTPlanningContext constructor" << std::endl;
    }
    ~RRTPlanningContext() override {}

    bool solve(planning_interface::MotionPlanDetailedResponse& res) override;
    bool solve(planning_interface::MotionPlanResponse& res) override;
    bool terminate() override;
    void clear() override;

private:
    void printModelInfo();
    gsmpl::Bounds generateRRTBounds(const moveit::core::JointBoundsVector& jointBoundsVector);
    robot_trajectory::RobotTrajectory path2Trajectry(const std::vector<gsmpl::State>& path);
    gsmpl::State reqGoal2Values(const moveit_msgs::msg::Constraints& constraint);
    gsmpl::PlannerContext creatPlannerContextBiRRT();
    gsmpl::PlannerContext creatPlannerContextRRT();
    gsmpl::PlannerContext creatPlannerContextRRTStar();
    gsmpl::PlannerContext creatPlannerContextInformedRRTStar();

    moveit::core::RobotModelConstPtr robotModel_;
    gsmpl::PlannerSolution solution_;
    rclcpp::Node::SharedPtr node_;
    gsmpl::FKPandaPtr fk_;
    gsmpl::VisualTool visual_tool_;
};
} // namespace rrt_interface

#endif // RRT_INTERFACE__RRT_NTERFACE_HPP_
