#include <pluginlib/class_list_macros.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "rrt_planning_context.h"

namespace rrt_interface {
class RRTPlannerManager : public planning_interface::PlannerManager
{
public:
    RRTPlannerManager() : planning_interface::PlannerManager() {}
    ~RRTPlannerManager() override {}

    bool initialize(const moveit::core::RobotModelConstPtr& model,
                    const rclcpp::Node::SharedPtr& node,
                    const std::string& parameterNamespace) override
    {
        for (const std::string& gpName : model->getJointModelGroupNames()) {
            LOGD("group name: %s, robot model: %s", gpName.data(), model->getName().data());
            std::cout << "group name: " << gpName.data() << " robot model: " << model->getName().data() << std::endl;
            planningContexts_[gpName] = RRTPlanningContextPtr(
                new RRTPlanningContext("rrt_planning_context", gpName, model, node));
        }
        return true;
    }

    std::string getDescription() const override { return "MY_RRT"; }
    void getPlanningAlgorithms(std::vector<std::string>& algs) const override
    {
        algs.clear();
        algs.push_back("my_rrt");
    }

    planning_interface::PlanningContextPtr getPlanningContext(
        const planning_scene::PlanningSceneConstPtr& planningScene,
        const planning_interface::MotionPlanRequest& req,
        moveit_msgs::msg::MoveItErrorCodes& errorCode) const override
    {
        errorCode.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

        if (req.group_name.empty()) {
            std::cout << "No group specified to plan for" << std::endl;
            errorCode.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
            return planning_interface::PlanningContextPtr();
        }
        if (!planningScene) {
            std::cout << "No planning scene supplied as input" << std::endl;
            errorCode.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
            return planning_interface::PlanningContextPtr();
        }

        const RRTPlanningContextPtr& context = planningContexts_.at(req.group_name);
        std::cout << "rrt_planner_manager ===>>> context is made " << std::endl;

        context->setPlanningScene(planningScene);
        context->setMotionPlanRequest(req);
        std::cout << "context getGroupName: " << context->getGroupName()
                  << " getName: " << context->getName() << std::endl;

        errorCode.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

        return context;
    }

    bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override
    {
        return req.trajectory_constraints.constraints.empty();
    }

private:
    std::map<std::string, RRTPlanningContextPtr> planningContexts_;
};
} // namespace rrt_interface

PLUGINLIB_EXPORT_CLASS(rrt_interface::RRTPlannerManager, planning_interface::PlannerManager)
