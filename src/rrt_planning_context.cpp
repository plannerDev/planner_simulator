#include <chrono>
#include <assert.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <gsmpl/planner_interface.h>
#include <gsmpl/planner/bi_rrt/bi_rrt_param.h>
#include <gsmpl/planner/rrt/rrt_param.h>
#include <gsmpl/planner/rrt_star/rrt_star_param.h>
#include <gsmpl/planner/informed_rrt_star/informed_rrt_star_param.h>
#include <gsmpl/planner_data/planner_param.h>
#include <gsmpl/tools/checker/state_checker_group.h>
#include <gsmpl/tools/checker/cone_region_checker.h>

#include "collision_checker/collision_checker.h"
#include "rrt_planning_context.h"

namespace rrt_interface {
namespace {
gsmpl::State moveit2Planner(const moveit::core::RobotState& robotState,
                            const moveit::core::JointModelGroup* jmg) {
    gsmpl::State state;
    for (const auto& jointModel : jmg->getActiveJointModels()) {
        const double* valuePtr = robotState.getJointPositions(jointModel);
        double value = valuePtr[0];
        state.position.push_back(value);
    }
    return state;
}
} // namespace
bool RRTPlanningContext::solve(planning_interface::MotionPlanResponse& res) {
    planning_interface::MotionPlanDetailedResponse detailRes;
    if (solve(detailRes)) {
        std::cout << "detail_res.trajectory_ size "
                  << detailRes.trajectory_.size() << std::endl;
        res.trajectory_ =
            detailRes
                .trajectory_[0]; // TODO: generate multi-segment trajectories
        std::cout << "res.trajectory size: "
                  << res.trajectory_->getWayPointCount() << std::endl;
    }
    res.planning_time_ = detailRes.processing_time_[0];
    res.error_code_ = detailRes.error_code_;
    return true;
}

bool RRTPlanningContext::terminate() { return false; }

void RRTPlanningContext::clear() {}

robot_trajectory::RobotTrajectory RRTPlanningContext::path2Trajectry(
    const gsmpl::Path& path) {
    auto trajectory =
        robot_trajectory::RobotTrajectory(robotModel_, getGroupName());
    double dt = 0.01;
    for (const auto& point : path) {
        moveit::core::RobotState state(robotModel_);
        state.setJointGroupPositions(group_, point.position);
        // state.setVariableVelocities(point.velocity);
        // state.setVariableAccelerations(point.acceleration);
        trajectory.addSuffixWayPoint(state, dt);
    }
    std::cout << "rrt trajectory size: " << trajectory.getWayPointCount()
              << std::endl;
    return trajectory;
}

gsmpl::PlannerContext RRTPlanningContext::creatPlannerContextBiRRT() {
    // bounds
    const moveit::core::JointModelGroup* jmg =
        robotModel_->getJointModelGroup(group_);
    std::vector<const std::vector<moveit::core::VariableBounds>*>
        jointBoundsVector = jmg->getActiveJointModelsBounds();
    gsmpl::Bounds bounds{generateRRTBounds(jointBoundsVector)};
    // checkers
    gsmpl::CollisionCheckerPtr collisionChecker(
        std::make_shared<gsmpl::CollisionChecker>(robotModel_, fk_,
                                                  visual_tool_));
    collisionChecker->init(planning_scene_->getWorld());
    gsmpl::ZConeRegionCheckerPtr zConeRegionChecker(
        std::make_shared<gsmpl::ZConeRegionChecker>(
            fk_, bi_rrt::CONE_THRESHOLD)); // 0.3rad = 17deg

    gsmpl::StateCheckerGroupPtr checkers =
        std::make_shared<gsmpl::StateCheckerGroup>(bounds);
    checkers->addChecker(collisionChecker);
    checkers->addChecker(zConeRegionChecker);

    // param
    gsmpl::BiRRTParam param{bounds.size(),
                            bi_rrt::RRT_MAX_STEPS,
                            bi_rrt::RRT_STEP_SIZE,
                            bi_rrt::LOCAL_PLANNER_SETP_SIZE_JPS,
                            bi_rrt::LOCAL_PLANNER_SETP_SIZE_TCP,
                            bi_rrt::CONNECTION_RANGE};
    gsmpl::PathSimplifierParamters pathSimplifierParam{
        bi_rrt::LOCAL_PLANNER_SETP_SIZE_JPS,
        bi_rrt::LOCAL_PLANNER_SETP_SIZE_TCP,
        bi_rrt::PATH_SIMPLIFIER_MAX_EMPTY_STEPS,
        bi_rrt::PATH_SIMPLIFIER_REDUCE_VERTICES_MAX_STEPS,
        bi_rrt::PATH_SIMPLIFIER_REDUCE_VERTICES_RANGE_RATIO,
        bi_rrt::PATH_SIMPLIFIER_SMOOTH_BSPLINE_MAX_STEPS,
        bi_rrt::PATH_SIMPLIFIER_SMOOTH_BSPLINE_MIN_CHANGE,
        bi_rrt::PATH_SIMPLIFIER_COLLAPSE_CLOSE_VERTICES_MAX_STEPS};
    gsmpl::TrajectoryProcessingParamters trajProcessingParam{
        bi_rrt::LOCAL_PLANNER_SETP_SIZE_JPS,
        bi_rrt::LOCAL_PLANNER_SETP_SIZE_TCP, bi_rrt::DT};
    gsmpl::PlannerGeneralParamters generalParam{
        bounds.size(), pathSimplifierParam, trajProcessingParam, bounds,
        bi_rrt::CONE_THRESHOLD};
    // context
    return gsmpl::PlannerContext(
        gsmpl::PlannerType::BiRRT,
        std::make_shared<gsmpl::SampleWithBiasParam>(bi_rrt::SAMPLER_ATTEMPTS,
                                                     bi_rrt::GOAL_BIAS),
        std::make_shared<gsmpl::BiRRTParam>(param), generalParam, checkers,
        fk_);
}

gsmpl::PlannerContext RRTPlanningContext::creatPlannerContextRRT() {
    // bounds
    const moveit::core::JointModelGroup* jmg =
        robotModel_->getJointModelGroup(group_);
    std::vector<const std::vector<moveit::core::VariableBounds>*>
        jointBoundsVector = jmg->getActiveJointModelsBounds();
    gsmpl::Bounds bounds{generateRRTBounds(jointBoundsVector)};
    // checkers
    gsmpl::CollisionCheckerPtr collisionChecker(
        std::make_shared<gsmpl::CollisionChecker>(robotModel_, fk_,
                                                  visual_tool_));
    collisionChecker->init(planning_scene_->getWorld());
    gsmpl::ZConeRegionCheckerPtr zConeRegionChecker(
        std::make_shared<gsmpl::ZConeRegionChecker>(
            fk_, rrt::CONE_THRESHOLD)); // 0.3rad = 17deg

    gsmpl::StateCheckerGroupPtr checkers =
        std::make_shared<gsmpl::StateCheckerGroup>(bounds);
    checkers->addChecker(collisionChecker);
    checkers->addChecker(zConeRegionChecker);

    // param
    gsmpl::RRTParam param{bounds.size(),
                          rrt::RRT_MAX_STEPS,
                          rrt::RRT_STEP_SIZE,
                          rrt::LOCAL_PLANNER_SETP_SIZE_JPS,
                          rrt::LOCAL_PLANNER_SETP_SIZE_TCP,
                          rrt::GOAL_THRESHOLD,
                          rrt::GOAL_BIAS};
    gsmpl::PathSimplifierParamters pathSimplifierParam{
        rrt::LOCAL_PLANNER_SETP_SIZE_JPS,
        rrt::LOCAL_PLANNER_SETP_SIZE_TCP,
        rrt::PATH_SIMPLIFIER_MAX_EMPTY_STEPS,
        rrt::PATH_SIMPLIFIER_REDUCE_VERTICES_MAX_STEPS,
        rrt::PATH_SIMPLIFIER_REDUCE_VERTICES_RANGE_RATIO,
        rrt::PATH_SIMPLIFIER_SMOOTH_BSPLINE_MAX_STEPS,
        rrt::PATH_SIMPLIFIER_SMOOTH_BSPLINE_MIN_CHANGE,
        rrt::PATH_SIMPLIFIER_COLLAPSE_CLOSE_VERTICES_MAX_STEPS};
    gsmpl::TrajectoryProcessingParamters trajProcessingParam{
        rrt::LOCAL_PLANNER_SETP_SIZE_JPS, rrt::LOCAL_PLANNER_SETP_SIZE_TCP,
        rrt::DT};
    gsmpl::PlannerGeneralParamters generalParam{
        bounds.size(), pathSimplifierParam, trajProcessingParam, bounds,
        rrt::CONE_THRESHOLD};
    // context
    return gsmpl::PlannerContext(gsmpl::PlannerType::RRT,
                                 std::make_shared<gsmpl::SampleWithBiasParam>(
                                     rrt::SAMPLER_ATTEMPTS, rrt::GOAL_BIAS),
                                 std::make_shared<gsmpl::RRTParam>(param),
                                 generalParam, checkers, fk_);
}

gsmpl::PlannerContext RRTPlanningContext::creatPlannerContextRRTStar() {
    // bounds
    const moveit::core::JointModelGroup* jmg =
        robotModel_->getJointModelGroup(group_);
    std::vector<const std::vector<moveit::core::VariableBounds>*>
        jointBoundsVector = jmg->getActiveJointModelsBounds();
    gsmpl::Bounds bounds{generateRRTBounds(jointBoundsVector)};
    // checkers
    gsmpl::CollisionCheckerPtr collisionChecker(
        std::make_shared<gsmpl::CollisionChecker>(robotModel_, fk_,
                                                  visual_tool_));
    collisionChecker->init(planning_scene_->getWorld());
    gsmpl::ZConeRegionCheckerPtr zConeRegionChecker(
        std::make_shared<gsmpl::ZConeRegionChecker>(
            fk_, rrt_star::CONE_THRESHOLD)); // 0.3rad = 17deg

    gsmpl::StateCheckerGroupPtr checkers =
        std::make_shared<gsmpl::StateCheckerGroup>(bounds);
    checkers->addChecker(collisionChecker);
    checkers->addChecker(zConeRegionChecker);

    // param
    gsmpl::RRTStarParam param{bounds.size(),
                              rrt_star::RRT_MAX_STEPS,
                              rrt_star::RRT_STEP_SIZE,
                              rrt_star::LOCAL_PLANNER_SETP_SIZE_JPS,
                              rrt_star::LOCAL_PLANNER_SETP_SIZE_TCP,
                              rrt_star::GOAL_THRESHOLD,
                              rrt_star::GOAL_BIAS};
    gsmpl::PathSimplifierParamters pathSimplifierParam{
        rrt_star::LOCAL_PLANNER_SETP_SIZE_JPS,
        rrt_star::LOCAL_PLANNER_SETP_SIZE_TCP,
        rrt_star::PATH_SIMPLIFIER_MAX_EMPTY_STEPS,
        rrt_star::PATH_SIMPLIFIER_REDUCE_VERTICES_MAX_STEPS,
        rrt_star::PATH_SIMPLIFIER_REDUCE_VERTICES_RANGE_RATIO,
        rrt_star::PATH_SIMPLIFIER_SMOOTH_BSPLINE_MAX_STEPS,
        rrt_star::PATH_SIMPLIFIER_SMOOTH_BSPLINE_MIN_CHANGE,
        rrt_star::PATH_SIMPLIFIER_COLLAPSE_CLOSE_VERTICES_MAX_STEPS};
    gsmpl::TrajectoryProcessingParamters trajProcessingParam{
        rrt_star::LOCAL_PLANNER_SETP_SIZE_JPS,
        rrt_star::LOCAL_PLANNER_SETP_SIZE_TCP, rrt_star::DT};
    gsmpl::PlannerGeneralParamters generalParam{
        bounds.size(), pathSimplifierParam, trajProcessingParam, bounds,
        rrt_star::CONE_THRESHOLD};
    // context
    return gsmpl::PlannerContext(
        gsmpl::PlannerType::RRTStar,
        std::make_shared<gsmpl::SampleWithBiasParam>(rrt_star::SAMPLER_ATTEMPTS,
                                                     rrt_star::GOAL_BIAS),
        std::make_shared<gsmpl::RRTStarParam>(param), generalParam, checkers,
        fk_);
}
gsmpl::PlannerContext RRTPlanningContext::creatPlannerContextInformedRRTStar() {
    // bounds
    const moveit::core::JointModelGroup* jmg =
        robotModel_->getJointModelGroup(group_);
    std::vector<const std::vector<moveit::core::VariableBounds>*>
        jointBoundsVector = jmg->getActiveJointModelsBounds();
    gsmpl::Bounds bounds{generateRRTBounds(jointBoundsVector)};
    // checkers
    gsmpl::CollisionCheckerPtr collisionChecker(
        std::make_shared<gsmpl::CollisionChecker>(robotModel_, fk_,
                                                  visual_tool_));
    collisionChecker->init(planning_scene_->getWorld());
    gsmpl::ZConeRegionCheckerPtr zConeRegionChecker(
        std::make_shared<gsmpl::ZConeRegionChecker>(
            fk_, informed_rrt_star::CONE_THRESHOLD)); // 0.3rad = 17deg

    gsmpl::StateCheckerGroupPtr checkers =
        std::make_shared<gsmpl::StateCheckerGroup>(bounds);
    checkers->addChecker(zConeRegionChecker);
    checkers->addChecker(collisionChecker);

    // param
    gsmpl::BiRRTParam biRRTparam{bounds.size(),
                                 bi_rrt::RRT_MAX_STEPS,
                                 bi_rrt::RRT_STEP_SIZE,
                                 bi_rrt::LOCAL_PLANNER_SETP_SIZE_JPS,
                                 bi_rrt::LOCAL_PLANNER_SETP_SIZE_TCP,
                                 bi_rrt::CONNECTION_RANGE};
    gsmpl::InformedRRTStarParam param{
        bounds.size(),
        informed_rrt_star::RRT_MAX_STEPS,
        informed_rrt_star::RRT_STEP_SIZE,
        informed_rrt_star::LOCAL_PLANNER_SETP_SIZE_JPS,
        informed_rrt_star::LOCAL_PLANNER_SETP_SIZE_TCP,
        informed_rrt_star::GOAL_COST_THRESHOLD,
        biRRTparam};
    gsmpl::PathSimplifierParamters pathSimplifierParam{
        informed_rrt_star::LOCAL_PLANNER_SETP_SIZE_JPS,
        informed_rrt_star::LOCAL_PLANNER_SETP_SIZE_TCP,
        informed_rrt_star::PATH_SIMPLIFIER_MAX_EMPTY_STEPS,
        informed_rrt_star::PATH_SIMPLIFIER_REDUCE_VERTICES_MAX_STEPS,
        informed_rrt_star::PATH_SIMPLIFIER_REDUCE_VERTICES_RANGE_RATIO,
        informed_rrt_star::PATH_SIMPLIFIER_SMOOTH_BSPLINE_MAX_STEPS,
        informed_rrt_star::PATH_SIMPLIFIER_SMOOTH_BSPLINE_MIN_CHANGE,
        informed_rrt_star::PATH_SIMPLIFIER_COLLAPSE_CLOSE_VERTICES_MAX_STEPS};
    gsmpl::TrajectoryProcessingParamters trajProcessingParam{
        informed_rrt_star::LOCAL_PLANNER_SETP_SIZE_JPS,
        informed_rrt_star::LOCAL_PLANNER_SETP_SIZE_TCP, informed_rrt_star::DT};
    gsmpl::PlannerGeneralParamters generalParam{
        bounds.size(), pathSimplifierParam, trajProcessingParam, bounds,
        informed_rrt_star::CONE_THRESHOLD};
    // context
    return gsmpl::PlannerContext(
        gsmpl::PlannerType::InformedRRTStar,
        std::make_shared<gsmpl::SampleWithBiasParam>(bi_rrt::SAMPLER_ATTEMPTS,
                                                     bi_rrt::GOAL_BIAS),
        std::make_shared<gsmpl::InformedRRTStarParam>(param), generalParam,
        checkers, fk_);
}
bool RRTPlanningContext::solve(
    planning_interface::MotionPlanDetailedResponse& res) {
    // printModelInfo();
    auto startTime = std::chrono::steady_clock::now();
    // gsmpl::PlannerContext context = creatPlannerContextBiRRT();
    // gsmpl::PlannerContext context = creatPlannerContextRRT();
    // gsmpl::PlannerContext context = creatPlannerContextRRTStar();
    gsmpl::PlannerContext context = creatPlannerContextInformedRRTStar();

    res.trajectory_.clear();
    bool solved = false;
    gsmpl::DistanceBasePtr distance =
        std::make_shared<gsmpl::DistanceL2>(context.general_param.bounds);

    for (const auto& goalConstraint : request_.goal_constraints) {
        // start
        const moveit::core::JointModelGroup* jmg =
            robotModel_->getJointModelGroup(group_);
        const moveit::core::RobotState currentState =
            planning_scene_->getCurrentState();
        gsmpl::State start = moveit2Planner(currentState, jmg);
        gsmpl::State goal = reqGoal2Values(goalConstraint);

        // BiRRT goal
        gsmpl::GoalSingleStatePtr goalPtr =
            std::make_shared<gsmpl::GoalSingleState>(
                context.general_param.bounds, goal, distance);
        // RRT goal
        // gsmpl::GoalWithJointTolerancePtr goalPtr =
        //     std::make_shared<gsmpl::GoalWithJointTolerance>(
        //         context.general_param.bounds, rrt_star::GOAL_THRESHOLD, goal,
        //         distance);
        // optiObjective
        gsmpl::OptiPathLengthPtr optiObjective =
            std::make_shared<gsmpl::OptiPathLength>(
                bi_rrt::COST_THRESHOLD, context.general_param.bounds, distance);
        // ProblemDefinition
        gsmpl::ProblemDefinition pd{start, goalPtr, optiObjective};

        // PlannerInterface
        gsmpl::PlannerInterface pi(
            context, pd,
            std::bind(&gsmpl::VisualTool::visualizePlannerRecord, &visual_tool_,
                      std::placeholders::_1),
            std::bind(&gsmpl::VisualTool::visualizePoint, &visual_tool_,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3));
        solved = pi.plan(solution_);
        pi.visualize();

        gsmpl::Path path = solution_.trajectory.dense_waypoints;

        robot_trajectory::RobotTrajectory trajectory = path2Trajectry(path);
        res.trajectory_.push_back(
            std::make_shared<robot_trajectory::RobotTrajectory>(trajectory));

        if (!solved)
            break;
    }
    if (solved) {
        res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        std::cout << "rrt SUCCESS!" << std::endl;
    } else {
        res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
        std::cout << "rrt FAILURE!" << std::endl;
    }
    res.description_.push_back("rrt_planner");
    auto endTime = std::chrono::steady_clock::now();
    res.processing_time_.push_back(
        std::chrono::duration<double>((endTime - startTime)).count()); // s
    // std::cout << "processing_time: " << res.processing_time_[0] << " " <<
    // (endTime - startTime).count() << std::endl;

    return solved;
}

gsmpl::State RRTPlanningContext::reqGoal2Values(
    const moveit_msgs::msg::Constraints& constraint) {
    gsmpl::State state;
    for (const auto& jointConstraint : constraint.joint_constraints) {
        state.position.push_back(jointConstraint.position);
    }
    return state;
}

gsmpl::Bounds RRTPlanningContext::generateRRTBounds(
    const moveit::core::JointBoundsVector& jointBoundsVector) {
    gsmpl::Bounds bounds;
    for (const auto& jointBounds : jointBoundsVector) {
        assert(jointBounds->size() == 1);
        moveit::core::VariableBounds moveitBound = jointBounds->data()[0];
        bounds.push_back(gsmpl::Bound(
            moveitBound.min_position_, moveitBound.max_position_,
            gsmpl::JointType::Revolute)); // TODO: add other type case
    }
    return bounds;
}

void RRTPlanningContext::printModelInfo() {
    const moveit::core::JointModelGroup* jmg =
        robotModel_->getJointModelGroup(group_);
    for (const auto& name : jmg->getJointModelNames()) {
        std::cout << "joint: " << name << std::endl;
        const moveit::core::JointModel* jointModel = jmg->getJointModel(name);
        std::cout << "bound: " << jointModel->getVariableBounds().size()
                  << std::endl;
        for (const auto& bound : jointModel->getVariableBounds()) {
            std::cout << " max_position_: " << bound.max_position_
                      << " min_position_: " << bound.min_position_
                      << " max_velocity_: " << bound.max_velocity_
                      << " min_velocity_: " << bound.min_velocity_
                      << " max_acceleration_: " << bound.max_acceleration_
                      << " min_acceleration_: " << bound.min_acceleration_
                      << std::endl;
        }
        std::cout << "boundMsg: " << std::endl;
        for (const auto& boundMsg : jointModel->getVariableBoundsMsg()) {
            std::cout << " max_position_: " << boundMsg.max_position
                      << " min_position_: " << boundMsg.min_position
                      << " max_velocity_: " << boundMsg.max_velocity
                      << " max_acceleration_: " << boundMsg.max_acceleration
                      << std::endl;
        }
    }
    for (const auto& name : jmg->getLinkModelNames()) {
        std::cout << "link: " << name << std::endl;
    }
    for (const auto& name : robotModel_->getVariableNames()) {
        std::cout << "Variable name: " << name << std::endl;
    }
    std::cout << "getVariableCount: " << jmg->getVariableCount() << std::endl;
    std::cout << "getActiveVariableCount: " << jmg->getActiveVariableCount()
              << std::endl;
}
} // namespace rrt_interface
