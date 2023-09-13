#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <Eigen/Core>
#include <memory>
#include "rrt_planning_context.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("rrt_demo");
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions nodeOptions;
    nodeOptions.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> rrtDemoNode =
        rclcpp::Node::make_shared("rrt_demo_node", nodeOptions);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(rrtDemoNode);
    std::thread([&executor]() { executor.spin(); }).detach();

    // BEGIN_TUTORIAL
    // Start
    // ^^^^^
    // Setting up to start using a planner is pretty easy. Planners are
    // setup as plugins in MoveIt and you can use the ROS pluginlib
    // interface to load any planner that you want to use. Before we can
    // load the planner, we need two objects, a RobotModel and a
    // PlanningScene. We will start by instantiating a
    // :moveit_codedir:`RobotModelLoader<moveit_ros/planning/robot_model_loader/include/moveit/robot_model_loader/robot_model_loader.h>`
    // object, which will look up the robot description on the ROS
    // parameter server and construct a
    // :moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>`
    // for us to use.
    const std::string PLANNING_GROUP = "panda_arm";
    robot_model_loader::RobotModelLoader robotModelLoader(rrtDemoNode,
                                                          "robot_description");
    const moveit::core::RobotModelPtr& robotModel = robotModelLoader.getModel();
    /* Create a RobotState and JointModelGroup to keep track of the current
     * robot pose and planning group*/
    moveit::core::RobotStatePtr robotState(
        new moveit::core::RobotState(robotModel));
    const moveit::core::JointModelGroup* jmg =
        robotState->getJointModelGroup(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface moveGroup(rrtDemoNode,
                                                             PLANNING_GROUP);
    planning_scene::PlanningScenePtr planningScene(
        new planning_scene::PlanningScene(robotModel));

    pluginlib::ClassLoader<planning_interface::PlannerManager> polyLoader(
        "moveit_core", "planning_interface::PlannerManager");
    planning_interface::PlannerManagerPtr rrtPlanner =
        polyLoader.createSharedInstance(
            "rrt_interface/RRTPlanner"); // chomp_interface/CHOMPPlanner
                                         // ompl_interface/OMPLPlanner
    rrtPlanner->initialize(robotModel, rrtDemoNode,
                           rrtDemoNode->get_namespace());
    RCLCPP_INFO(LOGGER, "Using planning interface '%s'",
                rrtPlanner->getDescription().data());

    // Visualization
    // ^^^^^^^^^^^^^
    // The package MoveItVisualTools provides many capabilities for visualizing
    // objects, robots, and trajectories in RViz as well as debugging tools such
    // as step-by-step introspection of a script.
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visualTools(
        rrtDemoNode, "panda_link0", "move_group_tutorial",
        robotModel); // move_group.getRobotModel()
    visualTools.enableBatchPublishing();
    visualTools.deleteAllMarkers(); // clear all old markers
    visualTools.trigger();

    /* Remote control is an introspection tool that allows users to step through
      a high level script via buttons and keyboard shortcuts in RViz */
    visualTools.loadRemoteControl();

    /* RViz provides many types of markers, in this demo we will use text,
     * cylinders, and spheres*/
    Eigen::Isometry3d textPose = Eigen::Isometry3d::Identity();
    textPose.translation().z() = 1.75;
    visualTools.publishText(textPose, "Motion Planning API Demo", rvt::WHITE,
                            rvt::XLARGE);

    /* Batch publishing is used to reduce the number of messages being sent to
     * RViz for large visualizations */
    visualTools.trigger();
    /* We can also use visualTools to wait for user input */
    visualTools.prompt(
        "Press 'next' in the RvizVisualToolsGui window to start the demo");

    // add object to the word
    moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
    // box1
    moveit_msgs::msg::CollisionObject collisionObject;
    collisionObject.header.frame_id = moveGroup.getPlanningFrame();
    collisionObject.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.02;
    primitive.dimensions[primitive.BOX_Y] = 0.02;
    primitive.dimensions[primitive.BOX_Z] = 0.02;
    geometry_msgs::msg::Pose boxPose;
    boxPose.orientation.w = 1.0;
    boxPose.position.x = 0.55;
    boxPose.position.y = 0.0;
    boxPose.position.z = 0.6;
    collisionObject.primitives.push_back(primitive);
    collisionObject.primitive_poses.push_back(boxPose);
    collisionObject.operation = collisionObject.ADD;

    // sphere
    moveit_msgs::msg::CollisionObject collisionObjectSphere;
    collisionObjectSphere.header.frame_id = moveGroup.getPlanningFrame();
    collisionObjectSphere.id = "sphere";
    shape_msgs::msg::SolidPrimitive primitiveSphere;
    primitiveSphere.type = primitiveSphere.SPHERE;
    primitiveSphere.dimensions.resize(1);
    primitiveSphere.dimensions[primitiveSphere.SPHERE_RADIUS] = 0.1;
    geometry_msgs::msg::Pose spherePose;
    spherePose.orientation.w = 1.0;
    spherePose.position.x = 0.55;
    spherePose.position.y = 0.0;
    spherePose.position.z = 0.6;
    collisionObjectSphere.primitives.push_back(primitiveSphere);
    collisionObjectSphere.primitive_poses.push_back(spherePose);
    collisionObjectSphere.operation = collisionObjectSphere.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collisionObjects;
    collisionObjects.push_back(collisionObjectSphere);
    RCLCPP_INFO(LOGGER, "Add an object into the world");
    planningSceneInterface.addCollisionObjects(collisionObjects);
    planningScene->processCollisionObjectMsg(collisionObjectSphere);

    visualTools.publishText(textPose, "Add_object", rvt::WHITE, rvt::XLARGE);
    visualTools.trigger();
    visualTools.prompt(
        "Press 'next' in the RvizVisualToolsGui window to once the collision "
        "object appears in "
        "RViz");

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    req.group_name = PLANNING_GROUP;
    req.allowed_planning_time = 10;
    planning_interface::PlanningContextPtr context;

    // Visualize the result
    // ^^^^^^^^^^^^^^^^^^^^
    std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>>
        displayPublisher =
            rrtDemoNode->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
                "/display_planned_path", 1);
    moveit_msgs::msg::DisplayTrajectory displayTrajectory;
    moveit_msgs::msg::MotionPlanResponse response;

    // Joint Space Goals
    // ^^^^^^^^^^^^^^^^^
    // Now, setup a joint space goal
    moveit::core::RobotState goalState(robotModel);
    std::vector<double> goalValues = {
        -1.0, 0.7, 0.7, -1.5,
        -0.7, 2.0, 0.0}; // -0.4 -0.325 0.36 -1.996 -0.38 1.991 0.425
    goalState.setJointGroupPositions(jmg, goalValues);
    moveit_msgs::msg::Constraints jointGoal =
        kinematic_constraints::constructGoalConstraints(goalState, jmg);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(jointGoal);

    // Configure a valid robot state
    planningScene->getCurrentStateNonConst().setToDefaultValues(jmg, "ready");

    /* Re-construct the planning context */
    context =
        rrtPlanner->getPlanningContext(planningScene, req, res.error_code);
    RCLCPP_INFO(LOGGER, "Call the Planner");
    /* Call the Planner */
    context->solve(res);

    /* Check that the planning was successful */
    if (res.error_code.val != res.error_code.SUCCESS) {
        RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
        //   return 0;
    }
    RCLCPP_INFO(LOGGER, "finish plan");
    /* Visualize the trajectory */
    res.getMessage(response);
    displayTrajectory.trajectory.push_back(response.trajectory);

    // Visualize the path point
    rrt_interface::RRTPlanningContextPtr rrtContext =
        std::static_pointer_cast<rrt_interface::RRTPlanningContext>(context);
    visualTools.trigger();
    displayPublisher->publish(displayTrajectory);

    /* We will add more goals. But first, set the state in the planning
      scene to the final state of the last plan */
    robotState->setJointGroupPositions(
        jmg, response.trajectory.joint_trajectory.points.back().positions);
    planningScene->setCurrentState(*robotState.get());

    visualTools.trigger();
    visualTools.prompt(
        "Press 'next' in the RvizVisualToolsGui window to exit the demo");
    rrtPlanner.reset();

    rclcpp::shutdown();
    return 0;
}
