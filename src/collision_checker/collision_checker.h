#pragma once

#include <chrono>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/collision_detection/world.h>
#include <geometric_shapes/shapes.h>
#include <fcl/collision.h>
#include <Eigen/Geometry>
#include <utility/class_forward.h>
#include <tools/checker/state_checker_base.h>
#include <utility/export.h>
#include "../robot_algo/fk_panda.h"
#include "../visualizer/visual_tool.h"

namespace gsmpl {
GSMPL_CLASS_FORWARD(CollisionChecker)

class CollisionChecker : public StateCheckerBase {
public:
    CollisionChecker(moveit::core::RobotModelConstPtr robotModel,
                     FKPandaPtr& fk, VisualTool& vt)
        : StateCheckerBase(),
          robotModel_(robotModel),
          fk_(fk),
          vt_(vt),
          time_(0.0) {}
    ~CollisionChecker() {
        std::cout << "collisionChecker processing time: " << time_ << " times "
                  << times_ << " average time: " << time_ / times_ << std::endl;
    }

    void init(const collision_detection::WorldConstPtr& world);
    bool isValid(const State& q) override;

private:
    std::shared_ptr<fcl::CollisionGeometry> constructFCLGeometry(
        const shapes::ShapeConstPtr& shape);
    std::vector<std::shared_ptr<fcl::CollisionGeometry>>
    constructFCLGeometryRobotLink(const std::string& name);
    void constructFCLGeometryRobot();
    void constructFCLObjectRobotLink(const std::string name,
                                     const Eigen::Isometry3d& tf);
    void constructFCLObjectRobot(const State& q);
    void constructFCLObjectWorld(
        const collision_detection::WorldConstPtr& world);
    bool checkShapeGroupCollision(
        const std::pair<std::string,
                        std::vector<std::shared_ptr<fcl::CollisionObject>>>& g1,
        const std::pair<std::string,
                        std::vector<std::shared_ptr<fcl::CollisionObject>>>&
            g2);
    bool robotSelfCollisionCheck();
    bool robotWorldCollisionCheck();

    void FKUnitTest(const State& q);

    moveit::core::RobotModelConstPtr robotModel_;
    FKPandaPtr fk_;
    VisualTool vt_;
    std::vector<std::string> linkNames_;
    std::map<std::string, Eigen::Isometry3d> linksTf_;
    collision_detection::AllowedCollisionMatrix acm_;
    std::map<std::string, std::vector<std::shared_ptr<fcl::CollisionGeometry>>>
        fclGeometriesRobot_;
    std::vector<std::shared_ptr<fcl::CollisionGeometry>> fclGeometriesWorld_;
    std::map<std::string, std::vector<std::shared_ptr<fcl::CollisionObject>>>
        fclObjsRobotMap_;
    std::map<std::string, std::vector<std::shared_ptr<fcl::CollisionObject>>>
        fclObjsWorldMap_;
    double time_{0};
    int times_{0};
};
} // namespace gsmpl
