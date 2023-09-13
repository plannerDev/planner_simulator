#pragma once

#include <chrono>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/collision_detection/world.h>
#include <geometric_shapes/shapes.h>
#include <fcl/narrowphase/collision.h>
#include <Eigen/Geometry>

#include <gsmpl/utility/class_forward.h>
#include <gsmpl/tools/checker/state_checker_base.h>
#include <gsmpl/utility/export.h>

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
    std::shared_ptr<fcl::CollisionGeometryd> constructFCLGeometry(
        const shapes::ShapeConstPtr& shape);
    std::vector<std::shared_ptr<fcl::CollisionGeometryd>>
    constructFCLGeometryRobotLink(const std::string& name);
    void constructFCLGeometryRobot();
    void constructFCLObjectRobotLink(const std::string name,
                                     const Eigen::Isometry3d& tf);
    void constructFCLObjectRobot(const State& q);
    void constructFCLObjectWorld(
        const collision_detection::WorldConstPtr& world);
    bool checkShapeGroupCollision(
        const std::pair<std::string,
                        std::vector<std::shared_ptr<fcl::CollisionObjectd>>>& g1,
        const std::pair<std::string,
                        std::vector<std::shared_ptr<fcl::CollisionObjectd>>>&
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
    std::map<std::string, std::vector<std::shared_ptr<fcl::CollisionGeometryd>>>
        fclGeometriesRobot_;
    std::vector<std::shared_ptr<fcl::CollisionGeometryd>> fclGeometriesWorld_;
    std::map<std::string, std::vector<std::shared_ptr<fcl::CollisionObjectd>>>
        fclObjsRobotMap_;
    std::map<std::string, std::vector<std::shared_ptr<fcl::CollisionObjectd>>>
        fclObjsWorldMap_;
    double time_{0};
    int times_{0};
};
} // namespace gsmpl
