#include <assert.h>
#include "visual_tool.h"

namespace gsmpl {
void VisualToolImpl::visualizeAxis(const State& q, const std::string& poseDesc)
{
    if (q.size() > 0) {
        visualTool_.publishAxisLabeled(fk_->tcpPose(q), poseDesc);
        visualTool_.trigger();
    }
}

void VisualToolImpl::visualizeAxis(const geometry_msgs::msg::Pose& pose,
                                   const std::string& poseDesc)
{
    visualTool_.publishAxisLabeled(pose, poseDesc);
    visualTool_.trigger();
}

void VisualToolImpl::visualizeAxis(const Eigen::Isometry3d& pose, const std::string& poseDesc)
{
    visualTool_.publishAxisLabeled(pose, poseDesc);
    visualTool_.trigger();
}
void VisualToolImpl::visualizePoint(const Eigen::Vector3d& p, const RGBA& color,
                                    const std::string& name)
{
    double scale = 0.02;
    geometry_msgs::msg::Point point;
    point.x = p(0);
    point.y = p(1);
    point.z = p(2);

    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = frameId_;
    sphere.ns = name;
    sphere.id = 2;
    sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST; // LINE_STRIP
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.color.r = color.r;
    sphere.color.g = color.g;
    sphere.color.b = color.b;
    sphere.color.a = color.a;

    sphere.points.push_back(point);
    visualTool_.publishMarker(sphere);
    visualTool_.trigger();
}
void VisualToolImpl::visualizePoint(const State& q, const RGBA& color, const std::string& name)
{
    auto s = fk_->tcpPose(q).translation();
    Eigen::Vector3d sv(s.x(), s.y(), s.z());
    visualizePoint(sv, color, name);
}
void VisualToolImpl::visualizePoses(const std::vector<State>& qVector, const RGBA& color)
{
    std::cout << "visualizePoses path size: " << qVector.size() << std::endl;
    double scale = 0.02;
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = frameId_;
    sphere.ns = "sphere";
    sphere.id = 2;
    sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST; // LINE_STRIP
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.color.r = color.r;
    sphere.color.g = color.g;
    sphere.color.b = color.b;
    sphere.color.a = color.a;
    for (const auto& q : qVector) {
        sphere.points.push_back(state2Point(q));
    }
    visualTool_.publishMarker(sphere);
    visualTool_.trigger();
}

void VisualToolImpl::visualizeAxises(const std::vector<State>& qVector)
{
    for (const auto& q : qVector) {
        visualizeAxis(q, "");
    }
    visualTool_.trigger();
}

void VisualToolImpl::visualizeTree(const Tree& tree, const RGBA& color, const std::string& name)
{
    TreeVisualizeRecord record = tree2Record(tree.root());
    std::cout << "tree vertex size: " << record.vertexes.size() << std::endl;
    double scale = 0.001;
    visualization_msgs::msg::Marker edges;
    edges.header.frame_id = frameId_;
    edges.ns = name + "_edges";
    edges.id = 0;
    edges.type = visualization_msgs::msg::Marker::LINE_LIST; // LINE_STRIP
    edges.action = visualization_msgs::msg::Marker::ADD;
    edges.scale.x = scale;
    edges.scale.y = scale;
    edges.scale.z = scale;
    edges.color.r = color.r;
    edges.color.g = color.g;
    edges.color.b = color.b;
    edges.color.a = color.a;
    for (const auto& edge : record.edges) {
        edges.points.push_back(state2Point(edge.out->state));
        edges.points.push_back(state2Point(edge.in->state));
    }

    visualization_msgs::msg::Marker vertexes;
    vertexes.header.frame_id = frameId_;
    vertexes.ns = name + "_vertexes";
    vertexes.id = 1;
    vertexes.type = visualization_msgs::msg::Marker::SPHERE_LIST; // LINE_STRIP
    vertexes.action = visualization_msgs::msg::Marker::ADD;
    vertexes.scale.x = scale;
    vertexes.scale.y = scale;
    vertexes.scale.z = scale;
    vertexes.color.r = 0.0f;
    vertexes.color.g = 1.0f;
    vertexes.color.b = 0.0f;
    vertexes.color.a = 1.0f;

    for (const auto& vertex : record.vertexes)
        vertexes.points.push_back(state2Point(vertex->state));
    visualTool_.publishMarker(edges);
    visualTool_.publishMarker(vertexes);
    visualTool_.trigger();
}

geometry_msgs::msg::Point VisualToolImpl::state2Point(const State& q)
{
    geometry_msgs::msg::Point point;
    Eigen::Isometry3d tf = fk_->tcpPose(q);
    point.x = tf.translation().x();
    point.y = tf.translation().y();
    point.z = tf.translation().z();
    return point;
}
TreeVisualizeRecord VisualToolImpl::tree2Record(const VertexPtr root) const
{
    TreeVisualizeRecord record;
    updateRecord(root.get(), record);
    return record;
}
void VisualToolImpl::updateRecord(const Vertex* v, TreeVisualizeRecord& record) const
{
    if (!v)
        return;
    record.vertexes.push_back(v);
    for (const auto& child : v->children()) {
        const Vertex* next = child.get();
        record.edges.push_back({v, next});
        updateRecord(next, record);
    }
}

void VisualTool::visualizePlannerRecord(const PlannerRecord& data)
{
    deleteAllMarkers();
    VisualToolImpl::visualizePoint(data.start, Orange, "start");
    visualizeAxis(data.goal, "goal");

    std::vector<RGBA> colors{Yellow, Red, Blue, Purple, Gold, Orange};

    assert(data.trees.size() <= colors.size());
    for (std::size_t i = 0; i < data.trees.size(); i++)
        visualizeTree(data.trees[i].first, colors[i], data.trees[i].second);
}
} // namespace gsmpl
