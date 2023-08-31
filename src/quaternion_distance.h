#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include <tools/distance/distance.h>

namespace planner {
// PLANNER_CLASS_FORWARD(QuaternionDistance)
// PLANNER_CLASS_FORWARD(Se3Distance)

// class QuaternionDistance : public DistanceBase
// {
// public:
//     QuaternionDistance() : DistanceBase(4) {}

//     double distance(const State& s1, const State& s2) const override
//     {
//         assert(s1.size() == dimension_);
//         assert(s2.size() == dimension_);
//         Eigen::Quaterniond q1(s1[0], s1[1], s1[2], s1[3]);
//         Eigen::Quaterniond q2(s2[0], s2[1], s2[2], s2[3]);
//         q1.normalize();
//         q2.normalize();
//         auto qd = q1.inverse() * q2;
//         return abs(2 * atan2(qd.vec().norm(), qd.w()));
//     }
//     bool isEquivalent(const State& s1, const State& s2) const override
//     {
//         assert(s1.size() == dimension_);
//         assert(s2.size() == dimension_);
//         Eigen::Quaterniond q1(s1[0], s1[1], s1[2], s1[3]);
//         Eigen::Quaterniond q2(s2[0], s2[1], s2[2], s2[3]);
//         q1.normalize();
//         q2.normalize();
//         Eigen::Vector4d v1(q1.w(), q1.x(), q1.y(), q1.z());
//         Eigen::Vector4d v2(q2.w(), q2.x(), q2.y(), q2.z());

//         if (v1 == v2 || v1 == -v2)
//             return true;
//         return false;
//     }
// };
// class PoseDistance : public DistanceBase
// {
// public:
//     PoseDistance() : DistanceBase(3) {}

//     // L2-norm
//     double distance(const State& s1, const State& s2) const override
//     {
//         assert(s1.size() == dimension_);
//         assert(s2.size() == dimension_);

//         double sum = 0.0;
//         for (std::size_t i = 0; i < dimension_; i++)
//         {
//             sum += pow((s1[i] - s2[i]), 2);
//         }
//         return sqrt(sum);
//     }

//     bool isEquivalent(const State& s1, const State& s2) const override
//     {
//         assert(s1.size() == dimension_);
//         assert(s2.size() == dimension_);

//         for (std::size_t i = 0; i < dimension_; i++)
//         {
//             if (s1[i] != s2[i])
//                 return false;
//         }
//         return true;
//     }
// };
// struct Se3Position
// {
//     State pose;
//     State quat;

//     Se3Position State2Position(const State& s) const
//     {
//         Se3Position p;
//         p.pose = State({s[0], s[1], s[2]});
//         p.quat = State({s[3], s[4], s[5], s[6]});
//         return p;
//     }

// };
// class Se3Distance : public DistanceBase
// {
// public:

//     Se3Distance() : DistanceBase(7) {}

//     double distance(const State& s1, const State& s2) const override
//     {
//         assert(s1.size() == dimension_);
//         assert(s2.size() == dimension_);
//         Se3Position p1 = p1.State2Position(s1);
//         Se3Position p2 = p2.State2Position(s2);
//         return pose_.distance(p1.pose, p2.pose);
//     }
//     bool isEquivalent(const State& s1, const State& s2) const override
//     {
//         assert(s1.size() == dimension_);
//         assert(s2.size() == dimension_);

//         Se3Position p1 = p1.State2Position(s1);
//         Se3Position p2 = p2.State2Position(s2);

//         return quat_.isEquivalent(p1.quat, p2.quat) && pose_,isEquivalent(p1.pose, p2.pose);
//     }

// private:
//     QuaternionDistance quat_;
//     PoseDistance pose_;
// };
} // namespace planner