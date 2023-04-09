#pragma once

<<<<<<< HEAD
#include <drake_vendor/gz/math/Pose3.hh>
=======
#include <drake_vendor/ignition/math/Pose3.hh>
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c

#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace multibody {
namespace internal {

// Helper function to express a gz::math::Vector3d instance as
// a Vector3d instance.
Eigen::Vector3d ToVector3(const gz::math::Vector3d& vector);

// Helper function to express a gz::math::Pose3d instance as
// a RigidTransform instance.
math::RigidTransformd ToRigidTransform(const gz::math::Pose3d& pose);

// Helper function to express a RigidTransform instance as an
// gz::math::Pose3d instance.
gz::math::Pose3d ToIgnitionPose3d(const math::RigidTransformd& pose);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
