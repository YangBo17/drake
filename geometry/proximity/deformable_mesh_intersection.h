#pragma once

#include <memory>
#include <vector>

#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/deformable_contact_geometries.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
<<<<<<< HEAD
#include "drake/geometry/query_results/deformable_contact.h"
=======
#include "drake/geometry/query_results/deformable_rigid_contact.h"
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

<<<<<<< HEAD
/* Computes the contact surface between a deformable geometry and a rigid
=======
/* Computes the contact data between a deformable geometry and a rigid
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
 geometry and appends it to existing data.
 @param[in] deformable_D
     A deformable geometry expressed in frame D. It provides the
     deformed tetrahedral mesh and the approximated signed distance field.
<<<<<<< HEAD
 @param[in] deformable_id
     Id of the deformable geometry.
=======
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
 @param[in] rigid_id
     Id of the rigid geometry.
 @param[in] rigid_mesh_R
     The rigid geometry is represented as a surface mesh, whose vertices are
     measusured and expressed in frame R. We assume that triangles are oriented
     outward.
 @param[in] rigid_bvh_R
     A bounding volume hierarchy built on the geometry contained in
     rigid_mesh_R.
 @param[in] X_DR
     The pose of the rigid geometry's frame R in deformable geometry's frame D.
<<<<<<< HEAD
 @param[in, out] deformable_contact
     The deformable contact data to be appended to.
 @pre deformable_contact != nullptr. */
void AddDeformableRigidContactSurface(
    const deformable::DeformableGeometry& deformable_D,
    GeometryId deformable_id, GeometryId rigid_id,
    const TriangleSurfaceMesh<double>& rigid_mesh_R,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& rigid_bvh_R,
    const math::RigidTransform<double>& X_DR,
    DeformableContact<double>* deformable_contact);
=======
 @param[in, out] deformable_rigid_contact
     The deformable rigid contact data to be appended to.
 @pre deformable_rigid_contact != nullptr. */
void AppendDeformableRigidContact(
    const deformable::DeformableGeometry& deformable_D,
    const GeometryId rigid_id, const TriangleSurfaceMesh<double>& rigid_mesh_R,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& rigid_bvh_R,
    const math::RigidTransform<double>& X_DR,
    DeformableRigidContact<double>* deformable_rigid_contact);
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c

}  // namespace internal
}  // namespace geometry
}  // namespace drake
