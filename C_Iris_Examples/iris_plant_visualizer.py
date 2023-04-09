import numpy as np
from pydrake.all import (HPolyhedron, AngleAxis,
                         VPolytope, Sphere, Ellipsoid, InverseKinematics,
                         RationalForwardKinematics, GeometrySet, Role,
                         RigidTransform, RotationMatrix,
                         Hyperellipsoid, Simulator, Box)
import mcubes

import visualization_utils as viz_utils

import pydrake.symbolic as sym
from pydrake.all import MeshcatVisualizer, StartMeshcat, DiagramBuilder, \
    AddMultibodyPlantSceneGraph, TriangleSurfaceMesh, Rgba, SurfaceTriangle, Sphere
from scipy.linalg import null_space
import time


class IrisPlantVisualizer:
    def __init__(
            self,
            plant,
            builder,
            scene_graph,
            cspace_free_polytope,
            **kwargs):
        if plant.num_positions() > 3:
            raise ValueError(
                "Can't visualize the TC-Space of plants with more than 3-DOF")
        self.meshcat_task_space = StartMeshcat()
        self.meshcat_task_space.Delete()
        self.visualizer_task_space = MeshcatVisualizer.AddToBuilder(
            builder, scene_graph, self.meshcat_task_space)

        self.meshcat_cspace = StartMeshcat()
        self.meshcat_cspace.Delete()
        builder_cspace = DiagramBuilder()
        plant_cspace, scene_graph_cspace = AddMultibodyPlantSceneGraph(
            builder_cspace, time_step=0.0)
        plant_cspace.Finalize()

        self.visualizer_cspace = MeshcatVisualizer.AddToBuilder(
            builder_cspace, scene_graph_cspace, self.meshcat_cspace)


        self.plant = plant
        self.builder = builder
        self.scene_graph = scene_graph
        self.viz_role = kwargs.get('viz_role', Role.kIllustration)

        self.task_space_diagram = self.builder.Build()
        self.task_space_diagram_context = self.task_space_diagram.CreateDefaultContext()

        self.cspace_diagram = builder_cspace.Build()
        self.cspace_diagram_context = self.cspace_diagram.CreateDefaultContext()

        self.plant_context = plant.GetMyMutableContextFromRoot(
            self.task_space_diagram_context)
        self.task_space_diagram.ForcedPublish(self.task_space_diagram_context)
        self.simulator = Simulator(
            self.task_space_diagram,
            self.task_space_diagram_context)
        self.simulator.Initialize()

        self.cspace_free_polytope = cspace_free_polytope

        # SceneGraph inspectors for highlighting geometry pairs.
        self.model_inspector = self.scene_graph.model_inspector()
        self.query = self.scene_graph.get_query_output_port().Eval(
            self.scene_graph.GetMyContextFromRoot(self.task_space_diagram_context))

        # Construct Rational Forward Kinematics for easy conversions.
        self.forward_kin = RationalForwardKinematics(plant)
        self.s_variables = sym.Variables(self.forward_kin.s())
        self.s_array = self.forward_kin.s()
        self.num_joints = self.plant.num_positions()

        # the point around which we construct the stereographic projection
        self.q_star = kwargs.get('q_star', np.zeros(self.num_joints))

        self.q_lower_limits = plant.GetPositionLowerLimits()
        self.s_lower_limits = self.forward_kin.ComputeSValue(
            self.q_lower_limits, self.q_star)
        self.q_upper_limits = plant.GetPositionUpperLimits()
        self.s_upper_limits = self.forward_kin.ComputeSValue(
            self.q_upper_limits, self.q_star)

        # A dictionary mapping str -> (HPolyhedron, SearchResult, Color) where
        # SearchResult can be None. This is used for visualizing cspace regions
        # and their certificates in task space.
        self.region_certificate_groups = {}

        # Set up the IK object to enable visualization of the collision
        # constraint.
        self.ik = InverseKinematics(plant, self.plant_context)
        min_dist = 1e-5
        self.collision_constraint = self.ik.AddMinimumDistanceConstraint(
            min_dist, 1e-5)

        # The plane numbers which we wish to visualize.
        self._plane_indices_of_interest = []
        self.plane_indices = np.arange(
            0, len(cspace_free_polytope.separating_planes()))

    def clear_plane_indices_of_interest(self):
        self._plane_indices_of_interest = []
        cur_q = self.plant.GetPositions(self.plant_context)
        self.show_res_q(cur_q)

    def add_plane_indices_of_interest(self, *elts):
        for e in elts:
            if e not in self._plane_indices_of_interest:
                self._plane_indices_of_interest.append(e)
        cur_q = self.plant.GetPositions(self.plant_context)
        self.show_res_q(cur_q)

    def remove_plane_indices_of_interest(self, *elts):
        self._plane_indices_of_interest[:] = (
            e for e in self._plane_indices_of_interest if e not in elts)
        cur_q = self.plant.GetPositions(self.plant_context)
        self.show_res_q(cur_q)

    #     visualizer.update_certificates(s)

    def show_res_q(self, q):
        self.plant.SetPositions(self.plant_context, q)
        in_collision = self.check_collision_q_by_ik(q)
        s = self.forward_kin.ComputeSValue(np.array(q), self.q_star)

        color = Rgba(1, 0.72, 0, 1) if in_collision else Rgba(0.24, 1, 0, 1)
        self.task_space_diagram.ForcedPublish(self.task_space_diagram_context)

        self.plot_cspace_points(s, name='/s', color=color, radius=0.05)

        self.update_certificates(s)

    def show_res_s(self, s):
        q = self.forward_kin.ComputeQValue(np.array(s), self.q_star)
        self.show_res_q(q)

    def check_collision_q_by_ik(self, q, min_dist=1e-5):
        if np.all(q >= self.q_lower_limits) and \
                np.all(q <= self.q_upper_limits):
            return 1 - 1 * \
                float(self.collision_constraint.evaluator().CheckSatisfied(q, min_dist))
        else:
            return 1

    def check_collision_s_by_ik(self, s, min_dist=1e-5):
        s = np.array(s)
        q = self.forward_kin.ComputeQValue(s, self.q_star)
        return self.check_collision_q_by_ik(q, min_dist)

    def visualize_collision_constraint(self, **kwargs):
        if self.plant.num_positions() == 3:
            self._visualize_collision_constraint3d(**kwargs)
        else:
            self._visualize_collision_constraint2d(**kwargs)

    def _visualize_collision_constraint3d(
            self,
            N=50,
            factor=2,
            iso_surface=0.5,
            wireframe=True):
        """
        :param N: N is density of marchingcubes grid. Runtime scales cubically in N
        :return:
        """

<<<<<<< HEAD
        vertices, triangles = mcubes.marching_cubes_func(tuple(factor*self.s_lower_limits),
                                                         tuple(factor*self.s_upper_limits),
                                                         N, N, N, self.col_func_handle_rational, iso_surface)
        tri_drake = [SurfaceTriangle(*t) for t in triangles]
        self.meshcat2.SetObject("/collision_constraint",
                                      TriangleSurfaceMesh(tri_drake, vertices),
                                      Rgba(1, 0, 0, 1), wireframe=wireframe)

    def plot_surface(self, meshcat,
                     path,
                     X,
                     Y,
                     Z,
                     rgba=Rgba(.87, .6, .6, 1.0),
                     wireframe=False,
                     wireframe_line_width=1.0):
        # taken from https://github.com/RussTedrake/manipulation/blob/346038d7fb3b18d439a88be6ed731c6bf19b43de/manipulation/meshcat_cpp_utils.py#L415
        (rows, cols) = Z.shape
        assert (np.array_equal(X.shape, Y.shape))
        assert (np.array_equal(X.shape, Z.shape))

        vertices = np.empty((rows * cols, 3), dtype=np.float32)
        vertices[:, 0] = X.reshape((-1))
        vertices[:, 1] = Y.reshape((-1))
        vertices[:, 2] = Z.reshape((-1))

        # Vectorized faces code from https://stackoverflow.com/questions/44934631/making-grid-triangular-mesh-quickly-with-numpy  # noqa
        faces = np.empty((rows - 1, cols - 1, 2, 3), dtype=np.uint32)
        r = np.arange(rows * cols).reshape(rows, cols)
        faces[:, :, 0, 0] = r[:-1, :-1]
        faces[:, :, 1, 0] = r[:-1, 1:]
        faces[:, :, 0, 1] = r[:-1, 1:]
        faces[:, :, 1, 1] = r[1:, 1:]
        faces[:, :, :, 2] = r[1:, :-1, None]
        faces.shape = (-1, 3)

        # TODO(Russ): support per vertex / Colormap colors.
        meshcat.SetTriangleMesh(path, vertices.T, faces.T, rgba, wireframe,
                                wireframe_line_width)

    def visualize_collision_constraint2d(self, factor=2, num_points=20):
        s0 = np.linspace(factor * self.s_lower_limits[0], factor * self.s_upper_limits[0], num_points)
        s1 = np.linspace(factor * self.s_lower_limits[0], factor * self.s_upper_limits[0], num_points)
=======
        vertices, triangles = mcubes.marching_cubes_func(
            tuple(
                factor * self.s_lower_limits), tuple(
                factor * self.s_upper_limits), N, N, N, self.check_collision_s_by_ik, iso_surface)
        tri_drake = [SurfaceTriangle(*t) for t in triangles]
        self.meshcat_cspace.SetObject("/collision_constraint",
                                      TriangleSurfaceMesh(tri_drake, vertices),
                                      Rgba(1, 0, 0, 1), wireframe=wireframe)

    def _visualize_collision_constraint2d(self, factor=2, num_points=20):
        s0 = np.linspace(
            factor *
            self.s_lower_limits[0],
            factor *
            self.s_upper_limits[0],
            num_points)
        s1 = np.linspace(
            factor *
            self.s_lower_limits[0],
            factor *
            self.s_upper_limits[0],
            num_points)
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
        X, Y = np.meshgrid(s0, s1)
        Z = np.zeros_like(X)
        for i in range(num_points):
            for j in range(num_points):
<<<<<<< HEAD
                Z[i, j] = self.eval_cons_rational(X[i, j], Y[i, j])
                if Z[i, j] == 0:
                    Z[i, j] = np.nan
        Z = Z-1
        self.plot_surface(self.meshcat2, "/collision_constraint", X, Y, Z, Rgba(1,0,0,1))
        return Z

    def plot_regions(self, regions, ellipses = None,
                     region_suffix = '', colors = None,
                     wireframe = True,
                     opacity = 0.7,
                     fill = True,
                     line_width = 10,
                     darken_factor = .2,
                     el_opacity = 0.3):
        if colors is None:
            colors = viz_utils.n_colors_random(len(regions), rgbs_ret=True)

        for i, region in enumerate(regions):
            c = Rgba(*[col/255 for col in colors[i]],opacity)
            prefix = f"/iris/regions{region_suffix}/{i}"
            name = prefix + "/hpoly"
            if region.ambient_dimension() == 3:
                self.plot_hpoly3d(self.meshcat2, name, region,
                                  c, wireframe = wireframe, resolution = 30)
            elif region.ambient_dimension() == 2:
                self.plot_hpoly2d(self.meshcat2, name,
                                  region, *[col/255 for col in colors[i]],
                                  a=opacity,
                             line_width=line_width,
                             fill=fill)

            if ellipses is not None:
                name = prefix + "/ellipse"
                c = Rgba(*[col/255*(1-darken_factor) for col in colors[i]],el_opacity)
                self.plot_ellipse(self.meshcat2, name,
                                  ellipses[i], c)


    def plot_seedpoints(self, seed_points):
        for i in range(seed_points.shape[0]):
            self.meshcat2.SetObject(f"/iris/seedpoints/seedpoint{i}",
                                   Sphere(0.05),
                                   Rgba(0.06, 0.0, 0, 1))
            s = np.zeros(3)
            s[:len(seed_points[i])] = seed_points[i]
            self.meshcat2.SetTransform(f"/iris/seedpoints/seedpoint{i}",
                                       RigidTransform(RotationMatrix(),
                                                      s))

    def get_plot_poly_mesh(self, region, resolution):

        def inpolycheck(q0, q1, q2, A, b):
            q = np.array([q0, q1, q2])
            res = np.min(1.0 * (A @ q - b <= 0))
            # print(res)
            return res

        aabb_max, aabb_min = viz_utils.get_AABB_limits(region)

        col_hand = partial(inpolycheck, A=region.A(), b=region.b())
        vertices, triangles = mcubes.marching_cubes_func(tuple(aabb_min),
                                                         tuple(aabb_max),
                                                         resolution,
                                                         resolution,
                                                         resolution,
                                                         col_hand,
                                                         0.5)
        tri_drake = [SurfaceTriangle(*t) for t in triangles]
        return vertices, tri_drake

    def plot_hpoly3d(self, meshcat, name, hpoly, color, wireframe = True, resolution = 30):
        verts, triangles = self.get_plot_poly_mesh(hpoly,
                                                   resolution=resolution)
        meshcat.SetObject(name, TriangleSurfaceMesh(triangles, verts),
                                color, wireframe=wireframe)

    def plot_hpoly2d(self, meshcat, name, hpoly, r = 0., g = 0., b = 1., a = 0.,
                     line_width = 8,
                     fill = False):
        # plot boundary
        vpoly = VPolytope(hpoly)
        verts = vpoly.vertices()
        hull = ConvexHull(verts.T)
        inds = np.append(hull.vertices, hull.vertices[0])
        hull_drake = verts.T[inds, :].T
        hull_drake3d = np.vstack([hull_drake, np.zeros(hull_drake.shape[1])])
        meshcat.SetLine(name, hull_drake3d,
                        line_width=line_width, rgba=Rgba(r, g, b, 1))
        if fill:
            width = 0.5
            C = block_diag(hpoly.A(), np.array([-1, 1])[:, np.newaxis])
            d = np.append(hpoly.b(), width * np.ones(2))
            hpoly_3d = HPolyhedron(C,d)
            self.plot_hpoly3d(meshcat, name+"/fill",
                              hpoly_3d, Rgba(r, g, b, a),
                              wireframe=False)

    def plot_ellipse(self,  meshcat, name, ellipse, color):
        if ellipse.A().shape[0] == 2:
            ellipse = Hyperellipsoid(block_diag(ellipse.A(), 1),
                                     np.append(ellipse.center(), 0))
        shape, pose = ellipse.ToShapeWithPose()

        meshcat.SetObject(name, shape, color)
        meshcat.SetTransform(name, pose)


    def showres(self,q, idx_list = None):
        self.plant.SetPositions(self.plant_context, q)
        col = self.col_func_handle(q)
        s = self.forward_kin.ComputeTValue(np.array(q), self.q_star)
        s = viz_utils.stretch_array_to_3d(s)
        color = Rgba(1, 0.72, 0, 1) if col else Rgba(0.24, 1, 0, 1)


        self.diagram.Publish(self.diagram_context)
        self.visualize_planes(idx_list)
        #don't change this order
        if self.do_viz_2:
            self.meshcat2.SetObject(f"/s",
                                    Sphere(0.05),
                                    color)
            self.meshcat2.SetTransform(f"/s",
                                       RigidTransform(RotationMatrix(),
                                                      s))


    def showres_s(self, s):
        q = self.forward_kin.ComputeQValue(s, self.q_star)
        self.showres(q)

    def transform(self, a, b, p1, p2, plane_verts):
        alpha = (-b - a.T @ p1) / (a.T @ (p2 - p1))
        offset = alpha * (p2 - p1) + p1
        z = np.array([0, 0, 1])
        crossprod = np.cross(viz_utils.normalize(a), z)
        if np.linalg.norm(crossprod) <= 1e-4:
            R = np.eye(3)
        else:
            ang = np.arcsin(np.linalg.norm(crossprod))
            axis = viz_utils.normalize(crossprod)
            # R = viz_utils.get_rotation_matrix(-axis, -ang)
            R = viz_utils.get_rotation_matrix(axis, ang)

        verts_tf = (R @ plane_verts.T).T + offset
        return verts_tf, RigidTransform(RotationMatrix(R), offset)

    def animate_s(self, traj, steps, runtime, idx_list = None, sleep_time = 0.1):
=======
                Z[i, j] = self.check_collision_s_by_ik(
                    np.array([X[i, j], Y[i, j]]))
                if Z[i, j] == 0:
                    Z[i, j] = np.nan
        Z = Z - 1
        viz_utils.plot_surface(
            self.meshcat_cspace,
            "/collision_constraint",
            X,
            Y,
            Z,
            Rgba(
                1,
                0,
                0,
                1))
        return Z

    def update_region_visualization_by_group_name(self, name, **kwargs):
        region_and_certificates_list = self.region_certificate_groups[name]
        for i, (r, _, color) in enumerate(region_and_certificates_list):
            viz_utils.plot_polytope(r, self.meshcat_cspace, f"/{name}/{i}",
                                    resolution=kwargs.get("resolution", 30),
                                    color=color,
                                    wireframe=kwargs.get("wireframe", True),
                                    random_color_opacity=kwargs.get("random_color_opacity", 0.7),
                                    fill=kwargs.get("fill", True),
                                    line_width=kwargs.get("line_width", 10))

    def update_region_visualization(self, **kwargs):
        for name in self.region_certificate_groups.keys():
            self.update_region_visualization_by_group_name(name, **kwargs)

    def add_group_of_regions_to_visualization(
            self, region_color_tuples, group_name, **kwargs):
        # **kwargs are the ones for viz_utils.plot_polytopes
        self.region_certificate_groups[group_name] = [
            (region, None, color) for (
                region, color) in region_color_tuples]
        self.update_region_visualization_by_group_name(group_name, **kwargs)

    def add_group_of_regions_and_certs_to_visualization(
            self, region_cert_color_tuples, group_name, **kwargs):
        # **kwargs are the ones for viz_utils.plot_polytopes
        # each element of region_and_certs_list is an (HPolyhedron,
        # SearchResult)
        self.region_certificate_groups[group_name] = region_cert_color_tuples
        self.update_region_visualization_by_group_name(group_name, **kwargs)

    def plot_cspace_points(self, points, name, **kwargs):
        if len(points.shape) == 1:
            viz_utils.plot_point(points, self.meshcat_cspace, name, **kwargs)
        else:
            for i, s in enumerate(points):
                viz_utils.plot_point(
                    s, self.meshcat_cspace, name + f"/{i}", **kwargs)

    def highlight_geometry_id(self, geom_id, color, name=None):
        if name is None:
            name = f"/id_{geom_id}"
        shape = self.model_inspector.GetShape(geom_id)
        X_WG = self.get_geom_id_pose_in_world(geom_id)
        self.meshcat_task_space.SetObject(name, shape, color)
        self.meshcat_task_space.SetTransform(name, X_WG)

    def get_geom_id_pose_in_world(self, geom_id):
        frame_id = self.model_inspector.GetFrameId(geom_id)
        X_FG = self.model_inspector.GetPoseInFrame(geom_id)
        X_WF = self.query.GetPoseInWorld(frame_id)
        return X_WF @ X_FG

    def plot_plane_by_index_at_s(
            self,
            s,
            plane_index,
            search_result,
            color,
            name_prefix=""):
        name = name_prefix + f"/plane_{plane_index}"
        sep_plane = self.cspace_free_polytope.separating_planes()[plane_index]

        geom1, geom2 = sep_plane.positive_side_geometry.id(),\
            sep_plane.negative_side_geometry.id()

        # highlight the geometry
        self.highlight_geometry_id(geom1, color, name + f"/{geom1}")
        self.highlight_geometry_id(geom2, color, name + f"/{geom2}")

        env = {var_s: val_s for var_s, val_s in zip(
            self.cspace_free_polytope.rational_forward_kin().s(), s)}

        a = np.array([a_poly.Evaluate(env)
                     for a_poly in search_result.a[plane_index]])
        b = search_result.b[plane_index].Evaluate(env)

        expressed_body = self.plant.get_body(sep_plane.expressed_body)
        X_WE = self.plant.EvalBodyPoseInWorld(
            self.plant_context, expressed_body)
        X_EW = X_WE.inverse()
        X_WG1 = self.get_geom_id_pose_in_world(geom1)
        X_WG2 = self.get_geom_id_pose_in_world(geom2)
        p1 = (X_EW @ X_WG1).translation()
        p2 = (X_EW @ X_WG2).translation()

        mu = -b / (a.T @ (p2 - p1))
        offset = mu * (p2 - p1)
        axis = (a / np.linalg.norm(a))[:, np.newaxis]
        P = null_space(axis.T)
        R = np.hstack([P, axis])
        R = RotationMatrix(R)
        X_E_plane = RigidTransform(R, offset)

        self.meshcat_task_space.SetObject(name + "/plane",
                                          Box(5, 5, 0.02),
                                          Rgba(color.r(), color.g(), color.b(), 0.5))
        self.meshcat_task_space.SetTransform(name + "/plane", X_WE @ X_E_plane)

    def update_certificates(self, s):
        for group_name, region_and_cert_list in self.region_certificate_groups.items():
            for i, (region, search_result, color) in enumerate(
                    region_and_cert_list):
                plane_color = Rgba(color.r(), color.g(), color.b(), 1) if color is not None else None
                name_prefix = f"/{group_name}/region_{i}"
                if region.PointInSet(s) and search_result is not None:
                    for plane_index in self.plane_indices:
                        if plane_index in self._plane_indices_of_interest:
                            self.plot_plane_by_index_at_s(
                                s, plane_index, search_result, plane_color, name_prefix=name_prefix)
                        else:
                            self.meshcat_task_space.Delete(
                                name_prefix + f"/plane_{plane_index}")
                else:
                    self.meshcat_task_space.Delete(name_prefix)

    def animate_traj_s(self, traj, steps, runtime, idx_list = None, sleep_time = 0.1):
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
        # loop
        idx = 0
        going_fwd = True
        time_points = np.linspace(0, traj.end_time(), steps)
<<<<<<< HEAD

        for _ in range(runtime):
            # print(idx)
            t0 = time.time()
            q = self.forward_kin.ComputeQValue(traj.value(time_points[idx]), self.q_star)
            self.showres(q, idx_list)
=======
        frame_count = 0
        for _ in range(runtime):
            # print(idx)
            t0 = time.time()
            s = traj.value(time_points[idx])
            self.show_res_s(s)
            self.task_space_diagram_context.SetTime(frame_count * 0.01)
            self.task_space_diagram.ForcedPublish(self.task_space_diagram_context)
            self.cspace_diagram_context.SetTime(frame_count * 0.01)
            self.cspace_diagram.ForcedPublish(self.cspace_diagram_context)
            frame_count += 1
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
            if going_fwd:
                if idx + 1 < steps:
                    idx += 1
                else:
                    going_fwd = False
                    idx -= 1
            else:
                if idx - 1 >= 0:
                    idx -= 1
                else:
                    going_fwd = True
                    idx += 1
            t1 = time.time()
<<<<<<< HEAD
            pause = sleep_time- (t1-t0)
            if pause > 0:
                time.sleep(pause)

    def _is_collision_pair_of_interest(self, idA, idB):
        for gid_pairs in self.collision_pairs_of_interest:
            if (idA, idB) == gid_pairs or (idB, idA) == gid_pairs:
                return True
        return False

    def visualize_planes(self, idx_list = None):
        idx_list = [i for i in range(len(self.certified_region_solution_list))] if idx_list is None else idx_list
        q = self.plant.GetPositions(self.plant_context)
        s = self.forward_kin.ComputeTValue(np.array(q), self.q_star)
        if self.color_dict is None:
            colors = viz_utils.n_colors(len(self.certified_region_solution_list), rgbs_ret=True)
            color_dict = {i: tuple(val / 255 for val in c) for i, c in enumerate(colors)}
        else:
            color_dict = self.color_dict

        color_ctr = 0
        for region_number, sol in enumerate(self.certified_region_solution_list):
            if region_number in idx_list:
                # point is in region so see plot interesting planes
                if np.all(sol.C @ s <= sol.d):
                    for i, plane in enumerate(self._region_to_planes_of_interest_dict[sol]):
                        idA = plane.positive_side_geometry.id() if plane.positive_side_geometry is not None else None
                        idB = plane.negative_side_geometry.id() if plane.negative_side_geometry is not None else None
                        if self._is_collision_pair_of_interest(idA, idB):
                            self._plot_plane(plane, color_dict[region_number], color_dict[region_number],
                                        color_dict[region_number], s,
                                        region_number)
                            color_ctr += 1
                else:
                    # exited region so remove the visualization associated to this solution
                    self.meshcat1.Delete(f"/planes/region{region_number}")
            else:
                # exited region so remove the visualization associated to this solution
                self.meshcat1.Delete(f"/planes/region{region_number}")

    def _plot_plane(self, plane, bodyA_color, bodyB_color, plane_color, s, region_number):
        # get the vertices of the separated bodies
        vert_A = GetVertices(plane.positive_side_geometry.geometry())
        dims_A = np.max(np.abs(vert_A[:, :-1] - vert_A[:, 1:]), axis=1)
        vert_B = GetVertices(plane.positive_side_geometry.geometry())
        dims_B = np.max(np.abs(vert_B[:, :-1] - vert_B[:, 1:]), axis=1)

        # get the geometry id of the separated bodies
        geomA = plane.positive_side_geometry.id()
        geomB = plane.negative_side_geometry.id()

        # get the equation of the plane
        b = plane.b
        a = plane.a
        b_eval = b.Evaluate(dict(zip(b.GetVariables(), s)))
        a_eval = np.array([a_idx.Evaluate(dict(zip(a_idx.GetVariables(), s))) for a_idx in a])

        # transform from expressed frame of plane to world frame
        X_EW = self.plant.GetBodyFromFrameId(
            self.plant.GetBodyFrameIdIfExists(plane.expressed_link)) \
            .body_frame().CalcPoseInWorld(self.plant_context).inverse()
        X_WE = X_EW.inverse()

        # transform vertices of body A expressed in body A into world frame
        X_WA = self.plant.GetBodyFromFrameId(
            self.plant.GetBodyFrameIdIfExists(plane.positive_side_geometry.body_index())) \
            .body_frame().CalcPoseInWorld(self.plant_context)
        vert_A = X_WA @ vert_A

        # transform vertices of body A expressed in body B into world frame
        X_WB = self.plant.GetBodyFromFrameId(
            self.plant.GetBodyFrameIdIfExists(plane.negative_side_geometry.body_index())) \
            .body_frame().CalcPoseInWorld(self.plant_context)
        vert_B = X_WB @ vert_B

        verts_tf_E, trans = self.transform(a_eval, b_eval, X_EW @ vert_A[:, 0],
                                           X_EW @ vert_B[:, 0], self.plane_verts)

        box_transform = X_WE @ trans
        # verts_tf = (X_WE @ verts_tf_E.T).T
        # verts_tf = np.vstack([verts_tf, verts_tf[::-1,:]])
        prefix = f"/planes/region{region_number}"

        def plot_polytope_highlight(id, dims, trans, color):
            box = Box(dims[0], dims[1], dims[2])
            name = prefix + f"/body{id.get_value()}"
            self.meshcat1.SetObject(name,
                                    box,
                                    Rgba(*color, 1))

            offset = np.array([0, 0, dims[2] / 2])
            t_final = trans @ RigidTransform(RotationMatrix(), offset)
            self.meshcat1.SetTransform(name, t_final)

        plot_polytope_highlight(geomA, dims_A, X_WA, bodyA_color)
        plot_polytope_highlight(geomB, dims_B, X_WB, bodyB_color)

        path = prefix + f"/plane/{geomA.get_value()}, {geomB.get_value()}"
        self.meshcat1.SetObject(path,
                                self.box,
                                Rgba(*plane_color, 0.7))
        self.meshcat1.SetTransform(path, box_transform)

    def draw_traj_s_space(self, traj, maxit):
        # evals end twice fix later
        for it in range(maxit):
            pt = np.append(traj.value(it * traj.end_time() / maxit),0)
            pt_nxt = np.append(traj.value((it + 1) * traj.end_time() / maxit),0)

            path = f"/traj/points{it}"
            self.meshcat2.SetLine(path, np.hstack([pt[:,np.newaxis], pt_nxt[:, np.newaxis]]),
                                 line_width = 2, rgba = Rgba(0.0, 0.0, 1, 1))

    def MakeFromHPolyhedronSceneGraph(self, query, geom, expressed_in=None):
        shape = query.inspector().GetShape(geom)
        if isinstance(shape, (Sphere, Ellipsoid)):
            raise ValueError(f"Sphere or Ellipsoid not Supported")
        return HPolyhedron(query, geom, expressed_in)

    def MakeFromVPolytopeSceneGraph(self, query, geom, expressed_in=None):
        shape = query.inspector().GetShape(geom)
        if isinstance(shape, (Sphere, Ellipsoid)):
            raise ValueError(f"Sphere or Ellipsoid not Supported")
        return VPolytope(query, geom, expressed_in)
=======
            pause = sleep_time - (t1 - t0)
            if pause > 0:
                time.sleep(pause)

    def save_meshcats(self, filename_prefix):
        with open(filename_prefix + "_cspace.html", "w") as f:
            f.write(self.meshcat_cspace.StaticHtml())
        with open(filename_prefix + "_task_space.html", "w") as f:
            f.write(self.meshcat_task_space.StaticHtml())
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
