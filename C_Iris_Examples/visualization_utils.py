import numpy as np
from functools import partial
from pydrake.all import (MathematicalProgram, le, SnoptSolver,
                         SurfaceTriangle, TriangleSurfaceMesh,
                         VPolytope, HPolyhedron, Sphere, RigidTransform,
                         RotationMatrix, Rgba)
import mcubes
from scipy.spatial import ConvexHull
from scipy.linalg import block_diag


def plot_surface(meshcat_instance,
                 path,
                 X,
                 Y,
                 Z,
                 rgba=Rgba(.87, .6, .6, 1.0),
                 wireframe=False,
                 wireframe_line_width=1.0):
    # taken from
    # https://github.com/RussTedrake/manipulation/blob/346038d7fb3b18d439a88be6ed731c6bf19b43de/manipulation/meshcat_cpp_utils.py#L415
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
    meshcat_instance.SetTriangleMesh(
        path,
        vertices.T,
        faces.T,
        rgba,
        wireframe,
        wireframe_line_width)


def plot_point(point, meshcat_instance, name,
               color=Rgba(0.06, 0.0, 0, 1), radius=0.05):
    meshcat_instance.SetObject(name,
                               Sphere(radius),
                               color)
    meshcat_instance.SetTransform(name, RigidTransform(
        RotationMatrix(), stretch_array_to_3d(point)))


def plot_polytope(polytope, meshcat_instance, name,
                  resolution=50, color=None,
                  wireframe=True,
                  opacity=0.7,
                  fill=True,
                  line_width=10):
    if color is None:
        color = np.random.rand(3)
    color_RGBA = Rgba(*color, opacity)
    if polytope.ambient_dimension == 3:
        verts, triangles = get_plot_poly_mesh(polytope,
                                              resolution=resolution)
        meshcat_instance.SetObject(name, TriangleSurfaceMesh(triangles, verts),
                                   color_RGBA, wireframe=wireframe)

    else:
        plot_hpoly2d(polytope, meshcat_instance, name,
                     color_RGBA,
                     line_width=line_width,
                     fill=fill,
                     resolution=resolution,
                     wireframe=wireframe)


def plot_hpoly2d(polytope, meshcat_instance, name,
                 color_RGBA,
                 line_width=8,
                 fill=False,
                 resolution=30,
                 wireframe=True):
    # plot boundary
    vpoly = VPolytope(polytope)
    verts = vpoly.vertices()
    hull = ConvexHull(verts.T)
    inds = np.append(hull.vertices, hull.vertices[0])
    hull_drake = verts.T[inds, :].T
    hull_drake3d = np.vstack([hull_drake, np.zeros(hull_drake.shape[1])])
    color_RGB = Rgba(color_RGBA.r(), color_RGBA.g(), color_RGBA.b(), 1)
    meshcat_instance.SetLine(name, hull_drake3d,
                             line_width=line_width, rgba=color_RGB)
    if fill:
        width = 0.5
        C = block_diag(polytope.A(), np.array([-1, 1])[:, np.newaxis])
        d = np.append(polytope.b(), width * np.ones(2))
        hpoly_3d = HPolyhedron(C, d)
        verts, triangles = get_plot_poly_mesh(hpoly_3d,
                                              resolution=resolution)
        meshcat_instance.SetObject(name + "/fill",
                                   TriangleSurfaceMesh(triangles, verts),
                                   color_RGBA, wireframe=wireframe)


def get_plot_poly_mesh(polytope, resolution):
    def inpolycheck(q0, q1, q2, A, b):
        q = np.array([q0, q1, q2])
        res = np.min(1.0 * (A @ q - b <= 0))
        # print(res)
        return res

    aabb_max, aabb_min = get_AABB_limits(polytope)

    col_hand = partial(inpolycheck, A=polytope.A(), b=polytope.b())
    vertices, triangles = mcubes.marching_cubes_func(tuple(aabb_min),
                                                     tuple(aabb_max),
                                                     resolution,
                                                     resolution,
                                                     resolution,
                                                     col_hand,
                                                     0.5)
    tri_drake = [SurfaceTriangle(*t) for t in triangles]
    return vertices, tri_drake


def get_AABB_limits(hpoly, dim=3):
    max_limits = []
    min_limits = []
    A = hpoly.A()
    b = hpoly.b()

    for idx in range(dim):
        aabbprog = MathematicalProgram()
        x = aabbprog.NewContinuousVariables(dim, 'x')
        cost = x[idx]
        aabbprog.AddCost(cost)
        aabbprog.AddConstraint(le(A @ x, b))
        solver = SnoptSolver()
        result = solver.Solve(aabbprog)
        min_limits.append(result.get_optimal_cost() - 0.01)
        aabbprog = MathematicalProgram()
        x = aabbprog.NewContinuousVariables(dim, 'x')
        cost = -x[idx]
        aabbprog.AddCost(cost)
        aabbprog.AddConstraint(le(A @ x, b))
        solver = SnoptSolver()
        result = solver.Solve(aabbprog)
        max_limits.append(-result.get_optimal_cost() + 0.01)
    return max_limits, min_limits


def stretch_array_to_3d(arr, val=0.):
    if arr.shape[0] < 3:
        arr = np.append(arr, val * np.ones((3 - arr.shape[0])))
    return arr


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


def get_rotation_matrix(axis, theta):
    R = np.cos(theta) * np.eye(3) + \
        np.sin(theta) * crossmat(axis) + \
        (1 - np.cos(theta)) * (axis.reshape(-1, 1) @ axis.reshape(-1, 1).T)
    return R


def crossmat(vec):
    R = np.zeros((3, 3))
    R[0, 1] = -vec[2]
    R[0, 2] = vec[1]
    R[1, 0] = vec[2]
    R[1, 2] = -vec[0]
    R[2, 0] = -vec[1]
    R[2, 1] = vec[0]
    return R
