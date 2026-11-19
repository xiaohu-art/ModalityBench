import copy
from dataclasses import dataclass, replace
from itertools import product
from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import NDArray

from modalitybench.robots.articulation import Articulation


@dataclass(frozen=True)
class BoundingBox:
    """Bounds in the robot root frame, enclosing visual and collision geometry."""

    lower: NDArray[np.float64]
    upper: NDArray[np.float64]

    @property
    def center(self) -> NDArray[np.float64]:
        return (self.lower + self.upper) / 2

    @property
    def size(self) -> NDArray[np.float64]:
        return self.upper - self.lower

    @property
    def half_size(self) -> NDArray[np.float64]:
        return self.size / 2

    def corners(self) -> NDArray[np.float64]:
        return self.center + np.array(list(product((-1, 1), repeat=3))) * self.half_size


def robot_bounding_box(model: mujoco.MjModel, data: mujoco.MjData,
                       robot: Articulation) -> BoundingBox:
    """Root-frame bounds of the current posture; requires current kinematics."""
    root = robot.indexing.root_body_id
    root_rotation = data.xmat[root].reshape(3, 3)
    ids = np.flatnonzero(np.isin(model.geom_bodyid, robot.indexing.body_ids))
    if not len(ids):
        raise ValueError("Robot has no geometry to bound.")
    lowers, uppers = [], []
    for geom in ids:
        rotation = root_rotation.T @ data.geom_xmat[geom].reshape(3, 3)
        center = root_rotation.T @ (data.geom_xpos[geom] - data.xpos[root])
        kind, size = model.geom_type[geom], model.geom_size[geom]
        if kind == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = model.geom_dataid[geom]
            start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            vertices = model.mesh_vert[start:start + count] @ rotation.T + center
            lowers.append(vertices.min(axis=0))
            uppers.append(vertices.max(axis=0))
            continue
        if kind == mujoco.mjtGeom.mjGEOM_SPHERE:
            half = np.full(3, size[0])
        elif kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
            half = np.abs(rotation[:, 2]) * size[1] + size[0]
        elif kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
            axis = rotation[:, 2]
            half = np.sqrt(np.maximum(0, 1 - axis**2)) * size[0] + np.abs(axis) * size[1]
        elif kind == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
            half = np.linalg.norm(rotation * size, axis=1)
        elif kind == mujoco.mjtGeom.mjGEOM_BOX:
            half = np.abs(rotation) @ size
        else:
            raise ValueError(f"Unsupported robot geometry type: {kind}")
        lowers.append(center - half)
        uppers.append(center + half)
    return BoundingBox(np.min(lowers, axis=0), np.max(uppers, axis=0))


@dataclass(frozen=True)
class Collision:
    geom_id: int
    geom_name: str
    penetration: float


@dataclass(frozen=True)
class PlacementResult:
    xy: tuple[float, float]
    yaw: float
    root_z: float
    box_center: NDArray[np.float64]
    collisions: tuple[Collision, ...]
    attempts: int = 1

    @property
    def collision_free(self) -> bool:
        return not self.collisions


class PlacementChecker:
    """Read-only box queries against collision-enabled scene geoms, including floors.

    Contact is allowed; penetration beyond tolerance (meters) is rejected.
    Requires current kinematics. Rebuild after posture, height, or geometry changes.
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
                 robot: Articulation, *, tolerance: float = 1e-6):
        if robot.indexing.free_joint_q_adr.size != 7:
            raise ValueError("XY/yaw placement requires one robot free joint.")
        if not np.isfinite(tolerance) or tolerance < 0:
            raise ValueError("Collision tolerance must be finite and nonnegative.")
        root = robot.indexing.root_body_id
        if not np.allclose(data.xmat[root].reshape(3, 3)[2], [0, 0, 1], atol=1e-8):
            raise ValueError("XY/yaw placement requires an upright root (zero roll/pitch).")
        self.box = robot_bounding_box(model, data, robot)
        self.root_z = float(data.xpos[root, 2])
        self.tolerance = tolerance
        self._source_data = data
        self._indexing = robot.indexing
        self._joint_pos = data.qpos[robot.indexing.joint_q_adr].copy()
        is_robot = np.isin(model.geom_bodyid, robot.indexing.body_ids)
        self.scene_geom_ids = np.flatnonzero(~is_robot & ((model.geom_contype != 0)
                                                        | (model.geom_conaffinity != 0)))
        supported = {int(kind) for kind in (mujoco.mjtGeom.mjGEOM_PLANE, mujoco.mjtGeom.mjGEOM_SPHERE,
                     mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER,
                     mujoco.mjtGeom.mjGEOM_ELLIPSOID, mujoco.mjtGeom.mjGEOM_BOX,
                     mujoco.mjtGeom.mjGEOM_MESH)}
        for geom in self.scene_geom_ids:
            if model.geom_type[geom] not in supported:
                raise ValueError(f"Unsupported scene collision geometry: {model.geom(int(geom)).name}")
        # Replace a robot geom only in the private collision-query model.
        self._model = copy.copy(model)
        self._data = mujoco.MjData(self._model)
        self._proxy = int(np.flatnonzero(is_robot)[0])
        self._model.geom_type[self._proxy] = mujoco.mjtGeom.mjGEOM_BOX
        self._model.geom_dataid[self._proxy] = -1
        self._model.geom_size[self._proxy] = self.box.half_size
        self._model.geom_rbound[self._proxy] = np.linalg.norm(self.box.half_size)
        self._model.geom_aabb[self._proxy] = np.r_[np.zeros(3), self.box.half_size]
        # Signed distance queries require native CCD.
        self._model.opt.disableflags &= ~int(mujoco.mjtDisableBit.mjDSBL_NATIVECCD)

    def check(self, x: float, y: float, yaw: float) -> PlacementResult:
        """XY in meters, yaw in radians; Z and joints stay fixed."""
        if not np.all(np.isfinite([x, y, yaw])):
            raise ValueError("XY and yaw must be finite.")
        source = self._source_data
        if (not np.allclose(source.qpos[self._indexing.joint_q_adr], self._joint_pos,
                            atol=1e-10, rtol=0)
                or not np.isclose(source.qpos[self._indexing.free_joint_q_adr[2]],
                                  self.root_z, atol=1e-10, rtol=0)):
            raise ValueError("Robot posture changed; build a new placement checker.")
        c, s = np.cos(yaw), np.sin(yaw)
        rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        center = np.array([x, y, self.root_z]) + rotation @ self.box.center
        self._data.geom_xpos[:] = source.geom_xpos
        self._data.geom_xmat[:] = source.geom_xmat
        self._data.geom_xpos[self._proxy] = center
        self._data.geom_xmat[self._proxy] = rotation.ravel()

        ids = self.scene_geom_ids
        rotations = source.geom_xmat[ids].reshape(-1, 3, 3)
        bounds = self._model.geom_aabb[ids]
        centers = source.geom_xpos[ids] + np.einsum('nij,nj->ni', rotations, bounds[:, :3])
        half_sizes = np.einsum('nij,nj->ni', np.abs(rotations), bounds[:, 3:])
        box_half = np.abs(rotation) @ self.box.half_size
        nearby = np.all(np.abs(centers - center) <= half_sizes + box_half, axis=1)
        nearby |= self._model.geom_type[ids] == mujoco.mjtGeom.mjGEOM_PLANE
        collisions = []
        for geom in ids[nearby]:
            distance = mujoco.mj_geomDistance(self._model, self._data, self._proxy,
                                              int(geom), max(.001, self.tolerance), None)
            if distance < -self.tolerance:
                collisions.append(Collision(int(geom), self._model.geom(int(geom)).name,
                                            float(-distance)))
        return PlacementResult((float(x), float(y)), float(yaw), self.root_z,
                               center, tuple(collisions))

    def sample(self, xy_bounds, *, seed: int | None = None,
               max_attempts: int = 500) -> PlacementResult:
        """Sample within ((xmin, xmax), (ymin, ymax)); yaw in [-pi, pi).

        Does not check room membership or ground support. Leaves state unchanged.
        """
        bounds = np.asarray(xy_bounds, dtype=float)
        if (bounds.shape != (2, 2) or not np.all(np.isfinite(bounds))
                or np.any(bounds[:, 1] <= bounds[:, 0])):
            raise ValueError("Expected finite increasing ((xmin, xmax), (ymin, ymax)).")
        if max_attempts < 1:
            raise ValueError("max_attempts must be positive.")
        rng = np.random.default_rng(seed)
        for attempt in range(1, max_attempts + 1):
            xy = rng.uniform(bounds[:, 0], bounds[:, 1])
            result = self.check(*xy, rng.uniform(-np.pi, np.pi))
            if result.collision_free:
                return replace(result, attempts=attempt)
        raise RuntimeError(f"No collision-free bounding-box placement in {max_attempts} attempts.")


def find_robot_placement(
    scene: Path,
    robot: Articulation,
    xy_bounds: tuple[tuple[float, float], tuple[float, float]],
    *,
    seed: int | None = None,
    max_attempts: int = 500,
) -> tuple[tuple[float, float], float]:
    """Return (xy, yaw) using an independent query model; leave robot unmodified.

    Bounds: ((xmin, xmax), (ymin, ymax)), in meters. Yaw is in radians.
    """
    spec = mujoco.MjSpec.from_file(str(Path(scene).absolute()))
    spec.attach(robot.build_spec(), prefix=robot.prefix, frame=spec.worldbody.add_frame())
    model = spec.compile()
    data = mujoco.MjData(model)
    query_robot = copy.copy(robot)
    query_robot.initialize(model, data)
    mujoco.mj_forward(model, data)
    checker = PlacementChecker(model, data, query_robot)
    result = checker.sample(xy_bounds, seed=seed, max_attempts=max_attempts)
    return result.xy, result.yaw
