from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path
import re

import mujoco
import numpy as np
from numpy.typing import NDArray

from .camera import CameraCfg


@dataclass(frozen=True)
class ArticulationCfg:
    """Local MJCF and root body name before attachment to a scene."""

    xml_path: Path
    root_body: str
    prefix: str = "robot/"
    root_pos: tuple[float, float, float] | None = None
    joint_pos: dict[str, float] = field(default_factory=dict)
    cameras: tuple[CameraCfg, ...] = ()


@dataclass(frozen=True)
class ArticulationIndexing:
    """Global IDs and qpos/qvel addresses belonging to one robot subtree."""

    root_body_id: int
    body_ids: NDArray[np.intp]
    joint_ids: NDArray[np.intp]
    joint_q_adr: NDArray[np.intp]
    joint_v_adr: NDArray[np.intp]
    free_joint_q_adr: NDArray[np.intp]
    free_joint_v_adr: NDArray[np.intp]
    camera_ids: NDArray[np.intp]

    @classmethod
    def from_model(cls, model: mujoco.MjModel, root_body: str) -> "ArticulationIndexing":
        root_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, root_body)
        if root_id <= 0:
            raise ValueError(f"Robot root must be a named body, not world: {root_body!r}")
        bodies = {root_id}
        # MuJoCo orders parents before children.
        for body_id in range(root_id + 1, model.nbody):
            if int(model.body_parentid[body_id]) in bodies:
                bodies.add(body_id)
        joints, q_adr, v_adr, free_q, free_v = [], [], [], [], []
        for joint_id in range(model.njnt):
            if int(model.jnt_bodyid[joint_id]) not in bodies:
                continue
            kind = model.jnt_type[joint_id]
            q = int(model.jnt_qposadr[joint_id])
            v = int(model.jnt_dofadr[joint_id])
            if kind == mujoco.mjtJoint.mjJNT_FREE:
                free_q.extend(range(q, q + 7))
                free_v.extend(range(v, v + 6))
            else:
                joints.append(joint_id)
                ball = kind == mujoco.mjtJoint.mjJNT_BALL
                q_adr.extend(range(q, q + (4 if ball else 1)))
                v_adr.extend(range(v, v + (3 if ball else 1)))
        cameras = [i for i in range(model.ncam) if int(model.cam_bodyid[i]) in bodies]
        arrays = [np.array(ids, dtype=np.intp) for ids in
                  (sorted(bodies), joints, q_adr, v_adr, free_q, free_v, cameras)]
        for array in arrays:
            array.setflags(write=False)
        return cls(root_id, *arrays)


class ArticulationData:
    """State copies without a batch dimension; joint state excludes the free joint.

    Quaternions: wxyz. ``_w``: world frame. Link velocities: [linear, angular]
    at the link origin. Requires current kinematics.
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
                 indexing: ArticulationIndexing):
        self._model = model
        self._data = data
        self.indexing = indexing

    @property
    def joint_pos(self) -> NDArray[np.float64]:
        return self._data.qpos[self.indexing.joint_q_adr].copy()

    @property
    def joint_vel(self) -> NDArray[np.float64]:
        return self._data.qvel[self.indexing.joint_v_adr].copy()

    @property
    def root_link_pos_w(self) -> NDArray[np.float64]:
        return self._data.xpos[self.indexing.root_body_id].copy()

    @property
    def root_link_quat_w(self) -> NDArray[np.float64]:
        return self._data.xquat[self.indexing.root_body_id].copy()

    @property
    def root_link_pose_w(self) -> NDArray[np.float64]:
        return np.concatenate((self.root_link_pos_w, self.root_link_quat_w))

    def _link_velocity(self, body_id: int) -> NDArray[np.float64]:
        velocity = np.empty(6)
        # XBODY selects the link origin; BODY selects the inertial origin.
        mujoco.mj_objectVelocity(self._model, self._data, mujoco.mjtObj.mjOBJ_XBODY,
                                body_id, velocity, 0)
        return np.concatenate((velocity[3:], velocity[:3]))

    @property
    def root_link_vel_w(self) -> NDArray[np.float64]:
        return self._link_velocity(self.indexing.root_body_id)

    @property
    def body_link_pos_w(self) -> NDArray[np.float64]:
        return self._data.xpos[self.indexing.body_ids].copy()

    @property
    def body_link_quat_w(self) -> NDArray[np.float64]:
        return self._data.xquat[self.indexing.body_ids].copy()

    @property
    def body_link_pose_w(self) -> NDArray[np.float64]:
        return np.concatenate((self.body_link_pos_w, self.body_link_quat_w), axis=1)

    @property
    def body_link_vel_w(self) -> NDArray[np.float64]:
        return np.stack([self._link_velocity(int(i)) for i in self.indexing.body_ids])


class Articulation:
    """Robot subtree sharing the simulator's model and data."""

    def __init__(self, cfg: ArticulationCfg):
        self.cfg = cfg

    @property
    def prefix(self) -> str:
        return self.cfg.prefix

    def build_spec(self) -> mujoco.MjSpec:
        spec = mujoco.MjSpec.from_file(str(self.cfg.xml_path.absolute()))
        for camera in self.cfg.cameras:
            spec.body(camera.body).add_camera(
                name=camera.name, pos=camera.pos, quat=camera.quat, fovy=camera.fovy,
                mode=mujoco.mjtCamLight.mjCAMLIGHT_FIXED,
            )
        return spec

    def initialize(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Bind to the composed model and apply the initial pose before mj_forward."""
        self.indexing = ArticulationIndexing.from_model(model, self.prefix + self.cfg.root_body)
        self.data = ArticulationData(model, data, self.indexing)
        self._sim_data = data
        self.joint_names = tuple(model.joint(int(i)).name.removeprefix(self.prefix)
                                 for i in self.indexing.joint_ids)
        self.body_names = tuple(model.body(int(i)).name.removeprefix(self.prefix)
                                for i in self.indexing.body_ids)
        self.camera_names = tuple(model.camera(int(i)).name.removeprefix(self.prefix)
                                  for i in self.indexing.camera_ids)
        if self.cfg.root_pos is not None:
            if self.indexing.free_joint_q_adr.size != 7:
                raise ValueError("Initial root position requires one free joint.")
            data.qpos[self.indexing.free_joint_q_adr] = (*self.cfg.root_pos, 1, 0, 0, 0)
        for pattern, value in self.cfg.joint_pos.items():
            local_ids, _ = self.find_joints(pattern)
            for joint_id in self.indexing.joint_ids[local_ids]:
                if int(model.jnt_type[joint_id]) not in (
                    int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE),
                ):
                    raise ValueError("Initial scalar joint positions require hinge/slide joints.")
                data.qpos[model.jnt_qposadr[joint_id]] = value

    @property
    def is_fixed_base(self) -> bool:
        return self.indexing.free_joint_q_adr.size == 0

    @property
    def num_joints(self) -> int:
        return len(self.joint_names)

    @property
    def num_bodies(self) -> int:
        return len(self.body_names)

    @staticmethod
    def _find(patterns: str | Sequence[str], names: tuple[str, ...]) -> tuple[list[int], list[str]]:
        patterns = [patterns] if isinstance(patterns, str) else patterns
        matches = set()
        for pattern in patterns:
            ids = [i for i, name in enumerate(names) if re.fullmatch(pattern, name)]
            if not ids:
                raise ValueError(f"No names match {pattern!r}")
            matches.update(ids)
        indices = sorted(matches)
        return indices, [names[i] for i in indices]

    def find_joints(self, patterns: str | Sequence[str]) -> tuple[list[int], list[str]]:
        """Regex full matches; return local joint IDs and names in model order.

        Hinge/slide IDs also index joint_pos/joint_vel. Ball joints occupy
        four position and three velocity coordinates per joint instead.
        """
        return self._find(patterns, self.joint_names)

    def find_bodies(self, patterns: str | Sequence[str]) -> tuple[list[int], list[str]]:
        """Regex full matches; return local IDs indexing body_link_* arrays."""
        return self._find(patterns, self.body_names)

    def find_cameras(self, patterns: str | Sequence[str]) -> tuple[list[int], list[str]]:
        """Regex full matches; local IDs index indexing.camera_ids."""
        return self._find(patterns, self.camera_names)

    def read_state(self) -> dict[str, float | NDArray[np.float64]]:
        """Detached snapshot; call on the simulation thread between steps."""
        return {
            "time": float(self._sim_data.time),
            "joint_pos": self.data.joint_pos,
            "joint_vel": self.data.joint_vel,
            "root_link_pose_w": self.data.root_link_pose_w,
            "root_link_vel_w": self.data.root_link_vel_w,
            "body_link_pose_w": self.data.body_link_pose_w,
            "body_link_vel_w": self.data.body_link_vel_w,
        }
