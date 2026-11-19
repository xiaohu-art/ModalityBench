from dataclasses import replace
from pathlib import Path

from .articulation import Articulation, ArticulationCfg
from .camera import CameraCfg


class G1(Articulation):
    xml_path = Path(__file__).resolve().parents[1] / "assets" / "robots" / "g1" / "g1.xml"

    def __init__(self, *, cameras: tuple[CameraCfg, ...] | None = None):
        cfg = self.default_config()
        super().__init__(cfg if cameras is None else replace(cfg, cameras=cameras))

    @classmethod
    def default_config(cls) -> ArticulationCfg:
        """mjlab KNEES_BENT_KEYFRAME; meters and radians."""
        return ArticulationCfg(
            xml_path=cls.xml_path,
            root_body="pelvis",
            prefix="g1/",
            root_pos=(0.0, 0.0, 0.76),
            joint_pos={
                ".*_hip_pitch_joint": -0.312,
                ".*_knee_joint": 0.669,
                ".*_ankle_pitch_joint": -0.363,
                ".*_elbow_joint": 0.6,
                ".*_shoulder_pitch_joint": 0.2,
                "left_shoulder_roll_joint": 0.2,
                "right_shoulder_roll_joint": -0.2,
            },
        )


class G1WithHeadCamera(G1):
    @classmethod
    def default_config(cls) -> ArticulationCfg:
        cfg = super().default_config()
        head_camera = CameraCfg(
            name="head_camera",
            body="torso_link",
            pos=(0.09, 0.0, 0.43),
            # Camera -Z faces torso +X; camera +Y aligns with torso +Z.
            quat=(0.5, 0.5, -0.5, -0.5),
        )
        return replace(cfg, cameras=(*cfg.cameras, head_camera))