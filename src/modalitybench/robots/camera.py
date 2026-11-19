from dataclasses import dataclass


@dataclass(frozen=True)
class CameraCfg:
    """Body-relative pose: meters, quaternion wxyz, vertical FOV degrees."""

    name: str
    body: str
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    fovy: float = 60.0
