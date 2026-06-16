"""
twinnexus_policy.py
-------------------
π0.5 input/output transforms for the TwinNexus Admittance Platform.

Robot:    UR5e right arm + WSG32 gripper
Cameras:  overhead (D455) + wrist_right (D415)
Action:   7-DOF absolute joint positions [6 joints (rad) + 1 gripper (m)]

LeRobot dataset keys → π0.5 model keys:
    observation.images.overhead     → base_0_rgb
    observation.images.wrist_right  → left_wrist_0_rgb
    observation.state               → state  (7,)
    action                          → actions (T, 7)
    task                            → prompt
"""

import dataclasses

import einops
import numpy as np

from openpi import transforms
from openpi.models import model as _model


def make_twinnexus_example() -> dict:
    """Creates a random input example for testing the TwinNexus policy transform."""
    return {
        "observation/state": np.random.rand(7).astype(np.float32),
        "observation/image": np.random.randint(256, size=(480, 640, 3), dtype=np.uint8),
        "observation/wrist_image": np.random.randint(256, size=(480, 640, 3), dtype=np.uint8),
        "prompt": "pick up the screwdriver and place it in the paper box",
    }


def _parse_image(image) -> np.ndarray:
    """
    Normalize image to uint8 HWC format.
    LeRobot stores images as float32 CHW — this handles both.
    """
    image = np.asarray(image)
    if np.issubdtype(image.dtype, np.floating):
        image = (255 * image).astype(np.uint8)
    if image.ndim == 3 and image.shape[0] == 3:
        image = einops.rearrange(image, "c h w -> h w c")
    return image


@dataclasses.dataclass(frozen=True)
class TwinNexusInputs(transforms.DataTransformFn):
    """
    Maps TwinNexus LeRobot dataset observations to π0.5 model inputs.

    Camera assignment:
        base_0_rgb       ← observation.images.overhead   (D455 — best overall view)
        left_wrist_0_rgb ← observation.images.wrist_right (D415 — end-effector view)
        right_wrist_0_rgb← zeros (no left wrist camera yet)

    State: 7-dim [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3, gripper]
    """

    model_type: _model.ModelType = _model.ModelType.PI0

    def __call__(self, data: dict) -> dict:
        # Parse images — handles both float32 CHW and uint8 HWC
        overhead_image    = _parse_image(data["observation/image"])       # base view
        wrist_right_image = _parse_image(data["observation/wrist_image"]) # wrist view

        inputs = {
            # State: full 7-dim vector (6 joints rad + 1 gripper m)
            "state": np.asarray(data["observation/state"], dtype=np.float32),

            # Images: map to π0.5 slots
            "image": {
                "base_0_rgb":       overhead_image,
                "left_wrist_0_rgb": wrist_right_image,
                # No left wrist camera — pad with zeros
                "right_wrist_0_rgb": np.zeros_like(overhead_image),
            },

            # Masks: False = this slot is padding, model ignores it
            "image_mask": {
                "base_0_rgb":       np.True_,
                "left_wrist_0_rgb": np.True_,
                # π0 (not FAST) masks right wrist since it's padding
                "right_wrist_0_rgb": (
                    np.True_
                    if self.model_type == _model.ModelType.PI0_FAST
                    else np.False_
                ),
            },
        }

        # Actions only present during training
        if "actions" in data:
            inputs["actions"] = np.asarray(data["actions"], dtype=np.float32)

        # Task instruction
        if "prompt" in data:
            inputs["prompt"] = data["prompt"]

        return inputs


@dataclasses.dataclass(frozen=True)
class TwinNexusOutputs(transforms.DataTransformFn):
    """
    Maps π0.5 model outputs back to TwinNexus robot actions.

    π0.5 outputs 32-dim actions (model default).
    We take the first 7 dims: [6 joints (rad) + 1 gripper (m)].
    """

    def __call__(self, data: dict) -> dict:
        return {"actions": np.asarray(data["actions"][:, :7], dtype=np.float32)}