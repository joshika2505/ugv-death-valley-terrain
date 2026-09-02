"""
Semantic Segmenter implementations:
1. BaseSegmenter: Abstract interface
2. FeatureSegmenter: Fast feature/texture/color/depth segmenter for offline simulation & unit testing
3. DeepSegmenter: ONNX / TensorRT / PyTorch wrapper for PIDNet-S & MobileNetV4 models
"""

import abc
import os
from typing import Optional, Dict, Any, Tuple
import numpy as np
import cv2

from .taxonomy import SemanticClass


class BaseSegmenter(abc.ABC):
    """Abstract base class for semantic segmentation backbones."""

    @abc.abstractmethod
    def predict(self, rgb: np.ndarray, depth: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Segment an RGB image into class indices (0..3).
        
        Args:
            rgb: (H, W, 3) RGB image (uint8)
            depth: (H, W) Metric depth in meters (float)
            
        Returns:
            mask: (H, W) uint8 array of SemanticClass integers
        """
        pass


class FeatureSegmenter(BaseSegmenter):
    """
    Feature-based Segmenter utilizing color spaces (HSV), local textures,
    and depth cues to segment terrain into functional classes.
    """

    def __init__(
        self,
        grass_hue_range: Tuple[int, int] = (25, 85),
        soil_hue_range: Tuple[int, int] = (10, 30),
        min_saturation: int = 35,
    ):
        self.grass_hue_range = grass_hue_range
        self.soil_hue_range = soil_hue_range
        self.min_saturation = min_saturation

    def predict(self, rgb: np.ndarray, depth: Optional[np.ndarray] = None) -> np.ndarray:
        h, w, _ = rgb.shape
        mask = np.full((h, w), SemanticClass.FREE_DRIVABLE, dtype=np.uint8)

        # Convert to HSV color space for robust color/texture detection
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        hue = hsv[..., 0]
        sat = hsv[..., 1]
        val = hsv[..., 2]

        # 1. Soft Traversable (Vegetation / Grass): Hue in green-yellow range, moderate saturation
        grass_mask = (
            (hue >= self.grass_hue_range[0]) &
            (hue <= self.grass_hue_range[1]) &
            (sat >= self.min_saturation) &
            (val >= 30)
        )
        mask[grass_mask] = SemanticClass.SOFT_TRAVERSABLE

        # 2. Free Drivable: Dirt trails, light gray pavement, brown soil
        dirt_mask = (
            ((hue >= self.soil_hue_range[0]) & (hue <= self.soil_hue_range[1]) & (sat < 90)) |
            (sat < 30)  # Low saturation = pavement / gravel
        )
        mask[dirt_mask & ~grass_mask] = SemanticClass.FREE_DRIVABLE

        # 3. Rigid Obstacles: High texture variance, dark rock shadows, or stark color deviations
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        laplacian = cv2.Laplacian(gray, cv2.CV_64F)
        texture_var = cv2.GaussianBlur(np.abs(laplacian), (9, 9), 2.0)
        
        # High texture roughness on non-grass surfaces often indicates rocks/trunks
        rough_rigid = (texture_var > 25.0) & (sat < 60) & (val > 40) & (val < 200)
        mask[rough_rigid & ~grass_mask] = SemanticClass.RIGID_OBSTACLE

        return mask


class DeepSegmenter(BaseSegmenter):
    """
    Inference wrapper for trained deep learning segmentation backbones
    (e.g., PIDNet-S, MobileNetV4, SegFormer) via ONNX Runtime or TensorRT.
    """

    def __init__(self, model_path: Optional[str] = None, fallback_segmenter: Optional[BaseSegmenter] = None):
        self.model_path = model_path
        self.fallback = fallback_segmenter or FeatureSegmenter()
        self.session = None
        
        if model_path and os.path.exists(model_path):
            self._load_model(model_path)

    def _load_model(self, model_path: str) -> None:
        """Attempt to load ONNX model."""
        try:
            import onnxruntime as ort
            self.session = ort.InferenceSession(model_path, providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
        except Exception:
            self.session = None

    def predict(self, rgb: np.ndarray, depth: Optional[np.ndarray] = None) -> np.ndarray:
        if self.session is None:
            return self.fallback.predict(rgb, depth)

        # Standard preprocessing for segmentation backbones: Resize, Normalize, Transpose
        h_orig, w_orig, _ = rgb.shape
        img_resized = cv2.resize(rgb, (640, 480))
        img_norm = (img_resized.astype(np.float32) / 255.0 - [0.485, 0.456, 0.406]) / [0.229, 0.224, 0.225]
        input_tensor = np.transpose(img_norm, (2, 0, 1))[np.newaxis, ...].astype(np.float32)

        input_name = self.session.get_inputs()[0].name
        outputs = self.session.run(None, {input_name: input_tensor})
        logits = outputs[0][0]  # (C, H, W)
        pred = np.argmax(logits, axis=0).astype(np.uint8)

        # Resize back to original dimensions
        if (pred.shape[0] != h_orig) or (pred.shape[1] != w_orig):
            pred = cv2.resize(pred, (w_orig, h_orig), interpolation=cv2.INTER_NEAREST)

        return pred
