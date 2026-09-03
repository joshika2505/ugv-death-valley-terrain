"""
Semantic segmentation taxonomy and inference engines.
"""

from .taxonomy import SemanticClass, CLASS_COLORS_RGB, CLASS_COLORS_BGR, CLASS_COLORS, CLASS_NAMES
from .segmenter import BaseSegmenter, FeatureSegmenter, DeepSegmenter

__all__ = [
    "SemanticClass",
    "CLASS_COLORS",
    "CLASS_COLORS_RGB",
    "CLASS_COLORS_BGR",
    "CLASS_NAMES",
    "BaseSegmenter",
    "FeatureSegmenter",
    "DeepSegmenter",
]
